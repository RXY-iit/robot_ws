"""Bounded perception semantics and capability compilation, independent of ROS."""
import re


def target_config():
    from robot_skills.perception_capabilities import load_targets
    return load_targets()


def contains(text, word):
    pattern = re.escape(word).replace(r"\ ", r"[ _-]+")
    return bool(re.search(r"(?<![a-z0-9])" + pattern + r"(?![a-z0-9])", text.lower()))


def vocab(config):
    return sorted(set(config["objects"]) | set(config["yolo_open_vocab_classes"]))


def direct_object(name, config):
    if name in config["yolo_open_vocab_classes"]:
        return True
    entry = config["objects"].get(name, {})
    exact = set(entry.get("classes", [])) - set(entry.get("fallback_classes", []))
    return bool(exact & set(config["yolo_open_vocab_classes"]))


def aliases(name, config, group):
    return [name, *config[group].get(name, {}).get("aliases", [])]


def text_markers(config):
    return sorted({"word", "read", *config.get("text", {}),
                   *(a for item in config.get("text", {}).values() for a in item.get("aliases", []))},
                  key=len, reverse=True)


def literal_ocr_query(source, config, text_context=False):
    """Resolve only complete literal text expressions; never discard qualifiers."""
    source = source.strip()
    if re.fullmatch(r"shelf_label_0?[1-6]", source, re.I):
        return source.lower()
    if re.fullmatch(r"text_\w+", source, re.I):
        return "text_" + source[5:]
    source = re.sub(r"^(?:check|find|read)\s+", "", source, flags=re.I)
    markers = "|".join(re.escape(x) for x in text_markers(config))
    # A determiner before the text marker is grammar, not part of the literal.
    # Keep all other qualifiers so unsupported requirements still block binding.
    match = re.fullmatch(rf"(?:(?:a|an|the)\s+)?(?:{markers})[_ :]+(\w+)", source, re.I)
    if not match:
        match = re.fullmatch(rf"(\w+)[_ :]+(?:{markers})", source, re.I)
    if match:
        return "text_" + match.group(1)
    if text_context and source.isdigit():
        return "text_" + source
    return None


def parse_seed(query, config):
    # Keep compound attributes in one clause; only explicit conjunctions split it.
    parts = re.split(r"\s+(?:and|or)\s+|\s*[,、]\s*", query, flags=re.I)
    text_context = bool(re.search(r"shelf_label_|text_", query, re.I)) or any(contains(query, a) for a in text_markers(config))
    relation = "any" if re.search(r"\bor\b", query, re.I) else "all"
    ambiguous = bool(re.search(r"\band\b", query, re.I) and re.search(r"\bor\b", query, re.I))
    units = []
    for part in parts:
        if not part.strip():
            continue
        text = part.strip()
        colors = [name for name in config["colors"] if any(contains(text, a) for a in aliases(name, config, "colors"))]
        objects = [name for name in vocab(config) if any(contains(text, a) for a in aliases(name, config, "objects"))]
        # Prefer registered aliases over a substring class (e.g. wine glass).
        objects = sorted(objects, key=len, reverse=True)
        if len(colors) > 1 or (len(objects) > 1 and not all(contains(objects[0], obj) for obj in objects[1:])):
            ambiguous = True
        if colors and objects and text.lower() in set(colors) & set(objects):
            ambiguous = True
        describe = bool(re.search(r"danger|hazard|unsafe|describe|anything wrong|危険|危ない|安全|危险|描述", text, re.I))
        ocr_query = literal_ocr_query(text, config, text_context)
        ocr = bool(ocr_query) or any(contains(text, a) for a in text_markers(config))
        kind = ("image_describe" if describe else "ocr" if ocr else "composite" if colors and objects
                else "color" if colors else "object" if objects else "unknown")
        units.append({"source": text, "kind": kind, "object": objects[0] if objects else "none",
                      "source_object": objects[0] if objects else "none",
                      "color": colors[0] if colors else "none", "literal_ocr_query": ocr_query})
    return units, relation, ambiguous


def questions_for(units, config, relation):
    def choice(instructions, options):
        return {"type": "choice", "instructions": instructions, "criteria": {v: v for v in options}}
    questions = {
        "coverage": choice("Do these source clauses cover the complete requested perception goal, without dropping qualifiers?", ["complete", "unclear"]),
        "relation": choice("Are clauses all required or explicit alternatives? Do not split a color from its object.", ["all", "any", "unclear"]),
        "unit_count": choice("Count semantic requirements, not camera moves. A color plus a specific object category is two requirements in one same-instance clause. A color with a generic reference (thing/object/something) requests only one color requirement, not an unknown object class.", [str(i) for i in range(1, 13)] + ["unclear"]),
    }
    defaults = {"coverage": "complete", "relation": relation,
                "unit_count": str(sum(2 if u["kind"] == "composite" else 1 for u in units))}
    for i, unit in enumerate(units):
        source_context = (
            f"Answer ONLY for source fragment {unit['source']!r} (clause index {i}). "
            "Do not copy a category or color from another fragment. "
        )
        if unit.get("literal_ocr_query"):
            questions[f"kind_{i}"] = choice(
                f"Clause {i} is an explicit text-reading contract: {unit['source']!r}. "
                "shelf_label_NN denotes the printed shelf number, not a physical shelf object. "
                "text_X denotes literal text X. Number + letter/label also requests reading text. "
                "Choose OCR or unknown if the contract cannot be interpreted.", ["ocr", "unknown"])
            questions[f"object_{i}"] = choice("A literal OCR clause requests no physical object class.", ["none"])
            questions[f"color_{i}"] = choice("A literal OCR clause requests no color attribute.", ["none"])
            defaults.update({f"kind_{i}": "ocr", f"object_{i}": "none", f"color_{i}": "none"})
            continue
        questions[f"kind_{i}"] = choice(source_context + "Classify this fragment. Preserve scene interpretation/safety questions as image_describe, never as proof of safety from detection.",
                                       ["ocr", "object", "color", "composite", "image_describe", "unknown"])
        questions[f"kind_{i}"]["criteria"].update({
            "ocr": "Read printed text, letters, digits or labels; not an object detector class.",
            "object": "Detect a physical object category, such as bottle or car.",
            "color": "Detect a configured color without a specific object category; generic thing/object/something adds no category constraint.",
            "composite": "Check color and a specific object category on the same instance; a generic thing is not a category.",
            "image_describe": "Interpret a scene, hazards or relationships beyond detection.",
            "unknown": "Cannot resolve all requested semantics."})
        questions[f"object_{i}"] = choice(source_context + "Choose its requested object; recognized names include unsupported goals such as box. "
            "This is semantic identification, NOT proof of detector support. Choose none for generic references such as thing/object/something without a specific category. "
            "Use unknown for an unlisted specific category; never pick the nearest available class.", ["none", "unknown", *vocab(config)])
        questions[f"color_{i}"] = choice(source_context + "Choose its explicitly requested color. Choose none when no color is specified. "
            "Unknown means a color IS specified but is unsupported, such as light blue; never silently replace it with blue.", ["none", "unknown", *config["colors"]])
        questions[f"color_{i}"]["criteria"].update(
            none="No color requirement in this fragment.",
            unknown="An explicit color/shade is not among the supported choices.")
        for key in ("kind", "object", "color"):
            defaults[f"{key}_{i}"] = unit[key]
    return questions, defaults


def compile_checks(units, config, approved=None, *, generic_color_reference=False):
    checks, gaps, proposals = [], [], []
    approved = approved or {}
    for i, unit in enumerate(units):
        kind, source, obj, color = (unit[k] for k in ("kind", "source", "object", "color"))
        if kind == "image_describe":
            gaps.append({"source": source, "kind": kind, "reason": "image_describe API not configured"})
            continue
        if kind == "ocr":
            # The reviewed OCR kind supplies context even when Mission bound a
            # bare numeric token. Extra qualifiers must still resolve in full.
            query = unit.get("literal_ocr_query") or literal_ocr_query(source, config, text_context=True)
            if query and config.get("text_contract", {}).get("scripts") == ["latin", "digits"] and not query.isascii():
                query = None
            if query:
                checks.append({"id": f"c{i}", "kind": "ocr", "query": query, "provider": "yolo11s_ocr"})
            else:
                gaps.append({"source": source, "kind": kind, "reason": "OCR token or extra qualifiers unresolved; ask user"})
            continue
        valid_color = color in config["colors"] and any(contains(source, a) for a in aliases(color, config, "colors"))
        valid_object = obj != "none" and direct_object(obj, config)
        mentioned = any(contains(source, a) for a in aliases(obj, config, "objects"))
        if approved.get(str(i)) == obj:
            mentioned = True
        if valid_object and not mentioned:
            proposals.append({"unit": str(i), "object": obj, "source": source})
            valid_object = False
        if obj != "none" and not valid_object:
            reason = (f"Object interpretation requires confirmation: {source} -> {obj}"
                      if direct_object(obj, config) else f"No approved exact detector for {obj}")
            gaps.append({"source": source, "kind": "object", "reason": reason})
            for candidate in config["objects"].get(obj, {}).get("fallback_classes", []):
                if direct_object(candidate, config):
                    proposals.append({"unit": str(i), "object": candidate, "source": source})
        if kind in {"color", "composite"} and not valid_color:
            gaps.append({"source": source, "kind": "color", "reason": "No configured HSV color matching the request"})
        # Unknown qualifiers must survive even if a known noun/color was extracted.
        residue = source
        original = unit.get("source_object", obj) if approved.get(str(i)) else obj
        for name, group in ((original, "objects"), (obj, "objects"), (color, "colors")):
            for alias in sorted(aliases(name, config, group), key=len, reverse=True):
                pattern = re.escape(alias).replace(r"\ ", r"[ _-]+")
                residue = re.sub(r"(?<![a-z0-9])" + pattern + r"(?![a-z0-9])", " ", residue, flags=re.I)
        residue = re.sub(r"\b(find|search|check|detect|a|an|the|object|color|please)\b|[_-]", " ", residue, flags=re.I).strip()
        if generic_color_reference and kind == "color" and obj == "none":
            # Only category-free color requests may drop generic noun references.
            # Specific categories and modifiers must remain unsupported evidence.
            residue = re.sub(r"\b(thing|things|something|objects)\b", " ", residue, flags=re.I).strip()
        if approved.get(str(i)) and kind == "object" and re.fullmatch(r"\w+", source):
            residue = ""
        if (approved.get(str(i)) == obj and unit.get("source_object") == "none"
                and kind in {"object", "composite"} and re.fullmatch(r"[A-Za-z][A-Za-z0-9]*", residue)):
            # Explicit approval can resolve one unknown noun, not extra shade/attribute phrases.
            residue = ""
        if residue:
            gaps.append({"source": source, "kind": "qualifier", "reason": f"Unresolved qualifier: {residue}"})
        if valid_color and valid_object and kind == "composite":
            checks.append({"id": f"c{i}", "kind": "composite", "query": f"{color} {obj}", "provider": "yolo11s", "relation": "same_instance",
                           "color_query": f"color_{color}"})
        else:
            if valid_color and kind in {"color", "composite"}:
                checks.append({"id": f"c{i}:color", "kind": "color", "query": f"color_{color}", "provider": "yolo11s"})
            if valid_object and kind in {"object", "composite"}:
                checks.append({"id": f"c{i}:object", "kind": "object", "query": obj, "provider": "yolo11s"})
        if kind == "unknown":
            gaps.append({"source": source, "kind": kind, "reason": "Unresolved perception requirement"})
    return checks, gaps, proposals
