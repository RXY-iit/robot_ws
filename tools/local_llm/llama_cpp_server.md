# llama.cpp Parser Service

This service is intentionally outside the ROS2 packages. It is a local parser backend for L3 only.

## Role

```text
User text
  -> deterministic parser
  -> SentenceTransformer parser
  -> llama.cpp fallback
  -> MissionIntent JSON
```

The llama.cpp parser must not output ROS topics, coordinates, `/cmd_vel`, or direct motor commands.
It only returns a typed intent. L2 resolves targets and enforces safety.

## Recommended First Setup

Use a small Japanese/English instruct GGUF model, for example a Qwen-family 3B instruct model.
Keep the exact model configurable in `src/robot_agent/config/model_policy.yaml`.

Example server shape:

```bash
./llama-server \
  -m /path/to/model.gguf \
  --host 127.0.0.1 \
  --port 8080 \
  --ctx-size 4096
```

The agent expects an OpenAI-compatible endpoint:

```text
http://127.0.0.1:8080/v1/chat/completions
```

## Safety Contract

Manual endpoint test:

```bash
curl http://127.0.0.1:8080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "model": "qwen2.5-3b-instruct-q4_k_m",
    "temperature": 0,
    "response_format": {"type": "json_object"},
    "messages": [
      {
        "role": "system",
        "content": "You are a robot command parser. Output one JSON object only. Do not apologize. Do not chat. Do not explain."
      },
      {
        "role": "user",
        "content": "Allowed task_type values: navigate_to_object, navigate_to_place, search_object_near_place, search_object_here, inspect_place, robot_status, list_known_targets, cancel, move_lift_bounded, move_relative, ask_human. Required fields: task_type,target,relation,object_query,constraints,requires_confirmation. Parse this command: 06棚に赤い箱がありますか"
      }
    ]
  }'
```

Expected shape:

```json
{
  "task_type": "search_object_near_place",
  "target": "06棚",
  "relation": "near",
  "object_query": "赤い箱",
  "constraints": {},
  "requires_confirmation": false
}
```

Valid fallback output is JSON only:

```json
{
  "task_type": "search_object_near_place",
  "target": "06棚",
  "relation": "near",
  "object_query": "06番号",
  "constraints": {},
  "requires_confirmation": false
}
```

Invalid output:

```json
{
  "task_type": "publish_topic",
  "topic": "/cmd_vel"
}
```

Unknown or ambiguous text should return:

```json
{
  "task_type": "ask_human",
  "target": null,
  "relation": null,
  "object_query": null,
  "constraints": {},
  "requires_confirmation": false
}
```
