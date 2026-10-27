from robot_agent.nodes import dashboard_node


def test_symlink_install_serves_assets_next_to_real_index(tmp_path, monkeypatch):
    dist = tmp_path / "dist"
    (dist / "assets").mkdir(parents=True)
    (dist / "index.html").write_text('<script src="/assets/new.js"></script>')
    (dist / "assets" / "new.js").write_text("window.loaded = true")
    share = tmp_path / "share"
    (share / "web" / "assets").mkdir(parents=True)
    (share / "web" / "index.html").symlink_to(dist / "index.html")
    monkeypatch.setattr(dashboard_node, "get_package_share_directory", lambda _: str(share))
    assert dashboard_node._find_web_dist() == dist
    assert (dashboard_node._find_web_dist() / "assets" / "new.js").is_file()
