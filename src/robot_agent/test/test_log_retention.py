import importlib.util
import os
from pathlib import Path

spec = importlib.util.spec_from_file_location('prune_debug_logs', Path(__file__).resolve().parents[3] / 'tools/prune_debug_logs.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_prune_protects_live_pinned_bags_recent_and_symlinks(tmp_path):
    for name in ('old', 'recent', 'pinned', 'bag', 'live'):
        p = tmp_path / name
        p.mkdir()
        (p / 'log.json').write_text('{}')
    (tmp_path / 'pinned' / 'KEEP').touch()
    (tmp_path / 'bag' / 'data.mcap').touch()
    (tmp_path / 'live' / f'.pid-{os.getpid()}').touch()
    for p in tmp_path.rglob('*'):
        os.utime(p, (0, 0))
    os.utime(tmp_path / 'recent', (1000000, 1000000))
    (tmp_path / 'link').symlink_to(tmp_path / 'old', target_is_directory=True)
    found = list(module.candidates(tmp_path, 7, now=1000000))
    assert [p.name for p, size in found] == ['old']
    assert (tmp_path / 'old').exists()  # preview only
