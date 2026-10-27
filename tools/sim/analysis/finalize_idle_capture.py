#!/usr/bin/env python3
"""Run a single final summary after a bounded idle capture; sends no ROS commands."""
import argparse,json,time,subprocess,sys
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',required=True);a=p.parse_args();folder=Path(a.folder);m=json.loads((folder/'manifest.json').read_text());deadline=m['started_wall_sec']+m['duration_wall_sec']+15
while time.time()<deadline:time.sleep(min(60,max(.1,deadline-time.time())))
subprocess.run([sys.executable,str(Path(__file__).with_name('summarize_idle_health.py')),'--folder',str(folder)],check=True)
