"""Exercise the real startup fallback with mocked ROS lifecycle transitions."""
from pathlib import Path
import subprocess
import pytest

ROOT=Path(__file__).resolve().parents[3]

@pytest.mark.parametrize('initial,transitions',[
    ('inactive [2]',['activate']),('unconfigured [1]',['configure','activate']),('active [3]',[])])
def test_map_fallback_does_not_confuse_inactive_with_active(tmp_path,initial,transitions):
    script=(ROOT/'tools/sim/open_sim_nav2_windows.sh').read_text()
    start=script.index('for node in /map_server ')
    loop=script[start:script.index('\ndone',start)+len('\ndone')]
    (tmp_path/'map-state').write_text(initial+'\n')
    mocks=r'''
    sleep() { :; }
    timeout() { shift; "$@"; }
    ros2() {
      if [[ "$1 $2" == "lifecycle get" ]]; then
        if [[ "$3" == "/map_server" ]]; then cat map-state; else echo "active [3]"; fi
      elif [[ "$1 $2" == "lifecycle set" ]]; then
        echo "$3 $4" >> transitions
        if [[ "$4" == "configure" ]]; then echo "inactive [2]" > map-state; fi
        if [[ "$4" == "activate" ]]; then echo "active [3]" > map-state; fi
      else return 2; fi
    }
    '''
    subprocess.run(['bash','-c',mocks+'\n'+loop],cwd=tmp_path,check=True,capture_output=True,text=True)
    actual=(tmp_path/'transitions').read_text().splitlines() if (tmp_path/'transitions').exists() else []
    assert actual==['/map_server '+step for step in transitions]
    assert (tmp_path/'map-state').read_text().strip()=='active [3]'
