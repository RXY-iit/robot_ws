"""CLI integer-looking coordinates must match rclpy DOUBLE declarations."""
import ast
from pathlib import Path
import pytest
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

LAUNCH=Path(__file__).resolve().parents[1]/'launch/fast_lio_localization.launch.py'


@pytest.mark.parametrize('raw',['0','0.0','-2','1e-3'])
def test_coordinate_and_crop_overrides_are_always_double(raw,monkeypatch,tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'ros-log'))
    tree=ast.parse(LAUNCH.read_text())
    keys={'initial_x','initial_y','initial_yaw','scan_min_height','scan_max_height'}
    expressions={k.value:v for node in ast.walk(tree) if isinstance(node,ast.Dict)
                 for k,v in zip(node.keys,node.values) if isinstance(k,ast.Constant) and k.value in keys}
    assert set(expressions)==keys
    context=LaunchContext()
    for name,expr in expressions.items():
        context.launch_configurations[name]=raw
        value=eval(compile(ast.Expression(expr),str(LAUNCH),'eval'),
                   dict(ParameterValue=ParameterValue,LaunchConfiguration=LaunchConfiguration))
        result=value.evaluate(context)
        assert type(result) is float and result==float(raw)
