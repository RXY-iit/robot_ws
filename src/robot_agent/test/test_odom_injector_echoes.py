from geometry_msgs.msg import TransformStamped
from inject_odom_error import TransformEchoes


def transform(stamp, x=0.):
    t=TransformStamped();t.header.stamp.sec=stamp;t.header.frame_id='odom'
    t.child_frame_id='base_footprint';t.transform.translation.x=x;t.transform.rotation.w=1.
    return t


def test_delayed_own_echoes_are_consumed_once_but_duplicates_are_conflicts():
    echoes=TransformEchoes()
    for stamp in range(5):echoes.expect(transform(stamp))
    for stamp in [3,1,0,4,2]:assert echoes.consume(transform(stamp))
    assert not echoes.consume(transform(3))


def test_different_transform_at_same_stamp_is_not_our_echo():
    echoes=TransformEchoes();echoes.expect(transform(5))
    assert not echoes.consume(transform(5,1.))
    assert echoes.consume(transform(5))
