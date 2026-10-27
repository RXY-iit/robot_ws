import importlib.util
from pathlib import Path
import struct
import math
import pytest
from sensor_msgs.msg import PointCloud2,PointField
spec=importlib.util.spec_from_file_location('cloud_inject',Path(__file__).resolve().parents[3]/'tools/sim/inject_gicp_cloud.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def cloud(big=False):
    c=PointCloud2();c.height=2;c.width=2;c.point_step=16;c.row_step=36;c.is_bigendian=big
    c.header.frame_id='livox_frame';c.header.stamp.sec=12
    c.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(['x','y','z','intensity'])]
    buf=bytearray(b'P'*72)
    for row in range(2):
        for col in range(2):struct.pack_into(('>' if big else '<')+'ffff',buf,row*36+col*16,3,4,0,42)
    c.data=bytes(buf);return c

def test_envelope():
    assert [m.strength(t,30,15,10,.3) for t in [-1,0,15,30,45,50,55,100]]==pytest.approx([0,0,.15,.3,.3,.15,0,0])

@pytest.mark.parametrize('big',[False,True])
def test_layout_and_seed(big):
    c=cloud(big);before=bytes(c.data)
    assert m.corrupt_cloud(c,0,1) is c
    a=m.corrupt_cloud(c,.3,42);b=m.corrupt_cloud(c,.3,42)
    assert bytes(a.data)==bytes(b.data)!=before
    assert bytes(c.data)==before
    assert a.header==c.header and a.fields==c.fields and a.row_step==c.row_step
    for row in range(2):
        assert bytes(a.data)[row*36+32:row*36+36]==b'PPPP'
        for col in range(2):
            x,y,z,i=struct.unpack_from(('>' if big else '<')+'ffff',bytes(a.data),row*36+col*16)
            assert i==42 and z==0 and x/y==pytest.approx(.75)

def test_invalid_and_nonfinite_points():
    c=cloud();buf=bytearray(c.data);struct.pack_into('<f',buf,0,float('nan'));c.data=bytes(buf)
    a=m.corrupt_cloud(c,.3,1)
    assert bytes(a.data)[:16]==bytes(c.data)[:16]
    c.fields=[]
    with pytest.raises(ValueError):m.corrupt_cloud(c,.1,1)

def test_bias_rotation_and_sector():
    c=cloud()
    a=m.corrupt_cloud(c,0,1,bias=1,yaw_deg=90)
    x,y,z,intensity=struct.unpack_from('<ffff',bytes(a.data),0)
    assert (x,y,z,intensity)==pytest.approx((-3,3,0,42))
    a=m.corrupt_cloud(c,0,1,sector_deg=180)
    assert all(math.isnan(v) for v in struct.unpack_from('<fff',bytes(a.data),0))
    assert struct.unpack_from('<f',bytes(a.data),12)[0]==42
    assert a.is_dense is False
    assert m.corrupt_cloud(c,0,1,bias=0,yaw_deg=0,sector_deg=0) is c

@pytest.mark.parametrize('big',[False,True])
def test_measured_perturbation_and_occlusion(big):
    c=cloud(big);stats={}
    assert m.corrupt_cloud(c,0,1,metrics=stats) is c
    assert stats['input_valid_points']==4
    assert stats['radial_noise_rmse_m']==0
    stats={}
    m.corrupt_cloud(c,0,1,bias=1,metrics=stats)
    assert stats['xyz_perturbation_rmse_m']==pytest.approx(1)
    stats={}
    m.corrupt_cloud(c,.1,1,sector_deg=180,metrics=stats)
    assert stats['occluded_fraction_valid']==1
    assert stats['output_valid_points']==0
    assert stats['radial_noise_rmse_m'] is None


def test_partial_occlusion_denominator_excludes_invalid_input():
    c=cloud();buf=bytearray(c.data)
    struct.pack_into('<fff',buf,0,-3.,4.,0.)
    struct.pack_into('<f',buf,16,float('nan'))
    c.data=bytes(buf);stats={}
    m.corrupt_cloud(c,0,1,sector_deg=180,metrics=stats)
    assert stats['input_valid_points']==3
    assert stats['occluded_valid_points']==2
    assert stats['occluded_fraction_valid']==pytest.approx(2/3)


def test_vector_translation_preserves_layout_and_measures_displacement():
    c=cloud();stats={}
    out=m.corrupt_cloud(c,0,1,translation=[0,1.5,0],metrics=stats)
    assert struct.unpack_from('<ffff',bytes(out.data),0)==pytest.approx((3,5.5,0,42))
    assert stats['xyz_perturbation_rmse_m']==pytest.approx(1.5)
    with pytest.raises(ValueError):m.corrupt_cloud(c,0,1,translation=[0,float('nan'),0])
