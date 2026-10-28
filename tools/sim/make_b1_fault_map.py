#!/usr/bin/env python3
"""Create an offline localization-only PLY variant and immutable geometry manifest."""
import argparse, hashlib, json, math
from pathlib import Path
import numpy as np


def deform(points, roi, normal, spacing, fraction, voxel, seed):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():raise ValueError('Finite Nx3 map required')
    lo,hi=np.asarray(roi[:3]),np.asarray(roi[3:]);normal=np.asarray(normal,dtype=float)
    if (not np.isfinite([*roi,*normal,spacing,fraction,voxel]).all() or np.any(lo>=hi)
            or np.linalg.norm(normal)==0 or spacing<0 or not 0<=fraction<=1 or voxel<0):raise ValueError('Invalid map geometry parameters')
    mask=np.all((points>=lo)&(points<=hi),axis=1);ids=np.flatnonzero(mask)
    if not len(ids):raise ValueError('ROI contains no map points')
    normal=normal/np.linalg.norm(normal)
    chosen=np.random.default_rng(seed).choice(ids,size=round(len(ids)*fraction),replace=False)
    if fraction and (not len(chosen) or not spacing):raise ValueError('Double wall requires nonzero spacing and duplicated points')
    result=np.concatenate((points,points[chosen]+spacing*normal))
    before_voxel=len(result)
    if voxel:
        _,index=np.unique(np.floor(result/voxel).astype(np.int64),axis=0,return_index=True)
        result=result[np.sort(index)]
    return result,dict(input_points=len(points),roi_points=len(ids),duplicated_points=len(chosen),
        actual_duplicate_fraction=len(chosen)/len(ids),normal=normal.tolist(),roi=roi,
        spacing_m=spacing,voxel_m=voxel,pre_voxel_points=before_voxel,output_points=len(result),
        roi_point_density_per_m3=len(ids)/float(np.prod(hi-lo)),seed=seed,
        scope='localization PLY only; Nav2 grid and Isaac physical scene unchanged')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--roi',type=float,nargs=6,required=True,metavar=('X0','Y0','Z0','X1','Y1','Z1'))
    p.add_argument('--normal',type=float,nargs=3,required=True);p.add_argument('--spacing',type=float,default=.15)
    p.add_argument('--fraction',type=float,default=.5);p.add_argument('--voxel',type=float,default=0);p.add_argument('--seed',type=int,default=20261028);a=p.parse_args()
    manifest=a.output.with_suffix('.manifest.json')
    if a.output.exists() or manifest.exists() or a.output.resolve()==a.source.resolve():p.error('New output/manifest paths required; source never overwritten')
    import open3d as o3d
    cloud=o3d.io.read_point_cloud(str(a.source));result,details=deform(np.asarray(cloud.points),a.roi,a.normal,a.spacing,a.fraction,a.voxel,a.seed)
    generated=o3d.geometry.PointCloud();generated.points=o3d.utility.Vector3dVector(result)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    if not o3d.io.write_point_cloud(str(a.output),generated):raise RuntimeError('PLY write failed')
    details.update(source=str(a.source.resolve()),output=str(a.output.resolve()),source_sha256=hashlib.sha256(a.source.read_bytes()).hexdigest(),output_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),attributes='XYZ only; GICP input geometry')
    manifest.write_text(json.dumps(details,indent=2)+'\n');print(manifest)
if __name__=='__main__':main()
