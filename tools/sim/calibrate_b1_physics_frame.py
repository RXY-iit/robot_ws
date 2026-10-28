#!/usr/bin/env python3
"""Offline SE2 calibration from a measured coincident pose pair; never publish TF."""
import argparse,json,math
from pathlib import Path


def calibration(physics,map_pose):
    a=map_pose[2]-physics[2];c,s=math.cos(a),math.sin(a)
    return [map_pose[0]-c*physics[0]+s*physics[1],map_pose[1]-s*physics[0]-c*physics[1],math.atan2(math.sin(a),math.cos(a))]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--physics-pose',type=float,nargs=3,required=True);p.add_argument('--map-pose',type=float,nargs=3,required=True)
    p.add_argument('--basis',choices=['independent_survey','clean_initial_alignment'],required=True);p.add_argument('--evidence',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if not all(math.isfinite(v) for v in a.physics_pose+a.map_pose):p.error('Finite coincident poses required')
    value=dict(map_from_physics_se2=calibration(a.physics_pose,a.map_pose),basis=a.basis,evidence=a.evidence,
        physics_pose=a.physics_pose,map_pose=a.map_pose,
        accuracy_claim='independently aligned physics pose' if a.basis=='independent_survey' else 'relative drift after clean initial alignment; not absolute map accuracy')
    with a.output.open('x') as f:json.dump(value,f,indent=2)
if __name__=='__main__':main()
