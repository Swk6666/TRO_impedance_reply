"""Render saved joint trajectories in MuJoCo, without rerunning control.

Use MUJOCO_GL=egl on a headless Linux workstation. MP4 needs ffmpeg.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

import mujoco
import numpy as np
from matplotlib.image import imsave

from .model import PlantParameters,build_model,home_data
from .scenarios import Scenario
from .experiment import source_hash


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--image',type=Path)
    parser.add_argument('--time',type=float,default=15.)
    parser.add_argument('--video',type=Path)
    parser.add_argument('--speed',type=float,default=2.,help='video playback speed relative to simulation')
    parser.add_argument('--fps',type=int,default=30)
    args=parser.parse_args()
    if not args.image and not args.video:parser.error('choose --image and/or --video')
    if args.speed<=0 or args.fps<=0:parser.error('speed and fps must be positive')
    s=json.loads((args.run/'summary.json').read_text())
    if s['source_sha256']!=source_hash():
        raise ValueError('Runtime/scenario code differs from this recording; check out its recorded version first')
    a=np.load(args.run/'trajectory.npz')
    scenario=Scenario(s['experiment'],a['desired'][0].astype(float),s['trial'])
    model=build_model(PlantParameters(**s['plant']),environment=scenario.environment())
    # Display-only transparency exposes the TCP behind the obstacle; these
    # colors do not change the geometry used in the recorded experiment.
    for name in ('wall','threshold'):
        geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name)
        if geom>=0:model.geom_rgba[geom,3]=.35
    for name in ('tool_shaft','tool_tip'):
        model.geom_rgba[model.geom(name).id]=[.9,.15,.08,1]
    model.vis.global_.offwidth=960;model.vis.global_.offheight=720
    data=home_data(model)
    camera=mujoco.MjvCamera()
    camera.lookat[:]=[.35,0,.40];camera.distance=1.65
    camera.azimuth=135;camera.elevation=-22
    def render_at(renderer,t):
        index=min(int(np.searchsorted(a['time'],t)),len(a['time'])-1)
        data.qpos[:]=a['q'][index];data.qvel[:]=a['qvel'][index]
        mujoco.mj_forward(model,data)
        renderer.update_scene(data,camera=camera)
        return renderer.render()
    with mujoco.Renderer(model,height=720,width=960) as renderer:
        if args.image:
            args.image.parent.mkdir(parents=True,exist_ok=True)
            imsave(args.image,render_at(renderer,args.time))
            print(args.image)
        if args.video:
            ffmpeg=shutil.which('ffmpeg')
            if not ffmpeg:raise RuntimeError('Install ffmpeg to write MP4 replay')
            args.video.parent.mkdir(parents=True,exist_ok=True)
            command=[ffmpeg,'-y','-loglevel','error','-f','rawvideo','-vcodec','rawvideo',
                     '-s','960x720','-pix_fmt','rgb24','-r',str(args.fps),'-i','-',
                     '-an','-c:v','libx264','-threads','2','-pix_fmt','yuv420p','-crf','23',str(args.video)]
            process=subprocess.Popen(command,stdin=subprocess.PIPE)
            try:
                for t in np.arange(0,a['time'][-1],args.speed/args.fps):
                    process.stdin.write(render_at(renderer,t).tobytes())
            finally:process.stdin.close()
            if process.wait()!=0:raise RuntimeError('ffmpeg failed')
            print(args.video)


if __name__=='__main__':main()
