"""Recover plotted (not raw experimental) C curves from the supplied PDF vectors.

Calibrations are for Fig. 9, PDF page 15 of DOI 10.1109/TRO.2026.3741518.
Poppler converts PDF to SVG. Polyline vertices may have been simplified by
its original exporter; vertex means and continuous-time statistics differ.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import numpy as np

NS='{http://www.w3.org/2000/svg}'
COLORS={
    'admittance':'rgb(46.665955%,67.451477%,18.823242%)',
    'bidirectional':'rgb(0%,44.7052%,74.118042%)',
    'hinf':'rgb(85.098267%,32.548523%,9.803772%)',
    'desired':'rgb(0%,0%,0%)',
    'beta':'rgb(30.195618%,30.195618%,30.195618%)',
}
BOUNDS={'force':(66.502081,392.001598,399.393983,656.647314),
        'beta':(485.750575,811.250092,61.820927,319.074259),
        'error_x':(71.541249,397.040766,737.376310,994.616852),
        'error_y':(485.750575,811.250092,737.376310,994.616852),
        'xy':(71.950521,397.450038,1075.294689,1332.023640)}


def paths(surface,color,bounds,dashed=None):
    result=[]
    for path in surface.iter(NS+'path'):
        if 'stroke:'+COLORS[color]+';' not in path.get('style',''):
            continue
        if dashed is not None and ('stroke-dasharray:' in path.get('style','')) != dashed:
            continue
        commands=path.get('d','')
        if set(re.findall('[A-Za-z]',commands))-{'M','L'}:
            continue
        numbers=re.findall(r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?',commands)
        if len(numbers)%2:
            continue
        points=np.array(numbers,dtype=float).reshape(-1,2)
        # Short colored legend samples do not belong to plotted data. Beta
        # has no same-color legend; retain any flat segments if present.
        if len(points)<(2 if color=='beta' else 20):
            continue
        xmin,xmax,ymin,ymax=bounds
        if np.all((points[:,0]>=xmin-.1)&(points[:,0]<=xmax+.1)&
                  (points[:,1]>=ymin-.1)&(points[:,1]<=ymax+.1)):
            result.append(points)
    return result


def time_value(segments,bounds,minimum,maximum):
    a=np.concatenate(segments)
    xmin,xmax,ymin,ymax=bounds
    a=np.c_[(a[:,0]-xmin)/(xmax-xmin)*30,
             minimum+(a[:,1]-ymin)/(ymax-ymin)*(maximum-minimum)]
    if np.any(np.diff(a[:,0]) < -1e-6):
        raise ValueError('Plot paths are not in chronological order')
    return a


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pdf',type=Path,default=Path('Nonlinear_mathcal_H_infty__Inverse-Optimal_Impedance_Control_for_Robotic_Systems.pdf'))
    parser.add_argument('--output',type=Path,default=Path('outputs/paper-c-digitized'))
    args=parser.parse_args()
    with tempfile.TemporaryDirectory() as temp:
        svg=Path(temp)/'page15.svg'
        subprocess.run(['pdftocairo','-svg','-f','15','-l','15',str(args.pdf),str(svg)],check=True)
        root=ET.parse(svg).getroot()
    candidates=[g for g in root.iter(NS+'g') if len(list(g))>1000]
    surface=max(candidates,key=lambda g:len(list(g)))
    arrays={};statistics={};force_segments={}
    for color in ('admittance','bidirectional','hinf','desired'):
        segments=paths(surface,color,BOUNDS['force']);force_segments[color]=segments
        a=time_value(segments,BOUNDS['force'],-10,60)
        assert len(segments)==24 and a[0,0]<.02 and a[-1,0]>29.98
        arrays['force_'+color]=a
        residual=a[:,1]-10-3*np.sin(a[:,0])
        statistics[color]=dict(vertices=len(a),segments=len(segments),
            force_min_N=float(a[:,1].min()),force_max_N=float(a[:,1].max()),
            plotted_force_rmse_N=float(np.sqrt(np.trapezoid(residual**2,a[:,0])/(a[-1,0]-a[0,0]))))
    assert statistics['desired']['plotted_force_rmse_N']<.01, 'Axis calibration does not recover 10+3sin(t)'
    beta=time_value(paths(surface,'beta',BOUNDS['beta']),BOUNDS['beta'],0,1)
    arrays['beta']=beta
    mask=(beta[:,0]>=13)&(beta[:,0]<=19)
    statistics['beta']=dict(vertices=len(beta),plotted_mean_13_19=float(np.trapezoid(beta[mask,1],beta[mask,0])/(beta[mask,0][-1]-beta[mask,0][0])),
                            caveat='Statistics of the rendered polyline, NOT the 4 kHz internal beta signal.')
    for color in ('admittance','bidirectional','hinf'):
        for axis,limits in [('x',(-.06,.08)),('y',(-.08,.04))]:
            for dashed,label in [(False,'error'),(True,'equilibrium')]:
                segments=paths(surface,color,BOUNDS['error_'+axis],dashed=dashed)
                assert len(segments)==24, (color,axis,label,len(segments))
                arrays[f'{label}_{axis}_{color}']=time_value(segments,BOUNDS['error_'+axis],*limits)
        t=np.arange(.01,30,.01)
        gap=np.column_stack([np.interp(t,*arrays[f'error_{axis}_{color}'].T)-
                             np.interp(t,*arrays[f'equilibrium_{axis}_{color}'].T) for axis in 'xy'])
        window=(t>=13)&(t<19)
        statistics[color]['plotted_xy_equilibrium_gap_13_19_mm']=float(np.sqrt(np.mean(np.sum(gap[window]**2,axis=1)))*1000)
        statistics[color]['plotted_xy_equilibrium_bias_13_19_mm']=float(np.linalg.norm(np.mean(gap[window],axis=0))*1000)
    for color in ('admittance','bidirectional','hinf','desired'):
        segments=paths(surface,color,BOUNDS['xy'])
        converted=[]
        for points in segments:
            # Axis x is vertical, axis y decreases left-to-right.
            converted.append(np.c_[.55+(points[:,1]-1078.223545)*(.05/(1132.362620-1078.223545)),
                                   .05-(points[:,0]-103.272655)*(.05/(157.398941-103.272655))])
        arrays['xy_'+color]=np.concatenate(converted)
        if color=='desired':
            assert len(converted)==len(force_segments['desired'])==24
            data=arrays['xy_desired'];center=(data.max(axis=0)+data.min(axis=0))/2
            amplitude=(data.max(axis=0)-data.min(axis=0))/2
            ends=np.array([x[-1] for x in converted]);normalized=(ends-center)/amplitude
            theta=np.unwrap(np.arctan2(normalized[:,1],normalized[:,0]/(2*normalized[:,1])))
            # Original export splits all four curves into matching 128-point
            # chunks. Validate the inferred phase at their shared boundaries.
            xmin,xmax,_,_=BOUNDS['force']
            times=np.array([(x[-1,0]-xmin)/(xmax-xmin)*30 for x in force_segments['desired']])
            phase_error=theta-np.pi*times/12
            arrays['phase_boundaries']=np.c_[times,theta,phase_error]
            assert abs(phase_error).max()<.003
            statistics['trajectory']=dict(center_m=center.tolist(),half_amplitude_m=amplitude.tolist(),
                phase_rate_rad_s=float(np.pi/12),cycles_30s=1.25,
                max_chunk_boundary_phase_error_rad=float(abs(phase_error).max()),
                formula='x=x0+0.05*sin(pi*t/6); y=y0+0.15*sin(pi*t/12)',
                caveat='Timing inferred from shared export chunk boundaries; verify against PDF figure, not raw robot logs.')
    args.output.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(args.output/'curves.npz',**arrays)
    metadata=dict(pdf_sha256=hashlib.sha256(args.pdf.read_bytes()).hexdigest(),pdf_page=15,figure=9,
        source_doi='10.1109/TRO.2026.3741518',method='Native vector polylines via pdftocairo SVG; calibrated against axis grids.',
        native_plot_axis_bounds=BOUNDS,statistics=statistics,
        limitations=['Plotted curves have approximately 10 ms spacing before exporter simplification.',
                     'Do not equate these curves with unpublished 1/4 kHz raw measurements.',
                     'RMSE and beta summaries describe the exported polylines; aliasing and pre-export decimation are unknown.'])
    (args.output/'metrics.json').write_text(json.dumps(metadata,indent=2))
    print(json.dumps(statistics,indent=2))


if __name__=='__main__':main()
