"""Recover Fig. 6 reference geometry and wall direction from PDF vectors.

The paper states that A uses the same infinity path and F repeats B. Phase
timing is cross-checked against Fig. 9's shared export chunk boundaries.
Extracted plotted data are not original controller-rate robot recordings.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from digitize_paper_c import NS, paths


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pdf',type=Path,default=Path('Nonlinear_mathcal_H_infty__Inverse-Optimal_Impedance_Control_for_Robotic_Systems.pdf'))
    parser.add_argument('--c-curves',type=Path,default=Path('outputs/paper-c-digitized/curves.npz'))
    parser.add_argument('--output',type=Path,default=Path('outputs/paper-ab-digitized'))
    args=parser.parse_args()
    with tempfile.TemporaryDirectory() as temp:
        svg=Path(temp)/'page14.svg'
        subprocess.run(['pdftocairo','-svg','-f','14','-l','14',str(args.pdf),str(svg)],check=True)
        root=ET.parse(svg).getroot()
    surface=max(root.iter(NS+'g'),key=lambda g:len(list(g)))
    bounds=(491.807593,817.307110,1156.817650,1413.546601)
    segments=paths(surface,'desired',bounds)
    assert len(segments)==24
    def convert(points):
        # Native y increases upwards. Grid spacing is .05 m in both axes.
        return np.c_[.45+(points[:,1]-1167.535473)*.05/(1215.970312-1167.535473),
                     -.05-(points[:,0]-505.684488)*.05/(554.119327-505.684488)]
    converted=[convert(p) for p in segments]
    xy=np.concatenate(converted);center=(xy.min(0)+xy.max(0))/2
    amplitude=(xy.max(0)-xy.min(0))/2
    assert np.allclose(amplitude,.13,atol=3e-5)
    ends=(np.array([s[-1] for s in converted])-center)/amplitude
    phase=np.unwrap(np.arctan2(ends[:,1],ends[:,0]/(2*ends[:,1])))
    times=np.load(args.c_curves)['phase_boundaries'][:,0]
    phase_error=phase-np.pi*times/12
    assert np.max(abs(phase_error))<.003
    actual=[convert(p)-center for p in paths(surface,'hinf',bounds)]
    # Interior contact segments only. Offset/radius are geometric estimates,
    # not manufacturer CAD or a fitted environment stiffness.
    contact=np.concatenate(actual[11:14])
    slope,offset=np.polyfit(contact[:,1],contact[:,0],1)
    args.output.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(args.output/'curves.npz',desired_xy=xy,hinf_xy=np.concatenate(actual)+center,
                        phase_boundaries=np.c_[times,phase,phase_error])
    metadata=dict(pdf_sha256=hashlib.sha256(args.pdf.read_bytes()).hexdigest(),figure=6,pdf_page=14,
        half_amplitude_m=amplitude.tolist(),center_m=center.tolist(),
        max_phase_error_rad=float(np.max(abs(phase_error))),
        trajectory='dx=.13*sin(pi*t/6); dy=.13*sin(pi*t/12)',
        estimated_wall=dict(slope=float(slope),offset_m=float(offset),implemented='dx-dy=.125 m'),
        limitations=['Drawing data, not author raw logs.',
                     'A trajectory reused according to text; F explicitly repeats B.',
                     'World center translated to a nonsingular FR3 posture.',
                     'Contact geometry estimated from plotted adaptation; no author CAD.'])
    (args.output/'metrics.json').write_text(json.dumps(metadata,indent=2))
    print(json.dumps(metadata,indent=2))


if __name__=='__main__':main()
