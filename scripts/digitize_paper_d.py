"""Recover plotted (not raw experimental) D curves from the supplied PDF vectors.

Fig. 11, PDF page 17 of DOI 10.1109/TRO.2026.3741518: one representative trial
per controller. Dashed lines are -m/K (-m for zero-stiffness roll), solid lines
are the task error. Exported polylines are simplified, so spikes may be clipped.
"""
import argparse
import hashlib
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

NS='{http://www.w3.org/2000/svg}'
SOLID='rgb(34.901428%,34.901428%,34.901428%)'
DASHED={'roll':'rgb(85.098267%,32.548523%,9.803772%)','pitch':'rgb(46.665955%,67.451477%,18.823242%)','yaw':'rgb(0%,44.7052%,74.118042%)'}
COLUMNS={'impedance':(79.17,178.59),'admittance':(201.55,300.97),'smc':(322.87,422.29),'hinf':(445.24,544.65)}
ROWS={'roll':(64.81,143.38),'pitch':(159.81,238.38),'yaw':(254.83,333.40)}
# (top, bottom) of each panel, read from its tick labels.
LIMITS={('impedance','roll'):(6,-2),('admittance','roll'):(3,-4),('smc','roll'):(12,-4),('hinf','roll'):(20,-10),
        ('impedance','pitch'):(.4,-.3),('admittance','pitch'):(.15,-.25),('smc','pitch'):(.3,-.3),('hinf','pitch'):(.1,-.1),
        ('impedance','yaw'):(1.5,-1.5),('admittance','yaw'):(1.5,-1.5),('smc','yaw'):(1.5,-1),('hinf','yaw'):(1.5,-1.5)}
STIFFNESS={'roll':1.,'pitch':200.,'yaw':10.}  # roll plots -m directly because K=0.
DT=.001


def matrix(text):
    result=np.eye(3)
    for kind,args in re.findall(r'(matrix|translate|scale)\(([^)]*)\)',text or ''):
        v=[float(x) for x in re.split(r'[ ,]+',args.strip())]
        if kind=='matrix':m=np.array([[v[0],v[2],v[4]],[v[1],v[3],v[5]],[0,0,1]])
        elif kind=='translate':m=np.array([[1,0,v[0]],[0,1,v[1] if len(v)>1 else 0],[0,0,1]])
        else:m=np.diag([v[0],v[-1],1.])
        result=result@m
    return result


def svg_property(element,name):
    direct=element.get(name)
    if direct is None:
        match=re.search(r'(?:^|;)\s*'+re.escape(name)+r'\s*:\s*([^;]+)',element.get('style',''))
        direct=match.group(1) if match else ''
    return re.sub(r'\s+','',direct)


def polylines(root):
    """Every M/L path in page coordinates, with inherited stroke and dash."""
    result=[]
    targets={e.get('id'):e for e in root.iter() if e.get('id')}
    def visit(element,transform,inherited):
        if element.tag.replace(NS,'') in ('defs','clipPath','mask','symbol'):return
        transform=transform@matrix(element.get('transform'))
        inherited=dict(inherited)
        for key in ('stroke','stroke-dasharray'):
            value=svg_property(element,key)
            if value:inherited[key]=value
        if element.tag==NS+'use':
            # Poppler stores repeated Form XObjects once; follow each reference.
            target=targets.get((element.get('{http://www.w3.org/1999/xlink}href') or element.get('href') or '#')[1:])
            if target is not None:
                moved=transform@np.array([[1,0,float(element.get('x',0))],[0,1,float(element.get('y',0))],[0,0,1]])
                for child in (target if target.tag==NS+'symbol' else [target]):visit(child,moved,inherited)
            return
        if element.tag==NS+'path':
            d=element.get('d','')
            numbers=re.findall(r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?',d)
            if numbers and len(numbers)%2==0 and not set(re.findall('[A-Za-z]',d))-{'M','L','Z'}:
                points=np.array(numbers,float).reshape(-1,2)
                page=(transform@np.c_[points,np.ones(len(points))].T).T[:,:2]
                dashed=inherited.get('stroke-dasharray','none')!='none'
                result.append((inherited.get('stroke',''),dashed,page))
        for child in element:visit(child,transform,inherited)
    visit(root,np.eye(3),{})
    return result


def curve(lines,stroke,dashed,column,row,limits):
    (left,right),(top,bottom)=COLUMNS[column],ROWS[row]
    parts=[p for s,d,p in lines if s==stroke and d==dashed and len(p)>=8
           and p[:,0].min()>=left-.5 and p[:,0].max()<=right+.5 and p[:,1].min()>=top-.5 and p[:,1].max()<=bottom+.5]
    if not parts:raise ValueError(f'No curve: {column} {row} {stroke}')
    parts.sort(key=lambda p:p[:,0].min())
    points=np.concatenate(parts)
    time=(points[:,0]-left)/(right-left)*35
    value=limits[0]+(points[:,1]-top)/(bottom-top)*(limits[1]-limits[0])
    order=np.argsort(time,kind='stable')
    return time[order],value[order]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pdf',type=Path,default=Path('Nonlinear_mathcal_H_infty__Inverse-Optimal_Impedance_Control_for_Robotic_Systems.pdf'))
    parser.add_argument('--output',type=Path,default=Path('fr3imp/data/paper_d_curves.npz'))
    args=parser.parse_args()
    with tempfile.TemporaryDirectory() as folder:
        svg=Path(folder)/'page.svg'
        subprocess.run(['pdftocairo','-svg','-f','17','-l','17',str(args.pdf),str(svg)],check=True)
        lines=polylines(ET.parse(svg).getroot())
    arrays={}
    for column in COLUMNS:
        for row in ROWS:
            limits=LIMITS[column,row]
            t,error=curve(lines,SOLID,False,column,row,limits)
            arrays[f'{column}_{row}_error']=np.c_[t,error]
            t,moment=curve(lines,DASHED[row],True,column,row,limits)
            # Stored as the external moment m, undoing the plotted -m/K.
            arrays[f'{column}_{row}_moment']=np.c_[t,-moment*STIFFNESS[row]]
    arrays['pdf_sha256']=np.array(hashlib.sha256(args.pdf.read_bytes()).hexdigest())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(args.output,**arrays)
    print(args.output,len(arrays))


if __name__=='__main__':main()
