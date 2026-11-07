"""Compare the W implied by paper Fig. 11 with W = Lambda*Mx^-1 from a published Indy7 model.

Parameters: Tadese et al., Sensors 22(24):9708, 2022 (DOI 10.3390/s22249708), Tables 1, A1, A2.
That study identified a different Indy7, without this paper's F/T sensor and handle.
M(q) is linear in its base parameters, so they are enough to evaluate it. Whether the
controller's nominal model contains the rotor inertia is not stated, so both are computed.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from report_d_admittance import paper_identification

# Modified DH (Table 1): a_{i-1}, alpha_{i-1}, d_i, theta offset.
DH=((0.,0.,.3,0.),(0.,np.pi/2,0.,np.pi/2),(.45,0.,.0035,np.pi/2),
    (0.,np.pi/2,.35,np.pi),(0.,np.pi/2,.1835,0.),(0.,-np.pi/2,.228,0.))
# Table A2, base parameters that enter M(q): conventional cold/hot, proposed cold/hot.
BASE=dict(
    b1=(7.8129,8.2334,7.9853,8.0502),b5=(-1.8496,-1.9353,-1.5784,-1.6053),b6=(.1143,-.2168,-.3067,-.2976),
    b7=(.0648,-.8763,-.9783,-.9511),b8=(.3234,.5146,.5230,.5133),b9=(2.1034,5.2002,5.2119,5.2998),
    b10=(6.0893,6.1889,5.9804,6.0565),b11=(.4989,.1934,.1591,.1608),
    b15=(-.5348,.1292,.7586,.7770),b16=(-.0592,-.0237,.0928,.0932),b17=(.6998,.3381,.5172,.5413),
    b18=(-.0241,.1494,.0920,.1234),b19=(1.5420,1.3622,1.2360,1.1307),b20=(-.1441,-.1250,-.1469,-.1363),
    b21=(-2.2848,-2.3060,-2.3033,-2.3337),b22=(1e-6,1e-6,1e-6,1e-6),
    b26=(.4385,.2657,.3580,.3685),b27=(-.0918,-.0552,-.0376,-.0350),b28=(.0347,.0322,.0875,.0889),
    b29=(-.0483,-.0661,-.0569,-.0502),b30=(.1282,.1372,.2361,.2097),b31=(.0398,.0528,.0793,.0780),
    b32=(-.6260,-.7349,-.7718,-.7869),b33=(.4611,.3825,.3457,.3246),
    b37=(.1059,.0997,.0331,.0342),b38=(-.0113,-.0096,.0019,.0020),b39=(-.0132,-.0099,-.0033,-.0028),
    b40=(.0030,.0030,-.0075,-.0086),b41=(.0995,.0840,.0940,.0930),b42=(.0492,.0356,.0293,.0281),
    b43=(.3489,.3482,.3371,.3488),b44=(.3579,.3804,.4397,.4255),
    b48=(.0371,.0222,.0050,.0058),b49=(-.0029,-.0011,.0070,.0062),b50=(.0234,.0108,-.0208,-.0192),
    b51=(-.0116,-.0067,-.0154,-.0136),b52=(.1567,.1045,.0705,.0649),b53=(-.0147,-.0066,-.0168,-.0194),
    b54=(-.0429,-.0191,-.0371,-.0357),b55=(.4289,.4390,.5405,.5306))
SETS=('conventional_cold','conventional_hot','proposed_cold','proposed_hot')
LAMBDA=dict(roll=2.,pitch=.4,yaw=.4)
AXES_COMPARED=(('phase1','pitch'),('phase1','yaw'),('phase2','pitch'),('phase2','yaw'),('phase2','roll'))


def link_frames(q):
    """Base transforms of link frames 1..6 under modified DH."""
    frames=[];T=np.eye(4)
    for value,(a,alpha,d,offset) in zip(q,DH):
        ca,sa=np.cos(alpha),np.sin(alpha);ct,st=np.cos(value+offset),np.sin(value+offset)
        T=T@np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1.]])
        frames.append(T)
    return frames


def point_jacobian(frames,index,point):
    Jv=np.zeros((3,6));Jw=np.zeros((3,6))
    for joint in range(index+1):
        axis=frames[joint][:3,2];Jw[:,joint]=axis
        Jv[:,joint]=np.cross(axis,point-frames[joint][:3,3])
    return Jv,Jw


def skew(v):
    return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0.]])


def inertia_matrix(q,links,rotor):
    """links[i] = dict(m, h, I) in link frame i about its origin; rotor = joint inertias."""
    frames=link_frames(q);M=np.diag(np.asarray(rotor,dtype=float))
    for i,link in enumerate(links):
        R=frames[i][:3,:3];Jv,Jw=point_jacobian(frames,i,frames[i][:3,3])
        h=skew(R@link['h']);I=R@link['I']@R.T
        coupling=-Jv.T@h@Jw
        M=M+link['m']*Jv.T@Jv+coupling+coupling.T+Jw.T@I@Jw
    return M


def tensor(xx,yy,zz,xy,xz,yz):
    return np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]],dtype=float)


def regrouped_links(b):
    """Table A1 with the merged parameters set to zero, which leaves M(q) unchanged."""
    zero=lambda:dict(m=0.,h=np.zeros(3),I=np.zeros((3,3)))
    links=[zero() for _ in range(6)]
    links[0]['I']=tensor(0,0,b['b1'],0,0,0)
    links[1]=dict(m=0.,h=np.array([b['b10'],b['b11'],0.]),I=tensor(b['b5'],0,b['b9'],b['b6'],b['b7'],b['b8']))
    links[2]=dict(m=0.,h=np.array([b['b20'],b['b21'],0.]),I=tensor(b['b15'],0,b['b19'],b['b16'],b['b17'],b['b18']))
    links[3]=dict(m=0.,h=np.array([b['b31'],b['b32'],0.]),I=tensor(b['b26'],0,b['b30'],b['b27'],b['b28'],b['b29']))
    links[4]=dict(m=0.,h=np.array([b['b42'],b['b43'],0.]),I=tensor(b['b37'],0,b['b41'],b['b38'],b['b39'],b['b40']))
    links[5]=dict(m=0.,h=np.array([b['b53'],b['b54'],0.]),I=tensor(b['b48'],0,b['b52'],b['b49'],b['b50'],b['b51']))
    return links


def base_from_full(links,rotor):
    """Table A1 combinations, used only to check that they agree with Table 1."""
    m=[0]+[x['m'] for x in links]
    h=[None]+[x['h'] for x in links];I=[None]+[x['I'] for x in links]
    xx=lambda i:I[i][0,0];yy=lambda i:I[i][1,1];zz=lambda i:I[i][2,2]
    s3=m[3]+m[4]+m[5]+m[6];s4=m[4]+m[5]+m[6];s5=m[5]+m[6];s6=m[6]
    return dict(
        b1=rotor[0]+yy(2)+yy(3)+zz(1)+.2025*s3+.007*h[3][2],
        b5=xx(2)-yy(2)-.2025*s3,b6=I[2][0,1],b7=I[2][0,2]-.0016*s3-.45*h[3][2],b8=I[2][1,2],
        b9=rotor[1]+zz(2)+.2025*s3,b10=.45*s3+h[2][0],b11=h[2][1],
        b15=xx(3)-yy(3)+yy(4)+.1225*s4+.7*h[4][2],b16=I[3][0,1],b17=I[3][0,2],b18=I[3][1,2],
        b19=yy(4)+zz(3)+.1225*s4+.7*h[4][2],b20=h[3][0],b21=h[3][1]-.35*s4-h[4][2],b22=rotor[2],
        b26=xx(4)-yy(4)+yy(5)+.0337*s5+.367*h[5][2],b27=I[4][0,1],b28=I[4][0,2],b29=I[4][1,2],
        b30=yy(5)+zz(4)+.0337*s5+.367*h[5][2],b31=h[4][0],b32=h[4][1]-.1835*s5-h[5][2],b33=rotor[3],
        b37=xx(5)-yy(5)+yy(6)+.052*s6+.456*h[6][2],b38=I[5][0,1],b39=I[5][0,2],b40=I[5][1,2],
        b41=yy(6)+zz(5)+.052*s6+.456*h[6][2],b42=h[5][0],b43=.228*s6+h[5][1]+h[6][2],b44=rotor[4],
        b48=xx(6)-yy(6),b49=I[6][0,1],b50=I[6][0,2],b51=I[6][1,2],b52=zz(6),b53=h[6][0],b54=h[6][1],b55=rotor[5])


def model(name,rotor):
    index=SETS.index(name)
    b={key:value[index] for key,value in BASE.items()}
    armature=[0.,0.,b['b22'],b['b33'],b['b44'],b['b55']] if rotor else [0.]*6
    return regrouped_links(b),armature


def tcp_pose(q,tool):
    frames=link_frames(q);R=frames[5][:3,:3]
    return frames[5][:3,3]+tool*R[:,2],R,frames


def jacobian(q,tool):
    point,_,frames=tcp_pose(q,tool)
    Jv,Jw=point_jacobian(frames,5,point)
    return np.vstack([Jv,Jw])


def rotation_error(R,target):
    E=target@R.T
    return .5*np.array([E[2,1]-E[1,2],E[0,2]-E[2,0],E[1,0]-E[0,1]])


def inverse_kinematics(point,target,start,tool,iterations=200):
    q=np.array(start,dtype=float)
    for _ in range(iterations):
        p,R,_=tcp_pose(q,tool)
        error=np.r_[point-p,rotation_error(R,target)]
        if np.linalg.norm(error)<1e-9:return q
        J=jacobian(q,tool)
        q=q+np.linalg.solve(J.T@J+1e-6*np.eye(6),J.T@error)
    return None


def rot_x(angle):
    c,s=np.cos(angle),np.sin(angle)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])


TOOL_DOWN=np.array([[-1.,0,0],[0,1,0],[0,0,-1]])
START=np.deg2rad([0,-15,-90,0,-75,0])


def rot_y(angle):
    c,s=np.cos(angle),np.sin(angle)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])


def experiment_pose(point,tool,roll,tilt=0.):
    """Tool down at point, then rolled about base x with the point held, as in Fig. 10."""
    start=rot_y(tilt)@TOOL_DOWN
    q=inverse_kinematics(point,start,START,tool)
    if q is None:return None,None
    first=q.copy()
    for fraction in np.linspace(0,1,19)[1:]:
        q=inverse_kinematics(point,rot_x(roll*fraction)@start,q,tool)
        if q is None:return first,None
    return first,q


def inverse_diagonal(q,tool,links,armature):
    J=jacobian(q,tool);M=inertia_matrix(q,links,armature)
    if np.linalg.eigvalsh(M)[0]<=0:return None
    return np.diag(J@np.linalg.solve(M,J.T))


def c_mass(curves,links,armature):
    """z-axis task mass along the digitized C trajectory, tool down; the height is not published."""
    path=np.load(curves)['xy_desired'][::200]
    values=[]
    for x,y in path:
        for z in (.1,.15,.2):
            for tool in (.05,.12,.2):
                q=inverse_kinematics(np.array([x,y,z]),TOOL_DOWN,START,tool)
                if q is None or link_frames(q)[2][2,3]<link_frames(q)[1][2,3]:continue
                J=jacobian(q,tool);M=inertia_matrix(q,links,armature)
                values.append(1/(J[2]@np.linalg.solve(M,J[2])))
    return dict(median=float(np.median(values)),low=float(np.percentile(values,5)),high=float(np.percentile(values,95)),count=len(values))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--c-curves',type=Path,default=Path('outputs/paper-c-digitized/curves.npz'))
    parser.add_argument('--output',type=Path,default=Path('docs/results/d-admittance/indy7_mx.json'))
    args=parser.parse_args()
    paper=paper_identification(dict(np.load('fr3imp/data/paper_d_curves.npz')))['admittance']['phases']
    observed=dict(phase1={a:paper['phase1']['W'][i]/LAMBDA[a] for i,a in enumerate(LAMBDA)},
                  phase2={a:paper['phase2']['W'][i]/LAMBDA[a] for i,a in enumerate(LAMBDA)})
    # Fig. 10 fixes the shape of the pose but not its numbers, so sweep them. The lateral
    # offset d3+d5 puts q1=0 at y=-.187, the centre of the paper's B trajectory.
    points=[np.array([x,y,z]) for x in (.35,.45,.55,.65) for y in (-.3,-.187,-.1) for z in (.25,.35,.45,.55)]
    tools=(.05,.12,.2);rolls=(-np.pi/2,);tilts=np.deg2rad([-15.,0.,15.])
    poses=[]
    for point in points:
        for tool in tools:
            for roll in rolls:
                for tilt in tilts:
                    first,second=experiment_pose(point,tool,roll,tilt)
                    if first is None or second is None:continue
                    # Elbow up and wrist away from singularity, as in the photographs.
                    frames=link_frames(first)
                    if frames[2][2,3]<frames[1][2,3]:continue
                    if min(np.linalg.svd(jacobian(first,tool),compute_uv=False)[-1],np.linalg.svd(jacobian(second,tool),compute_uv=False)[-1])<.02:continue
                    poses.append(dict(point=point.tolist(),tool=tool,roll=roll,tilt=float(tilt),first=first,second=second))
    nominal=[p for p in poses if p['point']==[.55,-.187,.45] and p['tool']==.12 and p['tilt']==0.]
    result=dict(source='Tadese et al., Sensors 22(24):9708 (2022), Tables 1, A1, A2',observed_inverse_inertia=observed,
                poses_used=len(poses),models={})
    for name in SETS:
        for rotor in (False,True):
            links,armature=model(name,rotor)
            rows=[]
            for pose in poses:
                a=inverse_diagonal(pose['first'],pose['tool'],links,armature)
                b=inverse_diagonal(pose['second'],pose['tool'],links,armature)
                if a is None or b is None:continue
                rows.append(dict(phase1=a,phase2=b))
            label=f'{name}/{"with_rotor" if rotor else "no_rotor"}'
            entry=dict(valid_poses=len(rows),total_poses=len(poses))
            if rows:
                for phase in ('phase1','phase2'):
                    stack=np.array([r[phase] for r in rows])
                    entry[phase]={a:dict(median=float(np.median(stack[:,3+i])),low=float(np.percentile(stack[:,3+i],5)),high=float(np.percentile(stack[:,3+i],95)))
                                  for i,a in enumerate(LAMBDA)}
                    entry[phase]['mass_z']=dict(median=float(np.median(1/stack[:,2])),low=float(np.percentile(1/stack[:,2],5)),high=float(np.percentile(1/stack[:,2],95)))
                # Share of poses where both wrist axes agree in both phases; roll is excluded
                # because its moment is too small to identify.
                close=[all(abs(r[phase][3+list(LAMBDA).index(axis)]/observed[phase][axis]-1)<.2
                           for phase,axis in AXES_COMPARED if axis!='roll') for r in rows]
                entry['poses_within_20_percent']=int(sum(close))
                entry['coefficient_needed']={f'{phase}_{axis}':observed[phase][axis]/entry[phase][axis]['median'] for phase,axis in AXES_COMPARED}
                entry['mass_z_c']=c_mass(args.c_curves,links,armature)
            result['models'][label]=entry
    result['nominal_pose_joints_deg']=[dict(phase1=np.rad2deg(p['first']).round(1).tolist(),phase2=np.rad2deg(p['second']).round(1).tolist()) for p in nominal][:1]
    if nominal:
        result['nominal_pose']={}
        for label in result['models']:
            name,kind=label.split('/')
            links,armature=model(name,kind=='with_rotor')
            a=inverse_diagonal(nominal[0]['first'],nominal[0]['tool'],links,armature)
            b=inverse_diagonal(nominal[0]['second'],nominal[0]['tool'],links,armature)
            if a is not None and b is not None:
                result['nominal_pose'][label]=dict(phase1=dict(roll=a[3],pitch=a[4],yaw=a[5],mass_z=1/a[2]),phase2=dict(roll=b[3],pitch=b[4],yaw=b[5],mass_z=1/b[2]))
    args.output.write_text(json.dumps(result,indent=2))
    print(f'poses {len(poses)}; observed {observed}; nominal joints {result["nominal_pose_joints_deg"]}')
    for label,entry in result.get('nominal_pose',{}).items():
        print('nominal',label,{p:{k:round(v,2) for k,v in entry[p].items()} for p in entry})
    for label,entry in result['models'].items():
        if 'phase1' not in entry:
            print(label,'no positive definite pose');continue
        cell=lambda p,a:f"{entry[p][a]['median']:.2f}[{entry[p][a]['low']:.2f},{entry[p][a]['high']:.2f}]"
        print(f"{label:34s} within20 {entry['poses_within_20_percent']:3d} c {[round(v,2) for v in entry['coefficient_needed'].values()]} Mz_C {entry['mass_z_c']['median']:.1f}[{entry['mass_z_c']['low']:.1f},{entry['mass_z_c']['high']:.1f}] n{entry['mass_z_c']['count']}")
        print(f"{label:34s} valid {entry['valid_poses']:3d}/{entry['total_poses']} p1 pitch {cell('phase1','pitch')} yaw {cell('phase1','yaw')} | p2 pitch {cell('phase2','pitch')} yaw {cell('phase2','yaw')} roll {cell('phase2','roll')} | Mz {cell('phase1','mass_z')}")


if __name__=='__main__':
    main()
