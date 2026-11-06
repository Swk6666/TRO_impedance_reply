"""Paper A-F parameters and explicitly specified FR3 task adaptations."""
from dataclasses import dataclass
import numpy as np


@dataclass
class Reference:
    p: np.ndarray
    v: np.ndarray
    a: np.ndarray
    force: np.ndarray


PARAMETERS = {
    'A': ([1,1,1,.1,.1,.1], [20,20,20,4,4,4], [100,100,100,40,40,40]),
    'B': ([10,10,10,1,1,1], [80,80,80,10,10,10], [400,400,400,50,50,50]),
    'C': ([4,4,10,.4,.4,.4], [80,80,200,80,80,80], [400,400,0,400,400,400]),
    'D': ([4,4,4,2,.4,.4], [200,200,200,20,2,4], [2000,2000,2000,0,200,10]),
    'E': ([10,10,10,1,1,1], [400,400,400,40,40,40], [1000,1000,1000,400,400,400]),
}
PARAMETERS['F'] = PARAMETERS['B']
COMPARISONS = {
    'A': ('impedance','admittance','hinf'),
    'B': ('impedance','admittance','bidirectional','smc','hinf'),
    'C': ('admittance','bidirectional','hinf'),
    'D': ('impedance','admittance','smc','hinf'),
    'E': ('bidirectional','smc','hinf'),
    'F': ('hinf_0.25','hinf_0.5','hinf_1'),
}


def smoothstep(t, duration):
    u = np.clip(t/duration,0,1)
    s = 10*u**3-15*u**4+6*u**5
    ds = (30*u**2-60*u**3+30*u**4)/duration
    dds = (60*u-180*u**2+120*u**3)/duration**2
    return s, ds, dds


class Scenario:
    def __init__(self, name, initial_pose, trial=0,variant="compact"):
        if variant not in ("compact","paper_figure","paper_diagonal"):
            raise ValueError("Unknown scenario variant")
        self.variant=variant
        self.name = name
        self.p0 = np.asarray(initial_pose).copy()
        self.inertia, self.damping, self.stiffness = [np.array(v,dtype=float) for v in PARAMETERS[name]]
        self.duration = 35. if name=='D' else 30.
        self.trial = trial
        # Ten reproducible virtual operator profiles, each with three repeats.
        profile = np.random.default_rng(1000+trial//3)
        repeat = np.random.default_rng(2000+trial)
        self.operator_scale = profile.uniform(.8,1.2) * repeat.uniform(.95,1.05)

    def reference(self,t):
        p = self.p0.copy(); v=np.zeros(6); a=np.zeros(6); force=np.zeros(6)
        if self.name in 'ABCF':
            s, ds, dds = smoothstep(t,30.)
            theta, w, acc = 2*np.pi*s, 2*np.pi*ds, 2*np.pi*dds
            ax,ay=.035,.045
            p[0] += ax*(1-np.cos(theta)); p[1] += ay*np.sin(2*theta)
            v[0] = ax*np.sin(theta)*w; v[1] = 2*ay*np.cos(2*theta)*w
            a[0] = ax*(np.cos(theta)*w*w+np.sin(theta)*acc)
            a[1] = 2*ay*(-2*np.sin(2*theta)*w*w+np.cos(2*theta)*acc)
            if self.name=='C':
                force[2] = 10+3*np.sin(t)
            if self.name=='C' and self.variant in ('paper_figure','paper_diagonal'):
                # Digitized Fig. 9 vector paths: 50/150 mm half amplitudes,
                # theta=(pi/12)*t. Starts at the center; 1.25 cycles in 30 s.
                # Absolute Indy7 position is translated to the FR3 home pose.
                theta=np.pi*t/12.;w=np.pi/12.
                p[:2]=self.p0[:2]+[.05*np.sin(2*theta), .15*np.sin(theta)]
                v[:2]=[.1*np.cos(2*theta)*w, .15*np.cos(theta)*w]
                a[:2]=[-.2*np.sin(2*theta)*w*w, -.15*np.sin(theta)*w*w]
        elif self.name=='E':
            s,ds,dds=smoothstep(t,5.)
            omega=2*np.pi/8
            p[0]+=.025*s*np.sin(omega*t)
            v[0]=.025*(ds*np.sin(omega*t)+s*omega*np.cos(omega*t))
            a[0]=.025*(dds*np.sin(omega*t)+2*ds*omega*np.cos(omega*t)-s*omega**2*np.sin(omega*t))
            # Starts 12 mm clear of the workpiece, then penetrates the target
            # by 8 mm. Descent is smooth and reaches the contact near 5 s.
            z,dz,ddz=smoothstep(t,9.)
            p[2]-=.020*z; v[2]=-.020*dz; a[2]=-.020*ddz
        return Reference(p,v,a,force)

    def environment(self):
        x,y,z=self.p0[:3]
        if self.name in ('B','F'):
            # The near face meets the tool at x0+0.050. This allows the
            # uncertain, friction-limited ordinary impedance baseline to
            # reach the same fixed obstacle as the robust controllers.
            return [dict(name='wall',pos=f'{x+.070} {y} {z}',size='.01 .20 .20')]
        if self.name=='C' and self.variant in ('paper_figure','paper_diagonal'):
            board=dict(name='whiteboard',pos=f'{x} {y} {z-.02}',size='.25 .35 .01')
            if self.variant=='paper_figure':
                # Fig. 8 shows a chamfered block, not an infinite plane.
                # Fig. 9 suggests TCP limits dx-dy≈.08, then dx≈-.035 m.
                # These are geometric estimates, not author-supplied CAD.
                radius=.01
                face=.08+np.sqrt(2)*radius
                corner=-.035+radius
                polygon=[(corner,-.32),(.20,-.32),(.20,.20-face),(corner,corner-face)]
                vertices=[(px,py,pz) for pz in (-.01,.04) for px,py in polygon]
                return [board,dict(name='threshold',pos=f'{x} {y} {z}',vertices=vertices)]
            # Estimated diagonal adaptation line in Fig. 9: dx-dy=0.08 m.
            # The tool radius and half-thickness shift the actual box center.
            normal=np.array([1.,-1.])/np.sqrt(2)
            center=np.array([x,y])+normal*(.08/np.sqrt(2)+.01+.006)
            return [board,
                    dict(name='threshold',pos=f'{center[0]} {center[1]} {z+.015}',
                         size='.006 .25 .025',quat=f'{np.cos(np.pi/8)} 0 0 {-np.sin(np.pi/8)}')]
        if self.name=='C':
            return [dict(name='whiteboard',pos=f'{x+.04} {y} {z-.02}',size='.25 .25 .01'),
                    dict(name='threshold',pos=f'{x+.078} {y} {z+.015}',size='.006 .15 .025')]
        if self.name=='E':
            return [dict(name='workpiece',pos=f'{x} {y} {z-.032}',size='.18 .15 .01')]
        return []

    def applied_wrench(self,t,pose,velocity):
        wrench=np.zeros(6)
        if self.name=='D':
            # Prescribed yaw/pitch moments; a virtual human spring-damper
            # with a moving reference rotates roll between phases. The
            # moving reference is an explicit external energy source.
            for start,axis,amplitude in [(2.,5,4.),(6.,4,6.),(23.,5,4.),(28.,4,6.)]:
                u=(t-start)/3.
                if 0<u<1:
                    wrench[axis]+=self.operator_scale*amplitude*np.sin(np.pi*u)**2
            if 11.<=t<=20.:
                r,dr,_=smoothstep(t-11.,7.)
                wrench[3]=np.clip(self.operator_scale*(300*(np.pi/2*r-pose[3])+8*(np.pi/2*dr-velocity[3])),-18,18)
        elif self.name=='E':
            # Equivalent motor imbalance, not a material-removal simulation.
            wrench[:3]=np.array([1.5*np.sin(2*np.pi*97*t),
                                .8*np.sin(2*np.pi*137*t),2*np.sin(2*np.pi*173*t)])
        return wrench

    @property
    def extra_force_noise(self):
        return 1.5 if self.name=='E' else 0.
