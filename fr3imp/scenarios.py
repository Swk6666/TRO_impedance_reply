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


# Fig. 11 moments are replayed mirrored about the x-z plane, so the roll stays
# +90 degrees like the grasp model (the paper rolled the other way).
REPLAY_MIRROR = np.array([-1.,1.,-1.])
REPLAY_COLUMNS = ('hinf','admittance','impedance','smc')


def smoothstep(t, duration):
    u = np.clip(t/duration,0,1)
    s = 10*u**3-15*u**4+6*u**5
    ds = (30*u**2-60*u**3+30*u**4)/duration
    dds = (60*u-180*u**2+120*u**3)/duration**2
    return s, ds, dds


class Scenario:
    def __init__(self, name, initial_pose, trial=0,variant="compact",initial_q=None,grinder_model='external',
                 impedance_overrides=None,operator_stiffness_scale=1.,operator_torque_scale=1.,operator_roll_hold=False,
                 operator_model='grasp',operator_replay=None):
        if variant not in ("compact","paper_figure","paper_diagonal"):
            raise ValueError("Unknown scenario variant")
        self.variant=variant
        self.name = name
        if grinder_model not in ('external','rotor'):
            raise ValueError('grinder_model must be external or rotor')
        self.grinder_model=grinder_model
        self.p0 = np.asarray(initial_pose).copy()
        from .model import HOME
        self.initial_q=np.array(HOME if initial_q is None else initial_q,copy=True)
        self.inertia, self.damping, self.stiffness = [np.array(v,dtype=float) for v in PARAMETERS[name]]
        for key,value in (impedance_overrides or {}).items():
            if key not in ('inertia','damping','stiffness'):
                raise ValueError(f'Unknown impedance parameter: {key}')
            value=np.asarray(value,dtype=float)
            if value.shape!=(6,) or not np.isfinite(value).all() or np.any(value<0) or (key=='inertia' and np.any(value==0)):
                raise ValueError(f'{key} must contain six finite, nonnegative values (inertia strictly positive)')
            setattr(self,key,value.copy())
        if not np.isfinite(operator_stiffness_scale) or operator_stiffness_scale<=0:
            raise ValueError('operator_stiffness_scale must be positive')
        self.operator_stiffness_scale=operator_stiffness_scale
        torque_scale=np.asarray(operator_torque_scale,dtype=float)
        if torque_scale.shape not in ((),(3,)) or not np.isfinite(torque_scale).all() or np.any(torque_scale<=0):
            raise ValueError('operator_torque_scale must be positive: scalar or three Euler moment scales')
        self.operator_torque_scale=np.broadcast_to(torque_scale,(3,)).copy()
        self.operator_roll_hold=bool(operator_roll_hold)
        self.duration = 35. if name=='D' else 30.
        self.trial = trial
        # Ten reproducible virtual operator profiles, each with three repeats.
        profile = np.random.default_rng(1000+trial//3)
        repeat = np.random.default_rng(2000+trial)
        self.operator_scale = profile.uniform(.8,1.2) * repeat.uniform(.95,1.05)
        if operator_model not in ('grasp','fig11_replay'):
            raise ValueError('operator_model must be grasp or fig11_replay')
        self.operator_model=operator_model
        if operator_model=='fig11_replay':
            self.load_replay(**(operator_replay or {}))
            self.duration=30.  # The digitized H-infinity trial ends near 30.6 s.

    def load_replay(self,column='hinf',scale=.6,grip=0.):
        """Moments the paper's operator applied in Fig. 11, as digitized.

        `scale` keeps them inside FR3's 12 Nm wrist limit (the paper peaks at
        18 Nm). `grip` adds a firm hand: a spring toward that trial's plotted
        motion, active only while the operator is pushing.
        """
        from scipy.signal import butter,sosfiltfilt
        from .model import ROOT
        if column not in REPLAY_COLUMNS or not 0<scale or grip<0:
            raise ValueError('Invalid Fig. 11 replay settings')
        with np.load(ROOT/'fr3imp/data/paper_d_curves.npz') as curves:
            moments=[curves[f'{column}_{axis}_moment'] for axis in ('roll','pitch','yaw')]
            errors=[curves[f'{column}_{axis}_error'] for axis in ('roll','pitch','yaw')]
        self.replay_grid=np.arange(0,35,.001)
        self.replay_moment=np.array([np.interp(self.replay_grid,m[:,0],m[:,1],right=0.) for m in moments]).T*REPLAY_MIRROR
        # Hand motion is the plotted response without its fast ringing.
        hand=np.array([np.interp(self.replay_grid,e[:,0],-e[:,1],right=0.) for e in errors])
        self.replay_hand=sosfiltfilt(butter(2,3.,fs=1000,output='sos'),hand,axis=1).T*REPLAY_MIRROR
        self.replay_scale=scale*self.operator_scale
        self.replay_grip=grip
        self.operator_replay=dict(column=column,scale=scale,grip=grip)

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
            if self.variant in ('paper_figure','paper_diagonal'):
                # Digitized Fig. 9 vector paths: 50/150 mm half amplitudes,
                # theta=(pi/12)*t. Starts at the center; 1.25 cycles in 30 s.
                # Absolute Indy7 position is translated to the FR3 home pose.
                theta=np.pi*t/12.;w=np.pi/12.
                ax,ay=(.05,.15) if self.name=='C' else (.13,.13)
                # Fig. 6: both half-amplitudes are 130 mm. A uses the
                # same free-motion reference; F explicitly repeats B.
                p[:2]=self.p0[:2]+[ax*np.sin(2*theta), ay*np.sin(theta)]
                v[:2]=[2*ax*np.cos(2*theta)*w, ay*np.cos(theta)*w]
                a[:2]=[-4*ax*np.sin(2*theta)*w*w, -ay*np.sin(theta)*w*w]
        elif self.name=='E':
            s,ds,dds=smoothstep(t,5.)
            omega=2*np.pi/8
            p[0]+=.025*s*np.sin(omega*t)
            v[0]=.025*(ds*np.sin(omega*t)+s*omega*np.cos(omega*t))
            a[0]=.025*(dds*np.sin(omega*t)+2*ds*omega*np.cos(omega*t)-s*omega**2*np.sin(omega*t))
            # Starts 12 mm clear of the workpiece, then penetrates the target
            # by 8 mm. Descent is smooth and reaches the contact near 5 s.
            travel,descent=(.078,6.) if self.variant=='paper_figure' else (.020,9.)
            z,dz,ddz=smoothstep(t,descent)
            p[2]-=travel*z; v[2]=-travel*dz; a[2]=-travel*ddz
        return Reference(p,v,a,force)

    def environment(self):
        x,y,z=self.p0[:3]
        if self.name in ('B','F'):
            if self.variant=='paper_figure':
                # Fig. 6 H-infinity contact trace: dx-dy approximately .125 m.
                # The 45-degree wall creates both x and y compliance; its
                # position includes the 10 mm tool radius and half thickness.
                normal=np.array([1.,-1.])/np.sqrt(2)
                center=np.array([x,y])+normal*(.125/np.sqrt(2)+.01+.01)
                return [dict(name='wall',pos=f'{center[0]} {center[1]} {z}',
                             size='.01 .35 .20',quat=f'{np.cos(np.pi/8)} 0 0 {-np.sin(np.pi/8)}')]
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
            depth=.090 if self.variant=='paper_figure' else .032
            return [dict(name='workpiece',pos=f'{x} {y} {z-depth}',size='.18 .15 .01')]
        return []

    def applied_wrench(self,t,pose,velocity):
        wrench=np.zeros(6)
        if self.name=='D':
            if self.operator_model=='fig11_replay':
                return self.replay_wrench(t,pose,velocity)
            if self.variant=='paper_figure':
                return self.operator_wrench(t,pose,velocity)
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
        elif self.name=='E' and self.grinder_model=='external':
            # Equivalent motor imbalance, not a material-removal simulation.
            wrench[:3]=np.array([1.5*np.sin(2*np.pi*97*t),
                                .8*np.sin(2*np.pi*137*t),2*np.sin(2*np.pi*173*t)])
        if self.name=='D':wrench[3:]*=self.operator_torque_scale
        return wrench

    def replay_wrench(self,t,pose,velocity):
        """Open-loop Fig. 11 moments; identical for every controller given a trial.

        Roll has zero stiffness, so an open-loop moment would not reach 90
        degrees; the operator guides it closed-loop over the plotted 13-17.5 s.
        """
        from .dynamics import euler_rate_map
        index=min(int(round(t/.001)),len(self.replay_grid)-1)
        moment=self.replay_scale*self.replay_moment[index]
        if self.replay_grip:
            stiffness=self.replay_grip*np.array([0.,3.,1.])
            pushing=np.abs(moment)>[.5,1.,.5]
            moment=moment+pushing*(stiffness*(self.replay_scale*self.replay_hand[index]-pose[3:])-self.replay_grip/40*velocity[3:])
        if 13.<=t<17.5:
            r,dr,_=smoothstep(t-13.,4.)
            moment[0]=self.operator_scale*(300*(np.pi/2*r-pose[3])+8*(np.pi/2*dr-velocity[3]))
        moment=np.clip(moment,[-18,-25,-25],[18,25,25])
        return np.r_[np.zeros(3),np.linalg.solve(euler_rate_map(pose[3:]).T,moment)]

    def operator_wrench(self,t,pose,velocity):
        """Declared virtual grasp, not a replay of unpublished human forces.

        A finite-stiffness handle makes the external moment depend on robot
        motion. Release really removes the grasp, allowing the specified
        under/critical damping to be observed. Identical law for all methods.
        """
        from .dynamics import euler_rate_map
        moment=np.zeros(3)
        for start,axis,target,stiffness in [
                (2.,2,.75,40.),(5.,2,-.75,40.),
                (8.,1,.12,120.),(10.,1,-.12,120.),
                (22.,2,.75,40.),(25.,2,-.75,40.),
                (28.,1,.12,120.),(30.,1,-.12,120.)]:
            elapsed=t-start
            if 0<=elapsed<1.2:
                ramp,_,_=smoothstep(elapsed,.15)
                moment[axis]=self.operator_scale*(self.operator_stiffness_scale*stiffness*(target*ramp-pose[axis+3])-.8*velocity[axis+3])
        if 13.<=t and (t<20. or self.operator_roll_hold):
            r,dr,_=smoothstep(t-13.,5.)
            moment[0]=self.operator_scale*(300*(np.pi/2*r-pose[3])+8*(np.pi/2*dr-velocity[3]))
        moment=np.clip(moment,[-18,-12,-12],[18,12,12])
        moment*=self.operator_torque_scale
        # Euler moments and physical moments are power-dual. In particular,
        # a 90-degree roll must not silently swap pitch/yaw stiffnesses.
        return np.r_[np.zeros(3),np.linalg.solve(euler_rate_map(pose[3:]).T,moment)]

    @property
    def extra_force_noise(self):
        return 1.5 if self.name=='E' and self.grinder_model=='external' else 0.
