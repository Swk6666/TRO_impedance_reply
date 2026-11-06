"""Five comparison laws, with equation numbers from the supplied papers.

No controller imports or reads the true plant state. All dynamics evaluations
use independent nominal models and measured joint coordinates/velocities.
"""
from dataclasses import dataclass, field
import numpy as np

from .dynamics import NominalDynamics, angle_error
from .model import DT, HOME


@dataclass
class ControlOutput:
    torque: np.ndarray
    reference_error: np.ndarray
    beta: float = 0.
    tank: float = 0.
    diagnostics: dict = field(default_factory=dict)


class Controller:
    def __init__(self, dynamics, scenario, alpha=1.,control_period=DT):
        self.dt=control_period
        self.dynamics=dynamics; self.scenario=scenario; self.alpha=alpha
        self.posture=scenario.initial_q.copy()
        self.integral=np.zeros(6); self.started=False
        self.L=scenario.inertia; self.D=scenario.damping; self.K=scenario.stiffness

    def references(self,state,reference,physical_wrench):
        F=state.generalized_wrench(physical_wrench)
        Fdes=state.generalized_wrench(reference.force)
        e=angle_error(reference.p,state.p)
        edot=reference.v-state.pdot
        if not self.started:
            # Integral sliding surface initialization; s(0)=0.
            self.integral=-self.L*edot-self.D*e
            self.started=True
        vref=reference.v+(self.D*e+self.integral)/self.L
        aref=reference.a+(self.D*edot+self.K*e+F-Fdes)/self.L
        s=vref-state.pdot
        self.integral+=self.dt*(self.K*e+F-Fdes)
        return F,vref,aref,s


class HInfinity(Controller):
    def step(self,t,state,wrench):
        F,vref,aref,s=self.references(state,self.scenario.reference(t),wrench)
        # (50)-(53), including Gamma_p(q,qdot) p_dot_ref.
        gain=self.alpha*(np.array([1000.,1000.,1000.,50.,50.,50.])+1/.1**2)
        task_force=(state.inertia@aref+self.dynamics.task_coriolis_product(vref)
                    +state.Jplus.T@state.gravity-F+gain*s)
        tau=state.J.T@task_force+self.dynamics.null_torque(self.posture)
        return ControlOutput(tau,s)


class Impedance(Controller):
    def step(self,t,state,wrench):
        F,_,aref,s=self.references(state,self.scenario.reference(t),wrench)
        # (54)-(55), extended to 7 axes using the inertia-weighted inverse.
        acceleration=state.Jplus@(aref-state.Jdot@state.v)
        tau=state.M@acceleration+state.bias-state.J.T@F
        tau+=state.null.T@(12*(self.posture-state.q)-4*state.v)
        return ControlOutput(tau,s)


class SlidingMode(Impedance):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.integral_sign=np.zeros(7)

    def step(self,t,state,wrench):
        F,_,aref,s=self.references(state,self.scenario.reference(t),wrench)
        sigma_q=state.J.T@(self.L*s)
        norm=np.linalg.norm(sigma_q)
        sign=sigma_q/(norm+.001)
        # (59)-(62). The seventh-axis value repeats the last wrist-axis gain.
        K1=np.array([30,30,25,25,25,25,25] if self.scenario.name=='E'
                    else [50,50,40,40,40,40,40])
        correction=K1*np.sqrt(norm)*sign+.1*self.integral_sign
        self.integral_sign+=self.dt*sign
        acceleration=state.Jplus@(aref-state.Jdot@state.v)+correction
        tau=state.M@acceleration+state.bias-state.J.T@F
        tau+=state.null.T@(12*(self.posture-state.q)-4*state.v)
        return ControlOutput(tau,s)


class Admittance(Controller):
    def __init__(self,*args,gain_scale=1.,virtual_substeps=1,**kwargs):
        if not isinstance(virtual_substeps,int) or virtual_substeps<1:
            raise ValueError("virtual_substeps must be a positive integer")
        self.virtual_substeps=virtual_substeps
        super().__init__(*args,**kwargs)
        self.proxy=NominalDynamics(self.dynamics.model,self.dynamics.origin)
        self.qn=self.posture.copy(); self.vn=np.zeros(7); self.integral_joint=np.zeros(7)
        self.G=gain_scale*np.array([1200,1200,600,400,400,400,400.])
        self.Kp=np.array([60,60,40,30,30,30,30.]); self.Ki=np.full(7,5.)

    def nominal_acceleration(self,t,q,v,wrench):
        n=self.proxy.evaluate(q,v)
        ref=self.scenario.reference(t)
        e=angle_error(ref.p,n.p)
        F=n.generalized_wrench(wrench-ref.force)
        aref=ref.a+(self.D*(ref.v-n.pdot)+self.K*e+F)/self.L
        acc=n.Jplus@(aref-n.Jdot@v)+n.null@(12*(self.posture-q)-4*v)
        torque=n.M@acc+n.bias-n.J.T@n.generalized_wrench(wrench)
        return acc,torque,n

    def advance_proxy(self,t,wrench):
        y=np.r_[self.qn,self.vn]
        def rhs(time,value):
            acc,_,_=self.nominal_acceleration(time,value[:7],value[7:],wrench)
            return np.r_[value[7:],acc]
        h=self.dt/self.virtual_substeps
        for k in range(self.virtual_substeps):
            time=t+k*h
            k1=rhs(time,y); k2=rhs(time+h/2,y+h/2*k1)
            k3=rhs(time+h/2,y+h/2*k2); k4=rhs(time+h,y+h*k3)
            y+=h/6*(k1+2*k2+2*k3+k4)
        self.qn=y[:7];self.vn=y[7:]

    def diagnostics(self,state,n,wrench,correction):
        ref=self.scenario.reference(self.diagnostic_time)
        F=state.generalized_wrench(wrench)
        Fn=n.generalized_wrench(wrench)
        Fd=state.generalized_wrench(ref.force)
        Fdn=n.generalized_wrench(ref.force)
        ev=self.vn-state.v
        return dict(proxy_pose=n.p.copy(),proxy_velocity=n.pdot.copy(),
                    proxy_q=self.qn.copy(),proxy_v=self.vn.copy(),
                    power_environment=float(ev@(state.J.T@F)),
                    power_environment_exact=float(n.pdot@Fn-state.pdot@F),
                    power_force_command=float(-(n.pdot@Fdn-state.pdot@Fd)),
                    power_compensation=float(ev@correction),
                    compensation_torque=correction.copy())

    def step(self,t,state,wrench):
        self.diagnostic_time=t
        _,torque,n=self.nominal_acceleration(t,self.qn,self.vn,wrench)
        error=self.qn-state.q; velocity_error=self.vn-state.v
        correction=self.G*(velocity_error+self.Kp*error+self.Ki*self.integral_joint)
        self.integral_joint+=self.dt*error
        _,_,_,s=self.references(state,self.scenario.reference(t),wrench)
        diagnostics=self.diagnostics(state,n,wrench,correction)
        self.advance_proxy(t,wrench)
        return ControlOutput(torque+correction,s,diagnostics=diagnostics)


class EnergyTank:
    """Lee et al. 2024 (19),(20)-(22),(28),(31)-(32).

    Main paper calls the modulation beta; the 2024 source calls it alpha.
    Integrate T directly: T_dot=psi, equivalent to T=z^2/2, z_dot=psi/z.
    The discrete jump cost (beta-new - beta-old)*(Sr-Sn) is accounted for
    explicitly. Numerical depletion is logged, never silently hidden.
    """
    def __init__(self,initial=1.,minimum=.05,maximum=1.,control_period=DT):
        self.dt=control_period
        self.energy=initial;self.minimum=minimum;self.maximum=maximum
        self.beta=0.;self.depletion_count=0

    def update(self,generation,compensation,Sr,Sn,diss_real,diss_nominal,beta_override=None):
        if self.energy<=self.minimum:
            beta=1.
        elif generation>0 and compensation>0:
            beta=min(1.,generation/compensation)
        else:
            beta=0.
        self.candidate_beta=beta
        if beta_override is not None:beta=beta_override
        mu=generation if compensation<=0 and self.energy>self.minimum else 0.
        delta=self.dt*((1-beta)*diss_nominal+beta*diss_real-(1-beta)*mu)
        self.dissipation=(1-beta)*diss_nominal+beta*diss_real
        self.jump=(beta-self.beta)*(Sr-Sn)
        self.mu=mu
        delta-=self.jump
        if self.energy<self.maximum or delta<0:
            self.energy+=delta
        self.energy=min(self.energy,self.maximum)
        if self.energy<0:
            self.depletion_count+=1
        self.beta=beta
        return beta


class Bidirectional(Admittance):
    def __init__(self,*args,energy_port="net",beta_override=None,coupling_update="held",**kwargs):
        if coupling_update not in ("held","stages"):
            raise ValueError("Unknown nominal coupling update")
        self.coupling_update=coupling_update
        self.held_real_q=HOME.copy();self.held_real_v=np.zeros(7);self.held_integral=np.zeros(7)
        if energy_port not in ("net","physical","physical_exact"):
            raise ValueError("Unknown energy port")
        if beta_override is not None and not 0<=beta_override<=1:
            raise ValueError("beta override must be between zero and one")
        self.energy_port=energy_port;self.beta_override=beta_override
        super().__init__(*args,**kwargs)
        self.tank=EnergyTank(control_period=self.dt)
        self.frozen_beta=0.;self.frozen_correction=np.zeros(7)

    def nominal_acceleration(self,t,q,v,wrench):
        n=self.proxy.evaluate(q,v)
        ref=self.scenario.reference(t)
        error=angle_error(ref.p,n.p)
        # Main paper (58), lambda=1, gn=0, PD nominal controller.
        # Hybrid C adds the desired force to the external-force input port.
        F=n.generalized_wrench(wrench-ref.force)
        tau=n.J.T@(self.K*error-self.D*n.pdot)
        tau+=n.null.T@(12*(self.posture-q)-4*v)
        c=n.bias-n.gravity
        correction=self.frozen_correction
        if self.coupling_update=='stages':
            # The virtual ODE contains its own current qn/vn. Actual joint
            # feedback, F/T, beta and the motor command remain held for 1 ms.
            correction=self.G*(v-self.held_real_v+self.Kp*(q-self.held_real_q)+self.Ki*self.held_integral)
        acc=np.linalg.solve(n.M,tau+n.J.T@F-self.frozen_beta*correction-c)
        return acc,tau,n

    def step(self,t,state,wrench):
        self.diagnostic_time=t
        _,_,n=self.nominal_acceleration(t,self.qn,self.vn,wrench)
        ref=self.scenario.reference(t)
        er=angle_error(ref.p,state.p);en=angle_error(ref.p,n.p)
        eq=self.qn-state.q;ev=self.vn-state.v
        correction=self.G*(ev+self.Kp*eq+self.Ki*self.integral_joint)
        F=state.generalized_wrench(wrench-ref.force)
        tau_ext=state.J.T@F
        Sr=.5*(state.v@state.M@state.v+er@(self.K*er))
        Sn=.5*(self.vn@n.M@self.vn+en@(self.K*en))
        diagnostics=self.diagnostics(state,n,wrench,correction)
        generation=float(ev@tau_ext) if self.energy_port=='net' else diagnostics[
            'power_environment_exact' if self.energy_port=='physical_exact' else 'power_environment']
        beta=self.tank.update(generation,ev@correction,Sr,Sn,
                              state.pdot@(self.D*state.pdot),n.pdot@(self.D*n.pdot),self.beta_override)
        diagnostics.update(power_generation=generation,storage_real=Sr,storage_nominal=Sn,
                           beta_candidate=self.tank.candidate_beta,tank_dissipation=self.tank.dissipation,
                           tank_jump=self.tank.jump,tank_mu=self.tank.mu)
        # lambda=1 auxiliary control, with the force-command port for C.
        auxiliary=state.gravity+state.J.T@(self.K*er-self.D*state.pdot-state.generalized_wrench(ref.force))
        auxiliary+=state.null.T@(12*(self.posture-state.q)-4*state.v)
        self.integral_joint+=self.dt*eq
        self.held_real_q=state.q.copy();self.held_real_v=state.v.copy()
        self.held_integral=self.integral_joint-self.dt*eq
        self.frozen_beta=beta;self.frozen_correction=correction
        _,_,_,s=self.references(state,ref,wrench)
        self.advance_proxy(t,wrench)
        return ControlOutput(auxiliary+(1-beta)*correction,s,beta,self.tank.energy,diagnostics)


def make_controller(name,dynamics,scenario,control_period=DT,*,alpha=1.,nric_gain_scale=1.,energy_port="net",beta_override=None,
                    virtual_substeps=1,coupling_update="held",ref_gain_scale=1.):
    if name.startswith('hinf'):
        alpha=float(name.split('_')[1]) if '_' in name else alpha
        return HInfinity(dynamics,scenario,alpha*ref_gain_scale,control_period)
    cls={'impedance':Impedance,'smc':SlidingMode,'admittance':Admittance,
         'bidirectional':Bidirectional}[name]
    kwargs={'gain_scale':nric_gain_scale,'virtual_substeps':virtual_substeps} if issubclass(cls,Admittance) else {}
    if cls is Bidirectional:kwargs.update(energy_port=energy_port,beta_override=beta_override,coupling_update=coupling_update)
    return cls(dynamics,scenario,control_period=control_period,**kwargs)
