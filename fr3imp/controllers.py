"""Five comparison laws, with equation numbers from the supplied papers.

No controller imports or reads the true plant state. All dynamics evaluations
use independent nominal models and measured joint coordinates/velocities.
"""
from dataclasses import dataclass
import numpy as np

from .dynamics import NominalDynamics, angle_error
from .model import DT, HOME


@dataclass
class ControlOutput:
    torque: np.ndarray
    reference_error: np.ndarray
    beta: float = 0.
    tank: float = 0.


class Controller:
    def __init__(self, dynamics, scenario, alpha=1.):
        self.dynamics=dynamics; self.scenario=scenario; self.alpha=alpha
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
        self.integral+=DT*(self.K*e+F-Fdes)
        return F,vref,aref,s


class HInfinity(Controller):
    def step(self,t,state,wrench):
        F,vref,aref,s=self.references(state,self.scenario.reference(t),wrench)
        # (50)-(53), including Gamma_p(q,qdot) p_dot_ref.
        gain=self.alpha*(np.array([1000.,1000.,1000.,50.,50.,50.])+1/.1**2)
        task_force=(state.inertia@aref+self.dynamics.task_coriolis_product(vref)
                    +state.Jplus.T@state.gravity-F+gain*s)
        tau=state.J.T@task_force+self.dynamics.null_torque(HOME)
        return ControlOutput(tau,s)


class Impedance(Controller):
    def step(self,t,state,wrench):
        F,_,aref,s=self.references(state,self.scenario.reference(t),wrench)
        # (54)-(55), extended to 7 axes using the inertia-weighted inverse.
        acceleration=state.Jplus@(aref-state.Jdot@state.v)
        tau=state.M@acceleration+state.bias-state.J.T@F
        tau+=state.null.T@(12*(HOME-state.q)-4*state.v)
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
        self.integral_sign+=DT*sign
        acceleration=state.Jplus@(aref-state.Jdot@state.v)+correction
        tau=state.M@acceleration+state.bias-state.J.T@F
        tau+=state.null.T@(12*(HOME-state.q)-4*state.v)
        return ControlOutput(tau,s)


class Admittance(Controller):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.proxy=NominalDynamics(self.dynamics.model,self.dynamics.origin)
        self.qn=HOME.copy(); self.vn=np.zeros(7); self.integral_joint=np.zeros(7)
        self.G=np.array([1200,1200,600,400,400,400,400.])
        self.Kp=np.array([60,60,40,30,30,30,30.]); self.Ki=np.full(7,5.)

    def nominal_acceleration(self,t,q,v,wrench):
        n=self.proxy.evaluate(q,v)
        ref=self.scenario.reference(t)
        e=angle_error(ref.p,n.p)
        F=n.generalized_wrench(wrench-ref.force)
        aref=ref.a+(self.D*(ref.v-n.pdot)+self.K*e+F)/self.L
        acc=n.Jplus@(aref-n.Jdot@v)+n.null@(12*(HOME-q)-4*v)
        torque=n.M@acc+n.bias-n.J.T@n.generalized_wrench(wrench)
        return acc,torque,n

    def advance_proxy(self,t,wrench):
        y=np.r_[self.qn,self.vn]
        def rhs(time,value):
            acc,_,_=self.nominal_acceleration(time,value[:7],value[7:],wrench)
            return np.r_[value[7:],acc]
        k1=rhs(t,y); k2=rhs(t+DT/2,y+DT/2*k1)
        k3=rhs(t+DT/2,y+DT/2*k2); k4=rhs(t+DT,y+DT*k3)
        y+=DT/6*(k1+2*k2+2*k3+k4)
        self.qn=y[:7];self.vn=y[7:]

    def step(self,t,state,wrench):
        _,torque,_=self.nominal_acceleration(t,self.qn,self.vn,wrench)
        error=self.qn-state.q; velocity_error=self.vn-state.v
        correction=self.G*(velocity_error+self.Kp*error+self.Ki*self.integral_joint)
        self.integral_joint+=DT*error
        _,_,_,s=self.references(state,self.scenario.reference(t),wrench)
        self.advance_proxy(t,wrench)
        return ControlOutput(torque+correction,s)


class EnergyTank:
    """Lee et al. 2024 (19),(20)-(22),(28),(31)-(32).

    Main paper calls the modulation beta; the 2024 source calls it alpha.
    Integrate T directly: T_dot=psi, equivalent to T=z^2/2, z_dot=psi/z.
    The discrete jump cost (beta-new - beta-old)*(Sr-Sn) is accounted for
    explicitly. Numerical depletion is logged, never silently hidden.
    """
    def __init__(self,initial=1.,minimum=.05,maximum=1.):
        self.energy=initial;self.minimum=minimum;self.maximum=maximum
        self.beta=0.;self.depletion_count=0

    def update(self,generation,compensation,Sr,Sn,diss_real,diss_nominal):
        if self.energy<=self.minimum:
            beta=1.
        elif generation>0 and compensation>0:
            beta=min(1.,generation/compensation)
        else:
            beta=0.
        mu=generation if compensation<=0 and self.energy>self.minimum else 0.
        delta=DT*((1-beta)*diss_nominal+beta*diss_real-(1-beta)*mu)
        delta-=(beta-self.beta)*(Sr-Sn)
        if self.energy<self.maximum or delta<0:
            self.energy+=delta
        self.energy=min(self.energy,self.maximum)
        if self.energy<0:
            self.depletion_count+=1
        self.beta=beta
        return beta


class Bidirectional(Admittance):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.tank=EnergyTank()
        self.frozen_beta=0.;self.frozen_correction=np.zeros(7)

    def nominal_acceleration(self,t,q,v,wrench):
        n=self.proxy.evaluate(q,v)
        ref=self.scenario.reference(t)
        error=angle_error(ref.p,n.p)
        # Main paper (58), lambda=1, gn=0, PD nominal controller.
        # Hybrid C adds the desired force to the external-force input port.
        F=n.generalized_wrench(wrench-ref.force)
        tau=n.J.T@(self.K*error-self.D*n.pdot)
        tau+=n.null.T@(12*(HOME-q)-4*v)
        c=n.bias-n.gravity
        acc=np.linalg.solve(n.M,tau+n.J.T@F-self.frozen_beta*self.frozen_correction-c)
        return acc,tau,n

    def step(self,t,state,wrench):
        _,_,n=self.nominal_acceleration(t,self.qn,self.vn,wrench)
        ref=self.scenario.reference(t)
        er=angle_error(ref.p,state.p);en=angle_error(ref.p,n.p)
        eq=self.qn-state.q;ev=self.vn-state.v
        correction=self.G*(ev+self.Kp*eq+self.Ki*self.integral_joint)
        F=state.generalized_wrench(wrench-ref.force)
        tau_ext=state.J.T@F
        Sr=.5*(state.v@state.M@state.v+er@(self.K*er))
        Sn=.5*(self.vn@n.M@self.vn+en@(self.K*en))
        beta=self.tank.update(ev@tau_ext,ev@correction,Sr,Sn,
                              state.pdot@(self.D*state.pdot),n.pdot@(self.D*n.pdot))
        # lambda=1 auxiliary control, with the force-command port for C.
        auxiliary=state.gravity+state.J.T@(self.K*er-self.D*state.pdot-state.generalized_wrench(ref.force))
        auxiliary+=state.null.T@(12*(HOME-state.q)-4*state.v)
        self.integral_joint+=DT*eq
        self.frozen_beta=beta;self.frozen_correction=correction
        _,_,_,s=self.references(state,ref,wrench)
        self.advance_proxy(t,wrench)
        return ControlOutput(auxiliary+(1-beta)*correction,s,beta,self.tank.energy)


def make_controller(name,dynamics,scenario):
    if name.startswith('hinf'):
        alpha=float(name.split('_')[1]) if '_' in name else 1.
        return HInfinity(dynamics,scenario,alpha)
    return {'impedance':Impedance,'smc':SlidingMode,'admittance':Admittance,
            'bidirectional':Bidirectional}[name](dynamics,scenario)
