"""Physical impedance metrics, independent of controller internal integrators.

Tracking error during contact includes the intended compliant displacement.
Compare it with the full desired differential equation as well as the paper's
quasi-static F/K plots; neither metric alone certifies stability/passivity.
"""
import numpy as np
from scipy.linalg import expm

from .dynamics import euler_rate_map


def ideal_impedance_error(time, force, inertia, damping, stiffness, initial_error,
                          initial_error_rate):
    """Exact ZOH integration of L e'' + D e' + K e = Fdes - Fext.

    Uses saved *physical* wrench, not controller REF or virtual robot state.
    Angular wrench must already be expressed in power-dual Euler coordinates.
    Zero stiffness (force control/free roll) is supported without inversion.
    """
    time=np.asarray(time,dtype=float)
    result=np.empty((len(time),6),dtype=float)
    if not len(time):return result
    dt=float(np.median(np.diff(time))) if len(time)>1 else .001
    if len(time)>1 and not np.allclose(np.diff(time),dt,atol=1e-10,rtol=0):
        raise ValueError('Impedance evaluation requires a uniform sample clock')
    matrices=[]
    for L,D,K in zip(inertia,damping,stiffness):
        matrices.append(expm(dt*np.array([[0.,1.,0.],[-K/L,-D/L,1/L],[0.,0.,0.]])))
    matrices=np.array(matrices)
    state=np.column_stack((initial_error,initial_error_rate))
    for i in range(len(time)):
        result[i]=state[:,0]
        state=np.einsum('ijk,ik->ij',matrices[:,:2,:2],state)+matrices[:,:2,2]*force[i,:,None]
    return result


def impedance_metrics(log,scenario):
    time=log['time'];pose=log['pose'].astype(float)
    error=log['desired'].astype(float)-pose
    error[:,3:]=(error[:,3:]+np.pi)%(2*np.pi)-np.pi
    force=log['force_true'].astype(float).copy()
    command=log['force_desired'].astype(float).copy()
    for i,p in enumerate(pose):
        force[i,3:]=euler_rate_map(p[3:]).T@force[i,3:]
        command[i,3:]=euler_rate_map(p[3:]).T@command[i,3:]
    dynamic=ideal_impedance_error(time,command-force,scenario.inertia,scenario.damping,
        scenario.stiffness,error[0],scenario.reference(time[0]).v-log['velocity'][0])
    gap=error-dynamic
    equilibrium=error.copy();nonzero=scenario.stiffness>0
    equilibrium[:,nonzero]+=(force-command)[:,nonzero]/scenario.stiffness[nonzero]
    ref_norm=np.linalg.norm(log['reference_error'][:,:3],axis=1)
    eq_norm=np.linalg.norm(equilibrium[:,:3],axis=1)
    result=dict(
        dynamic_impedance_position_rmse_m=float(np.sqrt(np.mean(np.sum(gap[:,:3]**2,axis=1)))),
        dynamic_impedance_rotation_rmse_rad=float(np.sqrt(np.mean(np.sum(gap[:,3:]**2,axis=1)))),
        reference_error_position_mean_m_s=float(ref_norm.mean()),
        reference_error_position_std_m_s=float(ref_norm.std()),
        equilibrium_position_error_mean_m=float(eq_norm.mean()),
        equilibrium_position_error_std_m=float(eq_norm.std()))
    if scenario.name in ('B','F'):
        contact=np.linalg.norm(log['force_contact'][:,:3],axis=1)>1
        result['first_contact_s']=float(time[contact][0]) if contact.any() else None
        result['last_contact_s']=float(time[contact][-1]) if contact.any() else None
    if scenario.name=='C':
        engaged=time>=1
        result['board_contact_fraction_after_1s']=float(np.mean(log['board_contact'][engaged])) if engaged.any() else None
    if scenario.name=='E':
        engaged=time>=6
        if engaged.any():
            active=log['environment_contact'] if 'environment_contact' in log else np.linalg.norm(log['force_contact'][:,:3],axis=1)>.01
            result['contact_loss_fraction_after_6s']=float(np.mean(active[engaged]==0))
            result['tangential_tracking_rmse_m']=float(np.sqrt(np.mean(error[engaged,0]**2)))
    return result,dynamic,equilibrium
