"""Headless experiment runner; 1 kHz FR3 control and F/T; independent physics substeps."""
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .controllers import make_controller
from .actuation import ActuatorParameters, TorqueInterface
from .dynamics import NominalDynamics, angle_error, euler_rate_map
from .evaluation import impedance_metrics
from .grinding import GrindingContact
from .model import ROOT, DT, HOME, TORQUE_LIMIT, PlantParameters, build_model, home_data
from .scenarios import Scenario
from .profiles import profile_parameters
from .sensors import SensorParameters, Sensors


def source_hash():
    digest=hashlib.sha256()
    for name in ('model.py','dynamics.py','sensors.py','scenarios.py','profiles.py','controllers.py','actuation.py','experiment.py','evaluation.py','grinding.py'):
        path=ROOT/'fr3imp'/name
        digest.update(path.name.encode());digest.update(path.read_bytes())
    digest.update((ROOT/'franka_fr3/fr3.xml').read_bytes())
    digest.update((ROOT/'fr3imp/data/paper_d_curves.npz').read_bytes())
    return digest.hexdigest()


def physical_pose(model,data,site,origin):
    R=data.site_xmat[site].reshape(3,3)
    rpy=Rotation.from_matrix(R@origin.T).as_euler('xyz')
    jp=np.zeros((3,model.nv));jr=np.zeros((3,model.nv))
    mujoco.mj_jacSite(model,data,jp,jr,site)
    twist=np.r_[jp@data.qvel,jr@data.qvel]
    velocity=np.r_[twist[:3],np.linalg.solve(euler_rate_map(rpy),twist[3:])]
    return np.r_[data.site_xpos[site],rpy],velocity,twist


def contact_wrench(model,data,tip_geom,tcp):
    wrench=np.zeros(6);value=np.zeros(6)
    for i in range(data.ncon):
        contact=data.contact[i]
        if tip_geom not in (contact.geom1,contact.geom2):
            continue
        mujoco.mj_contactForce(model,data,i,value)
        frame=contact.frame.reshape(3,3)
        sign=1. if contact.geom2==tip_geom else -1.
        force=sign*(frame.T@value[:3])
        moment=sign*(frame.T@value[3:])+np.cross(contact.pos-tcp,force)
        wrench+=np.r_[force,moment]
    return wrench


def metrics_from_log(log,scenario):
    error=log['desired']-log['pose']
    error[:,3:]=(error[:,3:]+np.pi)%(2*np.pi)-np.pi
    force=log['force_true'];s=log['reference_error']
    eq=error.copy()
    selected=scenario.stiffness>0
    # Dynamic trajectories do not satisfy the static equilibrium relation;
    # this metric is descriptive, not a substitute for the dynamic residual.
    generalized=force.copy()
    for i,p in enumerate(log['pose']):
        generalized[i,3:]=euler_rate_map(p[3:]).T@force[i,3:]
    eq[:,selected]+=generalized[:,selected]/scenario.stiffness[selected]
    contact=np.linalg.norm(log['force_contact'][:,:3],axis=1)>1.
    result={
        'position_rmse_m':float(np.sqrt(np.mean(np.sum(error[:,:3]**2,axis=1)))),
        'orientation_rmse_rad':float(np.sqrt(np.mean(np.sum(error[:,3:]**2,axis=1)))),
        'reference_error_position_rms_m_s':float(np.sqrt(np.mean(np.sum(s[:,:3]**2,axis=1)))),
        'reference_error_rotation_rms_rad_s':float(np.sqrt(np.mean(np.sum(s[:,3:]**2,axis=1)))),
        'force_peak_N':float(np.max(np.linalg.norm(force[:,:3],axis=1))),
        'moment_peak_Nm':float(np.max(np.linalg.norm(force[:,3:],axis=1))),
        'torque_peak_Nm':float(np.max(np.abs(log['torque_applied']))),
        'torque_rms_Nm':float(np.sqrt(np.mean(np.sum(log['torque_applied']**2,axis=1)))),
        'saturation_fraction':float(np.mean(log['saturated'])),
        'minimum_jacobian_singular_value':float(np.min(log['jacobian_sigma_min'])),
        'contact_fraction':float(np.mean(contact)),
        'mean_beta':float(np.mean(log['beta'])),
        'min_tank_energy_J':float(np.min(log['tank'])),
        'interaction_work_J':float(log['interaction_work'][-1]),
    }
    if scenario.name=='C':
        result['force_tracking_rmse_N']=float(np.sqrt(np.mean((force[:,2]-log['force_desired'][:,2])**2)))
    if scenario.name=='D':
        for axis,label in [(4,'pitch'),(5,'yaw')]:
            result[f'{label}_equilibrium_error_mae_rad']=float(np.mean(np.abs(eq[:,axis])))
        result['maximum_roll_rad']=float(np.max(log['pose'][:,3]))
        result['phase2_roll_mean_rad']=float(np.mean(log['pose'][log['time']>=23,3])) if np.any(log['time']>=23) else None
    result.update(impedance_metrics(log,scenario)[0])
    return result


def default_plant_parameters(experiment,profile='fr3',c_profile='fr3',grinder_model=None):
    """Shared, declared physical assumptions; never depend on controller choice."""
    parameters=PlantParameters()
    if profile=='fr3v2':
        # Friction sets the task-space disturbance the paper's robot showed;
        # contact is underdamped so it no longer adds phase lead. Neither is
        # identified: both were chosen from the probes in the fr3v2 report.
        parameters=replace(parameters,friction_scale=3.)
        if experiment in ('B','F'):
            return replace(parameters,contact_timeconstant=.02,contact_dampratio=.4,contact_impedance=(.95,.99,.001))
        if experiment=='C':
            return replace(parameters,contact_timeconstant=.04,contact_dampratio=.3,contact_impedance=(.9,.95,.001))
        return parameters
    if experiment=='C' and c_profile=='fr3':
        parameters=replace(parameters,contact_timeconstant=.04,contact_impedance=(.5,.5,.001))
    if profile in ('paper','fr3') and experiment in ('B','E','F'):
        parameters=replace(parameters,contact_timeconstant=.02,contact_impedance=(.5,.5,.001))
    if experiment=='E' and (grinder_model or ('rotor' if profile=='paper' else 'external'))=='rotor':
        # Bosch lists 1.8 kg and 2800–9300 rpm for GWS 7-100 ET. The
        # 0.1 kg rotor, 0.1 mm imbalance and 9000 rpm setting are assumptions.
        parameters=replace(parameters,tool_mass=1.8,rotor_mass=.1)
    return parameters


def default_sensor_parameters(profile,plant_parameters):
    """fr3v2 adds a 3 ms F/T transport delay, an assumption shared by all controllers."""
    return SensorParameters(tool_calibration_mass=plant_parameters.tool_mass,
                            delay_samples=3 if profile=='fr3v2' else 0)


def run_experiment(experiment='A',controller='hinf',*,trial=0,seed=0,
                   duration=None,output='outputs',ideal=False,physics_substeps=4,
                   control_period=DT,plant_parameters=None,sensor_parameters=None,c_profile="fr3",
                   scenario_variant=None,control_overrides=None,actuator_parameters=None,profile="fr3",
                   grinder_model=None,impedance_overrides=None,operator_stiffness_scale=None,operator_torque_scale=None,
                   operator_roll_hold=None,operator_model='grasp',operator_replay=None):
    configuration=profile_parameters(experiment,profile,c_profile)
    scenario_variant=scenario_variant or ('compact' if profile=='legacy' else 'paper_figure')
    impedance={**configuration['impedance'],**(impedance_overrides or {})}
    if operator_stiffness_scale is None:operator_stiffness_scale=configuration['operator_stiffness_scale']
    if operator_torque_scale is None:operator_torque_scale=configuration['operator_torque_scale']
    if operator_roll_hold is None:operator_roll_hold=configuration['operator_roll_hold']
    grinder_model=grinder_model or ('rotor' if profile=='paper' else 'external')
    if c_profile not in ("fr3","legacy"):
        raise ValueError("c_profile must be fr3 or legacy")
    if control_period not in (.001,.00025):
        raise ValueError("Supported control periods: .001 (FR3), .00025 (paper legacy)")
    dt=control_period
    log_stride=round(.001/dt)
    if not isinstance(physics_substeps,int) or physics_substeps<1:
        raise ValueError('physics_substeps must be a positive integer')
    if duration is not None and (not np.isfinite(duration) or duration<dt or not np.isclose(duration/dt,round(duration/dt))):
        raise ValueError('duration must be a positive integer number of control periods')
    folder=Path(output)/f'{experiment}_{controller}_trial{trial:02d}_seed{seed}'
    folder.mkdir(parents=True,exist_ok=True)
    # Explicit FR3 C adaptation: soft marker contact and sampled feedback gains.
    # Contact parameters are assumptions, not identified material properties.
    defaults=default_plant_parameters(experiment,profile,c_profile,grinder_model)
    plant_parameters=replace(plant_parameters or defaults,timestep=dt/physics_substeps)
    sensor_parameters=sensor_parameters or default_sensor_parameters(profile,plant_parameters)
    actuator_parameters=actuator_parameters or ActuatorParameters(cutoff_hz=0 if profile=='legacy' else 100)
    if ideal:
        plant_parameters=replace(plant_parameters,friction_scale=0,damping_scale=0,mass_error=0,
                                 tool_mass=0,nominal_tool_mass=0,rotor_mass=0)
        sensor_parameters=SensorParameters(q_std=0,v_std=0,force_std=0,moment_std=0,tool_calibration_mass=0)
    nominal=build_model(plant_parameters,nominal=True,visuals=False)
    initial_q=HOME.copy()
    if profile in ('paper','fr3','fr3v2') and experiment in ('A','B','F'):
        # Translate Fig. 6's 260 mm path into the FR3 workspace instead of
        # shrinking it or continuing through the extended-arm singularity.
        initial_q[[1,3,5]]=[-.5,-2.3,1.8]
    initial=home_data(nominal,initial_q);site=nominal.site('tcp').id
    origin=initial.site_xmat[site].reshape(3,3).copy()
    initial_pose=np.r_[initial.site_xpos[site],np.zeros(3)]
    scenario=Scenario(experiment,initial_pose,trial,variant=scenario_variant,initial_q=initial_q,
                      grinder_model=grinder_model,impedance_overrides=impedance,
                      operator_stiffness_scale=operator_stiffness_scale,operator_torque_scale=operator_torque_scale,
                      operator_roll_hold=operator_roll_hold,operator_model=operator_model,
                      operator_replay=operator_replay)
    plant=build_model(plant_parameters,environment=scenario.environment(),visuals=False)
    data=home_data(plant,initial_q)
    grinding=GrindingContact(plant,plant_parameters)
    tcp=plant.site('tcp').id;tip=plant.geom('tool_tip').id;body=plant.body('tool').id
    dyn=NominalDynamics(nominal,origin)
    settings=configuration['control_settings'].copy()
    settings.update(control_overrides or {})
    ctrl=make_controller(controller,dyn,scenario,dt,**settings)
    sensor=Sensors(sensor_parameters,seed,dt)
    data.ctrl[:7]=np.clip(dyn.evaluate(initial_q,np.zeros(7)).gravity,-TORQUE_LIMIT,TORQUE_LIMIT)
    mujoco.mj_forward(plant,data)
    interface=TorqueInterface(actuator_parameters,dt,data.ctrl[:7].copy())
    duration=scenario.duration if duration is None else duration
    steps=round(duration/dt)
    names=['time','q','qvel','q_measured','v_measured','pose','velocity','desired','force_desired',
           'force_true','force_contact','force_measured','force_raw_local','torque_command','torque_filtered','torque_applied',
           'reference_error','beta','tank','saturated','jacobian_sigma_min','interaction_work',
           'board_contact','threshold_contact','environment_contact','plant_q','plant_qvel',
           'rotor_motor_torque','rotor_motor_power','grinding_power','grinding_wrench','grinding_normal_load']
    diagnostic_defaults={name:np.zeros(size) for name,size in [('proxy_pose',6),('proxy_velocity',6),
                         ('proxy_q',7),('proxy_v',7),('compensation_torque',7)]}
    diagnostic_defaults.update({name:0. for name in ['power_environment','power_environment_exact',
        'power_force_command','power_compensation','power_generation','storage_real','storage_nominal',
        'beta_candidate','tank_dissipation','tank_jump','tank_mu']})
    records={name:[] for name in names+list(diagnostic_defaults)}
    status='completed';reason=None;work=0.;started=time.perf_counter();step=-1;completed_steps=0
    print(f'START {experiment} {controller} trial={trial} duration={duration:g}s',flush=True)
    metadata=dict(experiment=experiment,controller=controller,trial=trial,seed=seed,profile=profile,
                  requested_duration_s=duration,ideal=ideal,scenario_variant=scenario_variant,initial_q=initial_q.tolist(),
                  grinder_model=grinder_model if experiment=='E' else None,
                  operator_stiffness_scale=operator_stiffness_scale,
                  operator_torque_scale=scenario.operator_torque_scale.tolist(),
                  operator_roll_hold=scenario.operator_roll_hold,operator_model=operator_model,
                  operator_replay=getattr(scenario,'operator_replay',None),
                  control_settings=settings,actuator=asdict(actuator_parameters),c_profile=c_profile if experiment=="C" else None,
                  controller_parameters={**({"alpha":ctrl.alpha} if controller.startswith("hinf") else {}),
                                         **({"G":ctrl.G.tolist()} if hasattr(ctrl,"G") else {})},
                  control_period_s=dt,log_period_s=.001,log_stride=log_stride,
                  physics_substeps=physics_substeps,plant=asdict(plant_parameters),
                  sensors=asdict(sensor_parameters),mujoco_version=mujoco.__version__,
                  source_sha256=source_hash(),impedance=dict(inertia=scenario.inertia.tolist(),
                    damping=scenario.damping.tolist(),stiffness=scenario.stiffness.tolist()))
    (folder/'config.json').write_text(json.dumps(metadata,indent=2))
    try:
        for step in range(steps):
            t=step*dt
            # Causal sample: the sensor observes the state and held command
            # from the preceding control interval, before the next torque.
            mujoco.mj_forward(plant,data)
            pose,velocity,twist=physical_pose(plant,data,tcp,origin)
            applied=scenario.applied_wrench(t,pose,velocity)
            data.xfrc_applied[:]=0
            data.xfrc_applied[body,:3]=applied[:3]
            data.xfrc_applied[body,3:]=applied[3:]+np.cross(data.site_xpos[tcp]-data.xipos[body],applied[:3])
            mujoco.mj_forward(plant,data)
            grinding.update(data)
            if grinding.active:mujoco.mj_forward(plant,data)
            contact=contact_wrench(plant,data,tip,data.site_xpos[tcp])+grinding.wrench
            physical=contact+applied
            q,v=sensor.joints(data.qpos[:7].copy(),data.qvel[:7].copy())
            state=dyn.evaluate(q,v)
            Rmeas=Rotation.from_euler('xyz',state.p[3:]).as_matrix()@origin
            measured=sensor.wrench(step,data.sensordata[:6],Rmeas,
                                   0. if ideal else scenario.extra_force_noise)
            result=ctrl.step(t,state,measured)
            command=result.torque
            if not np.isfinite(command).all():
                raise FloatingPointError('Non-finite commanded torque')
            filtered=interface.step(command)
            data.ctrl[:7]=filtered # MJCF motor and joint limits clamp physically.
            work+=dt*float(physical@twist)
            if step%log_stride==0:
                ref=scenario.reference(t)
                values=[t,data.qpos[:7].copy(),data.qvel[:7].copy(),q,v,pose,velocity,ref.p,ref.force,
                        physical,contact,measured,data.sensordata[:6].copy(),command,filtered,
                        np.clip(filtered,-TORQUE_LIMIT,TORQUE_LIMIT),result.reference_error,
                        result.beta,result.tank,float(np.any(abs(filtered)>TORQUE_LIMIT)),state.sigma_min,work]
                active_geoms=set()
                for contact_index in range(data.ncon):
                    pair=data.contact[contact_index]
                    if tip in (pair.geom1,pair.geom2) and pair.efc_address>=0:
                        active_geoms.update((pair.geom1,pair.geom2))
                values.extend(float(mujoco.mj_name2id(plant,mujoco.mjtObj.mjOBJ_GEOM,name) in active_geoms)
                              for name in ('whiteboard','threshold'))
                values.append(float(bool(active_geoms)))
                motor_torque=float(data.actuator_force[7]) if plant.nu>7 else 0.
                motor_power=motor_torque*data.qvel[7] if plant.nu>7 else 0.
                values.extend([data.qpos.copy(),data.qvel.copy(),motor_torque,motor_power,
                               grinding.power,grinding.wrench.copy(),grinding.normal_load])
                for name,value in zip(names,values):records[name].append(value)
                for name,default in diagnostic_defaults.items():
                    records[name].append(result.diagnostics.get(name,default))
            if np.max(abs(data.qvel[:7]))>15:
                raise FloatingPointError('Joint speed exceeds numerical experiment limit (15 rad/s)')
            if np.linalg.norm(physical[:3])>400:
                raise FloatingPointError('Tool contact force exceeds experiment cutoff (400 N)')
            if np.any(data.qpos[:7]<plant.jnt_range[:7,0]-.05) or np.any(data.qpos[:7]>plant.jnt_range[:7,1]+.05):
                raise FloatingPointError('Joint limit exceeded by 0.05 rad')
            for _ in range(physics_substeps):
                if grinding.active:
                    mujoco.mj_forward(plant,data)
                    grinding.update(data)
                mujoco.mj_step(plant,data)
            if not np.isfinite(data.qpos).all() or data.time<t:
                raise FloatingPointError('MuJoCo state invalid or automatically reset')
            completed_steps+=1
    except (FloatingPointError,np.linalg.LinAlgError) as exc:
        status='terminated';reason=str(exc)
    log={k:np.asarray(v,dtype=np.float64 if k in ('time','plant_q','plant_qvel') else np.float32)
         for k,v in records.items()}
    np.savez_compressed(folder/'trajectory.npz',**log)
    metrics=metrics_from_log(log,scenario) if len(log['time']) else {}
    metadata.update(status=status,termination_reason=reason,completed_control_steps=completed_steps,
                    attempted_control_steps=step+1,
                    simulated_duration_s=float(data.time),wall_time_s=time.perf_counter()-started,
                    force_samples=sensor.force_sample_count,metrics=metrics)
    if hasattr(ctrl,'tank'):metadata['tank_depletion_count']=ctrl.tank.depletion_count
    (folder/'summary.json').write_text(json.dumps(metadata,indent=2))
    print(f'END {experiment} {controller}: {status}, t={data.time:.3f}s, '
          f'position RMSE={metrics.get("position_rmse_m",float("nan")):.5f}m; {reason or ""}',flush=True)
    return metadata
