"""Dissipative sliding contact for the fast rotating grinder disk.

Normal contact comes from MuJoCo. Tangential force is bounded by mu * Fn and
opposes relative slip at each physics step; it is applied to the disk, so its
motor supplies the grinding power. This avoids the soft friction-cone solver's
normal-force inflation at tens of metres/second. No material removal is modeled.
"""
import mujoco
import numpy as np


class GrindingContact:
    def __init__(self,model,parameters):
        self.model=model
        self.body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'grinder_rotor')
        self.active=self.body>=0 and parameters.grinder_contact=='disc'
        self.tip=model.geom('tool_tip').id
        self.tcp=model.site('tcp').id
        self.friction=parameters.contact_friction
        self.wrench=np.zeros(6)
        self.power=0.
        self.normal_load=0.
        self.jp=np.zeros((3,model.nv));self.jr=self.jp.copy()
        self.contact_force=np.zeros(6)

    def update(self,data):
        self.wrench[:]=0.;self.power=0.;self.normal_load=0.
        if not self.active:return
        data.xfrc_applied[self.body]=0.
        for index in range(data.ncon):
            contact=data.contact[index]
            if self.tip not in (contact.geom1,contact.geom2) or contact.efc_address<0:continue
            mujoco.mj_contactForce(self.model,data,index,self.contact_force)
            normal_force=max(0.,self.contact_force[0])
            normal=contact.frame[:3]
            mujoco.mj_jac(self.model,data,self.jp,self.jr,contact.pos,self.body)
            velocity=self.jp@data.qvel  # workpiece is fixed to the world
            slip=velocity-normal*(normal@velocity)
            force=-self.friction*normal_force*slip/np.sqrt(slip@slip+.01**2)
            data.xfrc_applied[self.body,:3]+=force
            data.xfrc_applied[self.body,3:]+=np.cross(contact.pos-data.xipos[self.body],force)
            self.wrench[:3]+=force
            self.wrench[3:]+=np.cross(contact.pos-data.site_xpos[self.tcp],force)
            self.power+=float(force@slip)
            self.normal_load+=normal_force
