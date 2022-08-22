from pathlib import Path
import numpy as np
import pinocchio
import rospkg
from spatialmath.base import r2q

from mobile_manipulation_central.ros_utils import compile_xacro


class RobotKinematics:
    def __init__(self, nq, nv, urdf_str, root_joint=None, tool_link_name=None):
        """Load the model from a URDF represented as a string."""
        self.nq = nq
        self.nv = nv

        if root_joint is not None:
            self.model = pinocchio.buildModelFromXML(urdf_str, root_joint)
        else:
            self.model = pinocchio.buildModelFromXML(urdf_str)
        self.data = self.model.createData()

        if tool_link_name is not None:
            self.tool_idx = self.model.getBodyId(tool_link_name)
        else:
            self.tool_idx = None

    @classmethod
    def from_urdf_file(
        cls, nq, nv, urdf_file_path, root_joint=None, tool_link_name=None
    ):
        """Load the model directly from a URDF file."""
        with open(urdf_file_path) as f:
            urdf_str = f.read()
        return cls(nq, nv, urdf_str, root_joint, tool_link_name)

    def forward(self, q, v=None, a=None):
        """Forward kinematics using (q, v, a) all in the world frame (i.e.,
        corresponding directly to the Pinocchio model."""
        if v is None:
            v = np.zeros(self.nv)
        if a is None:
            a = np.zeros(self.nv)

        assert q.shape == (self.nq,)
        assert v.shape == (self.nv,)
        assert a.shape == (self.nv,)

        pinocchio.forwardKinematics(self.model, self.data, q, v, a)
        pinocchio.updateFramePlacements(self.model, self.data)

    def forward_derivatives(self, q, v, a=None):
        """Compute derivatives of the forward kinematics using (q, v, a) all in
        the world frame (i.e., corresponding directly to the Pinocchio
        model."""
        if a is None:
            a = np.zeros(self.nv)

        assert q.shape == (self.nq,)
        assert v.shape == (self.nv,)
        assert a.shape == (self.nv,)

        pinocchio.computeForwardKinematicsDerivatives(self.model, self.data, q, v, a)

    def link_pose(self, link_idx=None):
        """Get pose of link at index link_idx.

        Must call forward(q, ...) first.

        Returns a tuple (position, quaternion).
        """
        if link_idx is None:
            link_idx = self.tool_idx
        pose = self.data.oMf[link_idx]
        r = pose.translation
        Q = r2q(pose.rotation, order="xyzs")
        return r.copy(), Q.copy()

    def link_velocity(self, link_idx=None):
        """Get velocity of link at index link_idx"""
        if link_idx is None:
            link_idx = self.tool_idx
        V = pinocchio.getFrameVelocity(
            self.model,
            self.data,
            link_idx,
            pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        return V.linear, V.angular

    def link_classical_acceleration(self, link_idx=None):
        if link_idx is None:
            link_idx = self.tool_idx
        A = pinocchio.getFrameClassicalAcceleration(
            self.model,
            self.data,
            link_idx,
            pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        return A.linear, A.angular

    def link_spatial_acceleration(self, link_idx=None):
        if link_idx is None:
            link_idx = self.tool_idx
        A = pinocchio.getFrameAcceleration(
            self.model,
            self.data,
            link_idx,
            pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        return A.linear, A.angular

    def jacobian(self, q):
        """Compute the robot geometric Jacobian."""
        return pinocchio.computeFrameJacobian(
            self.model,
            self.data,
            q,
            self.tool_idx,
            pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

    def link_velocity_derivatives(self, link_idx):
        """Compute derivative of link velocity with respect to q and v."""
        dVdq, dVdv = pinocchio.getFrameVelocityDerivatives(
            self.model,
            self.data,
            link_idx,
            pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        return dVdq, dVdv

    def link_classical_acceleration_derivatives(self, link_idx=None):
        """Compute derivative of link classical acceleration with respect to q, v, a."""
        dr, ω = self.link_velocity(link_idx=link_idx)
        dVdq, dVdv = self.link_velocity_derivatives(link_idx=link_idx)
        dAdq, dAdv, dAda = self.link_spatial_acceleration_derivatives(link_idx=link_idx)

        # derivative of the coriolis term
        ddrdq, dwdq = dVdq[:3, :], dVdq[3:, :]
        ddrdv, dwdv = dVdv[:3, :], dVdv[3:, :]
        dcdq = (np.cross(dwdq.T, dr) + np.cross(ω, ddrdq.T)).T
        dcdv = (np.cross(dwdv.T, dr) + np.cross(ω, ddrdv.T)).T

        # add the coriolis term to the spatial acceleration
        dAs_dq = dAdq + np.vstack((dcdq, np.zeros((3, self.dims.q))))
        dAs_dv = dAdv + np.vstack((dcdv, np.zeros((3, self.dims.v))))
        dAs_da = dAda

        return dAs_dq, dAs_dv, dAs_da

    def link_spatial_acceleration_derivatives(self, link_idx=None):
        """Compute derivative of link spatial acceleration with respect to q, v, a."""
        if link_idx is None:
            link_idx = self.tool_idx
        _, dAdq, dAdv, dAda = pinocchio.getFrameAccelerationDerivatives(
            self.model,
            self.data,
            link_idx,
            pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        return dAdq, dAdv, dAda


class MobileManipulatorKinematics(RobotKinematics):
    def __init__(
        self, filename="thing_no_wheels.urdf.xacro", tool_link_name="gripped_object"
    ):
        rospack = rospkg.RosPack()
        xacro_path = (
            Path(rospack.get_path("mobile_manipulation_central"))
            / "urdf/xacro"
            / filename
        )
        urdf_str = compile_xacro(xacro_path)

        # 3-DOF base joint
        root_joint = pinocchio.JointModelComposite(3)
        root_joint.addJoint(pinocchio.JointModelPX())
        root_joint.addJoint(pinocchio.JointModelPY())
        root_joint.addJoint(pinocchio.JointModelRZ())

        super().__init__(
            nq=9,
            nv=9,
            urdf_str=urdf_str,
            root_joint=root_joint,
            tool_link_name=tool_link_name,
        )
