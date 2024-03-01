from pathlib import Path
import numpy as np
import pinocchio
import rospkg
from spatialmath.base import r2q
from xacrodoc import XacroDoc

from mobile_manipulation_central.ros_utils import package_file_path


def _ref_frame_from_string(s):
    """Translate a string to pinocchio's ReferenceFrame enum value."""
    s = s.lower()
    if s == "local":
        return pinocchio.ReferenceFrame.LOCAL
    if s == "local_world_aligned":
        return pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED
    if s == "world":
        return pinocchio.ReferenceFrame.WORLD
    raise ValueError(f"{s} is not a valid Pinocchio reference frame.")


class RobotKinematics:
    """Class representing the kinematics model of a robot."""

    def __init__(self, model, tool_link_name=None):
        self.model = model
        self.nq = model.nq
        self.nv = model.nv
        self.data = self.model.createData()

        self.tool_link_name = tool_link_name
        if tool_link_name is not None:
            self.tool_idx = self.get_link_index(tool_link_name)
        else:
            self.tool_idx = None

    @classmethod
    def from_urdf_string(cls, urdf_str, root_joint=None, tool_link_name=None):
        """Load the model from a URDF represented as a string."""
        if root_joint is not None:
            model = pinocchio.buildModelFromXML(urdf_str, root_joint)
        else:
            model = pinocchio.buildModelFromXML(urdf_str)
        return cls(model, tool_link_name)

    @classmethod
    def from_urdf_file(cls, urdf_file_path, root_joint=None, tool_link_name=None):
        """Load the model directly from a URDF file."""
        with open(urdf_file_path) as f:
            urdf_str = f.read()
        return cls.from_urdf_string(urdf_str, root_joint, tool_link_name)

    def get_link_index(self, link_name):
        """Get index of a link by name."""
        # TODO: it would probably be desirable to rename "link" to "frame"
        # everywhere in this class
        if not self.model.existFrame(link_name):
            raise ValueError(f"Model has no frame named {link_name}.")
        return self.model.getFrameId(link_name)

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

    def link_pose(self, link_idx=None, rotation_matrix=False):
        """Get pose of link at index link_idx.

        Must call forward(q, ...) first.

        Returns a tuple (position, orientation). If `rotation_matrix` is True,
        then the orientation is a 3x3 matrix, otherwise it is a quaternion with
        the scalar part as the last element.
        """
        if link_idx is None:
            link_idx = self.tool_idx
        pose = self.data.oMf[link_idx]
        pos = pose.translation.copy()
        orn = pose.rotation.copy()
        if not rotation_matrix:
            orn = r2q(orn, order="xyzs")
        return pos, orn

    def link_velocity(self, link_idx=None, frame="local_world_aligned"):
        """Get velocity of link at index link_idx"""
        if link_idx is None:
            link_idx = self.tool_idx
        V = pinocchio.getFrameVelocity(
            self.model,
            self.data,
            link_idx,
            _ref_frame_from_string(frame),
        )
        return V.linear, V.angular

    def link_classical_acceleration(self, link_idx=None, frame="local_world_aligned"):
        """Get the classical acceleration of a link."""
        if link_idx is None:
            link_idx = self.tool_idx
        A = pinocchio.getFrameClassicalAcceleration(
            self.model,
            self.data,
            link_idx,
            _ref_frame_from_string(frame),
        )
        return A.linear, A.angular

    def link_spatial_acceleration(self, link_idx=None, frame="local_world_aligned"):
        """Get the spatial acceleration of a link."""
        if link_idx is None:
            link_idx = self.tool_idx
        A = pinocchio.getFrameAcceleration(
            self.model,
            self.data,
            link_idx,
            _ref_frame_from_string(frame),
        )
        return A.linear, A.angular

    def jacobian(self, q, link_idx=None, frame="local_world_aligned"):
        """Compute the robot geometric Jacobian."""
        if link_idx is None:
            link_idx = self.tool_idx
        return pinocchio.computeFrameJacobian(
            self.model,
            self.data,
            q,
            link_idx,
            _ref_frame_from_string(frame),
        )

    def jacobian_time_derivative(self, q, v, link_idx=None):
        """Compute the time derivative of the geometric Jacobian dJ/dt for a given frame."""
        raise NotImplementedError(
            "Jacobian time derivative is not implemented because it is not "
            "needed for acceleration-level control. See "
            "<https://github.com/stack-of-tasks/pinocchio/issues/1395>."
        )

    def link_velocity_derivatives(self, link_idx=None, frame="local_world_aligned"):
        """Compute derivative of link velocity with respect to q and v."""
        if link_idx is None:
            link_idx = self.tool_idx
        dVdq, dVdv = pinocchio.getFrameVelocityDerivatives(
            self.model,
            self.data,
            link_idx,
            _ref_frame_from_string(frame),
        )
        return dVdq, dVdv

    def link_classical_acceleration_derivatives(
        self, link_idx=None, frame="local_world_aligned"
    ):
        """Compute derivative of link classical acceleration with respect to q, v, a."""
        if link_idx is None:
            link_idx = self.tool_idx
        dr, ω = self.link_velocity(link_idx=link_idx, frame=frame)
        dVdq, dVdv = self.link_velocity_derivatives(link_idx=link_idx, frame=frame)
        dAdq, dAdv, dAda = self.link_spatial_acceleration_derivatives(
            link_idx=link_idx, frame=frame
        )

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

    def link_spatial_acceleration_derivatives(
        self, link_idx=None, frame="local_world_aligned"
    ):
        """Compute derivative of link spatial acceleration with respect to q, v, a."""
        if link_idx is None:
            link_idx = self.tool_idx
        _, dAdq, dAdv, dAda = pinocchio.getFrameAccelerationDerivatives(
            self.model,
            self.data,
            link_idx,
            _ref_frame_from_string(frame),
        )
        return dAdq, dAdv, dAda


class MobileManipulatorKinematics(RobotKinematics):
    def __init__(self, filepath=None, tool_link_name="gripper"):
        if filepath is None:
            filepath = package_file_path(
                "mobile_manipulation_central", "urdf/xacro/thing_no_wheels.urdf.xacro"
            )
        urdf_str = XacroDoc.from_file(filepath).to_urdf_string()

        # 3-DOF base joint
        root_joint = pinocchio.JointModelComposite(3)
        root_joint.addJoint(pinocchio.JointModelPX())
        root_joint.addJoint(pinocchio.JointModelPY())
        root_joint.addJoint(pinocchio.JointModelRZ())

        model = pinocchio.buildModelFromXML(urdf_str, root_joint)

        super().__init__(model, tool_link_name)
