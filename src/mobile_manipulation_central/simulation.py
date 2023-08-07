import numpy as np
import pybullet as pyb
import pybullet_data


class BulletSimulation:
    def __init__(self, timestep, gravity=(0, 0, -9.81), gui=True, extra_gui=False):
        """Basic PyBullet simulation.

        Parameters:
            timestep: Simulation timestep.
            gravity: (Optional) Defaults to [0, 0, -9.81].
            extra_gui: (Optional) True to show extra parts of the GUI. Defaults
                to False.
        """
        self.timestep = timestep
        self.gravity = np.array(gravity)

        if gui:
            self.client_id = pyb.connect(pyb.GUI, options="--width=1280 --height=720")
        else:
            self.client_id = pyb.connect(pyb.DIRECT)
        pyb.setGravity(*gravity)
        pyb.setTimeStep(self.timestep)

        pyb.resetDebugVisualizerCamera(
            cameraDistance=4,
            cameraYaw=42,
            cameraPitch=-35.8,
            cameraTargetPosition=[1.28, 0.045, 0.647],
        )

        # get rid of extra parts of the GUI unless desired
        if not extra_gui:
            pyb.configureDebugVisualizer(pyb.COV_ENABLE_GUI, 0)

        # setup ground plane
        # save its UID to reference it later (e.g., to change its friction)
        pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.ground_uid = pyb.loadURDF("plane.urdf", [0, 0, 0])

    def step(self, t=0):
        """Step the simulation forward in time by one timestep.

        Returns the incremented simulation time.
        """
        pyb.stepSimulation()
        return t + self.timestep

    def settle(self, duration):
        """Run simulation while doing nothing.

        Useful to let objects settle to rest before applying control.
        """
        t = 0
        while t < duration:
            pyb.stepSimulation()
            t += self.timestep


class BulletSimulatedRobot:
    def __init__(
        self,
        urdf_path,
        tool_joint_name,
        position=(0, 0, 0),
        orientation=(0, 0, 0, 1),
        actuated_joints=None,
        locked_joints=None,
    ):
        """Robot in a PyBullet simulation.

        Parameters:
            urdf_path: Path to URDF file represented the robot.
            tool_joint_name: Name of the joint for the main end effector/tool.
            position: (Optional) Position at which to load the robot.
            orientation: (Optional) Orientation at which to the load the robot.
                All orientations represented as quaternions (x, y, z, w).
            actuated_joints: (Optional) List of actuated joint names. If None
                (the default), then all joints are actuated.
            locked_joints: (Optional) Dict of {joint_name: joint_value} pairs.
                Joints in this list are fixed at the given values. If None (the
                default), then no joints are locked.
        """
        self.uid = pyb.loadURDF(urdf_path, position, orientation, useFixedBase=True)

        # build a dict of all joints, keyed by name
        self.joints = {}
        self.links = {}
        for i in range(pyb.getNumJoints(self.uid)):
            info = pyb.getJointInfo(self.uid, i)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            self.joints[joint_name] = info
            self.links[link_name] = info

        # get the indices for the actuated joints
        # if actuated joints are not named, we take all of the non-fixed joints
        self.robot_joint_indices = []
        if actuated_joints is None:
            for joint in self.joints.values():
                # joint type == 4 means a fixed joint, skip these
                if joint[2] == 4:
                    continue
                self.robot_joint_indices.append(joint[0])
        else:
            for name in actuated_joints:
                idx = self.joints[name][0]
                self.robot_joint_indices.append(idx)

        # set any locked joints to appropriate values
        self.locked_joints = {}
        if locked_joints is not None:
            for name, value in locked_joints.items():
                idx = self.joints[name][0]
                self.locked_joints[idx] = value

        # link index (of the tool, in this case) is the same as the joint
        self.tool_idx = self.joints[tool_joint_name][0]

    def reset_joint_configuration(self, q):
        """Reset the robot to a particular configuration.

        It is best not to do this during a simulation, as this overrides are
        dynamic effects.
        """
        for idx, angle in zip(self.robot_joint_indices, q):
            pyb.resetJointState(self.uid, idx, angle)

        # reset the locked/fixed joints as well
        for idx, value in self.locked_joints.items():
            pyb.resetJointState(self.uid, idx, value)

    def command_velocity(self, cmd_vel):
        """Command the velocity of the robot's joints."""
        pyb.setJointMotorControlArray(
            self.uid,
            self.robot_joint_indices,
            controlMode=pyb.VELOCITY_CONTROL,
            targetVelocities=list(cmd_vel),
        )

    def joint_states(self):
        """Get the current state of the joints.

        Return a tuple (q, v), where q is the n-dim array of positions and v is
        the n-dim array of velocities.
        """
        states = pyb.getJointStates(self.uid, self.robot_joint_indices)
        q = np.array([state[0] for state in states])
        v = np.array([state[1] for state in states])
        return q, v

    def link_pose(self, link_idx=None):
        """Get the pose of a particular link in the world frame.

        It is the pose of origin of the link w.r.t. the world. The origin of
        the link is the location of its parent joint.

        If no link_idx is provided, defaults to that of the tool.
        """
        if link_idx is None:
            link_idx = self.tool_idx
        state = pyb.getLinkState(self.uid, link_idx, computeForwardKinematics=True)
        pos, orn = state[4], state[5]
        return np.array(pos), np.array(orn)

    def link_velocity(self, link_idx=None):
        """Get the velocity of a link.

        If no link_idx is provided, defaults to that of the tool.

        Returns a tuple of (linear, angular) velocity.
        """
        if link_idx is None:
            link_idx = self.tool_idx
        state = pyb.getLinkState(
            self.uid,
            link_idx,
            computeLinkVelocity=True,
        )
        return np.array(state[-2]), np.array(state[-1])

    def jacobian(self, q=None):
        """Get the end effector Jacobian at the given configuration.

        If no configuration is given, then the current one is used.

        Returns the Jacobian matrix.
        """

        if q is None:
            q, _ = self.joint_states()
        z = list(np.zeros_like(q))
        q = list(q)

        tool_offset = [0, 0, 0]
        Jv, Jw = pyb.calculateJacobian(self.uid, self.tool_idx, tool_offset, q, z, z)
        J = np.vstack((Jv, Jw))
        return J
