import rclpy
from rclpy.node import Node
import numpy as np
from numpy.typing import NDArray

from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory

# TODO: Ensure this library is findable and the method is implemented
from transform_helpers.utils import rotmat2q

class YoubotKinematicBase(Node):
    def __init__(self, tf_suffix=''):
        super().__init__('youbot_kinematic_base')
        # Robot variables
        # Identify class used when broadcasting tf with a suffix
        self.tf_suffix = tf_suffix
	
        youbot_dh_parameters = {'a': [-0.033, 0.155, 0.135, +0.002, 0.0],
                                'alpha': [np.pi / 2, 0.0, 0.0, np.pi / 2, np.pi],
                                'd': [0.145, 0.0, 0.0, 0.0, -0.185],
                                'theta': [np.pi, np.pi / 2, 0.0, -np.pi / 2, np.pi]}
        
        self.dh_params = youbot_dh_parameters.copy()

        # Set current joint position
        self.current_joint_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # Set joint limits
        self.joint_limit_min = np.array([-169 * np.pi / 180, -65 * np.pi / 180, -150 * np.pi / 180,
                                         -102.5 * np.pi / 180, -167.5 * np.pi / 180])
        self.joint_limit_max = np.array([169 * np.pi / 180, 90 * np.pi / 180, 146 * np.pi / 180,
                                         102.5 * np.pi / 180, 167.5 * np.pi / 180])
                                         

        # ROS related
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            5)
        self.joint_state_sub  # prevent unused variable warning
        self.traj_publisher = self.create_publisher(JointTrajectory, '/EffortJointInterface_trajectory_controller/command', 5)
        self.traj_publisher
        # Initialize the transform broadcaster
        self.pose_broadcaster = TransformBroadcaster(self)


    def joint_state_callback(self, msg):
        """ ROS callback function for joint states of the robot. Broadcasts the current pose of end effector.

        Args:
            msg (JointState): Joint state message containing current robot joint position.

        """
        self.current_joint_position = list(msg.position)
        current_pose = self.forward_kinematics(self.current_joint_position)
        self.broadcast_pose(current_pose)

    def broadcast_pose(self, pose):
        """Given a pose transformation matrix, broadcast the pose to the TF tree.

        Args:
            pose (np.ndarray): Transformation matrix of pose to broadcast.

        """
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'arm_end_effector_' + self.tf_suffix

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = rotmat2q(pose[:3, :3])

        self.pose_broadcaster.sendTransform(transform)

    def forward_kinematics(self, joint_readings, up_to_joint=5):
        """This function solves forward kinematics by multiplying frame transformation up until a specified
        frame number. The frame transformation used in the computation are derived from dh parameters found in the
        init method and joint_readings.
        Args:
            joint_readings (list): the state of the robot joints. In a youbot those are revolute
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 5.
        """
        raise NotImplementedError

    def get_jacobian(self, joint):
        """Compute Jacobian given the robot joint values. Implementation found in child classes.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute.
        Returns: Jacaobian matrix.

        """
        raise NotImplementedError

    @staticmethod
    def standard_dh(a, alpha, d, theta):
        """This function computes the homogeneous 4x4 transformation matrix T_i based on the four standard DH parameters
         associated with link i and joint i.
        Args:
            a ([int, float]): Link Length. The distance along x_i ( the common normal) between z_{i-1} and z_i
            alpha ([int, float]): Link twist. The angle between z_{i-1} and z_i around x_i.
            d ([int, float]): Link Offset. The distance along z_{i-1} between x_{i-1} and x_i.
            theta ([int, float]): Joint angle. The angle between x_{i-1} and x_i around z_{i-1}
        Returns:
            [np.ndarray]: the 4x4 transformation matrix T_i describing  a coordinate transformation from
            the concurrent coordinate system i to the previous coordinate system i-1
        """
        assert isinstance(a, (int, float)), "wrong input type for a"
        assert isinstance(alpha, (int, float)), "wrong input type for =alpha"
        assert isinstance(d, (int, float)), "wrong input type for d"
        assert isinstance(theta, (int, float)), "wrong input type for theta"
        A = np.zeros((4, 4))

        # TODO: implement a method to get the transform matrix using DH Parameters
        raise NotImplementedError
        assert isinstance(A, np.ndarray), "Output wasn't of type ndarray"
        assert A.shape == (4, 4), "Output had wrong dimensions"
        return A

    def rotmat2rodrigues(self, T):
        """Convert transformation matrix to rodrigues vector. Done by first converting the rotation to quaternion then
	to rodrigues.

        Args:
            T (np.ndarray): 4x4 transformation matrix to convert to state vector - translation plus rodrigues rotation
	    representation.

        Returns:
            p (np.ndarray): An array where the first 3 elements specify the translation and the last three specify the
	    rotation.
        """
        assert isinstance(T, np.ndarray)

        # TODO: Implement a method to convert from a 4x4 transformation matrix to a translation and rodrigues vector

        p = np.empty(6, float)
        raise NotImplementedError
        return p
