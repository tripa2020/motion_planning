import numpy as np

from cs4750 import utils
from planning import dubins

import numpy as np
from matplotlib import pyplot as plt
import rospy
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


class PlanarProblem(object):
    def __init__(self, permissible_region, map_info=None, check_resolution=0.1):
        """Construct a planar planning problem.

        Args:
            permissible_region: Boolean np.array with shape map height x map width,
                where one indicates that the location is permissible
            map_info: map information, returned by get_map
            check_resolution: collision-checking resolution
        """
        self.permissible_region = permissible_region
        self.map_info = map_info
        self.check_resolution = check_resolution

        height, width = permissible_region.shape
        self.extents = np.zeros((3, 2))
        self.extents[0, 1] = width
        self.extents[1, 1] = height
        self.name = "CarSpace"

        if map_info is not None:
            map_angle = utils.quaternion_to_angle(map_info.origin.orientation)
            assert map_angle == 0
            utils.map_to_world(self.extents.T, map_info)
        self.extents = self.extents[:2, :]

    def check_state_validity(self, states):
        """Return whether states are valid.

        Valid states are within the extents of the map and collision-free.

        Args:
            states: np.array with shape N x D (where D may be 2 or 3)

        Returns:
            valid: np.array of Booleans with shape N
        """
        x = states[:, 0]
        y = states[:, 1]
        valid = np.ones_like(x, dtype=bool)  # feel free to delete this line

        # Check that x and y are within the extents of the map.
        xmin, xmax = self.extents[0, :]
        ymin, ymax = self.extents[1, :]
        within_x = (xmin <= x) & (x < xmax)
        within_y = (ymin <= y) & (y < ymax)
        valid = within_x & within_y

        # The units of the state are meters and radians. We need to convert the
        # meters to pixels, in order to index into the permissible region. This
        # function converts them in place.
        if self.map_info is not None:
            utils.world_to_map(states, self.map_info)

        yind = y.astype(int)
        xind = x.astype(int)
        coll_free = self.permissible_region[yind[valid], xind[valid]]
        valid[valid] = coll_free

        # Convert the units back from pixels to meters for the caller
        if self.map_info is not None:
            utils.map_to_world(states, self.map_info)

        return valid

    def check_edge_validity(self, q1, q2):
        """Return whether an edge is valid.

        Args:
            q1, q2: np.arrays with shape (1, D) (where D may be 2, 3 or 6)

        Returns:
            valid: True or False
        """

        path, length = self.steer(q1, q2)
        if length == 0:
            return False
        return self.check_state_validity(path).all()

    def cost_to_go(self, q1, q2):
        """Compute an admissible heuristic between two states.
        Args:
            q1, q2: np.arrays with shape (N, D) (where D may be 2, 3 or 6)
        Returns:
            heuristic: np.array with shape N of cost estimates between pairs of states
        """
        # Subclasses implement this.
        raise NotImplementedError

    def compute_distance(self, q1, q2):
        """Compute cost between two states.
        Args:
            q1, q2: np.arrays with shape (N, D) (where D may be 2, 3 or 6)
        Returns:
            heuristic: np.array with shape N of cost estimates between pairs of states
        """
        # Subclasses implement this.
        raise NotImplementedError
       
        

    def steer(self, q1, q2, **kwargs):
        """Return a local path connecting two states.
           Implemented by subclasses.

        Intermediate states are used for edge collision-checking.

        Args:
            q1, q2: np.arrays with shape (1, D) (where D may be 2, 3 or 6)

        Returns:
            path: sequence of states between q1 and q2
            length: length of local path
        """
        # Subclasses implement this.
        raise NotImplementedError


class R2Problem(PlanarProblem):
    def compute_distance(self, q1, q2):
        """Computes the Euclidean distance between corresponding elements of two arrays of states.

        Note that q1 and q2 may be of shape (2,) or (N,2) so make sure to use numpy broadcasting.

        Args:
            q1: np.array corresponding to (one or more) R2 states of dimension 2
            q2: np.array corresponding to (one or more) R2 states of dimension 2

        Returns:
            costs: np.array of shape (N,)
        """
        q1_2d = np.atleast_2d(q1)
        q2_2d = np.atleast_2d(q2)
        ### BEGIN QUESTION 1.1 #####################
        diff = q1_2d - q2_2d

        
        costs = np.linalg.norm(diff, axis=1)

        return costs
        ### END QUESTION 1.1 #######################
    
    def cost_to_go(self, q1, goal):
        """Computes the Euclidean distance between each element of an array of R2 states and a goal R2 state (used by AStar).

        Args:
            q1: np.array with shape (N, 2), N states where each row is a R2 state of dimension (1,2)
            goal: np.array with shape (1, 2), a goal R2 state

        Returns:
            heuristic: np.array with shape (N,), where element at index i is the Euclidean distance between
                       R2 state at index i of q1 and goal R2 state.
        """
        return self.compute_distance(q1,goal)
    

    def compute_distance_rrt(self, q1, q2):
        """Computes the Euclidean distance between two states (used by RRT).

        Args:
            q1: np.array with shape (1, 2), a R2 state
            q2: np.array with shape (1, 2), a R2 state

        Returns:
            heuristic: scalar float, Euclidean distance between R2 states q1 and q2
        """
        return np.ndarray.item(np.floor(self.compute_distance(q1, q2)))

    def steer(self, q1, q2, resolution=None, interpolate_line=True):
        """Return a straight-line path connecting two R2 states.

        Args:
            q1, q2: np.arrays with shape (1, 2)
            resolution: the space between waypoints in the resulting path
            interpolate_line: whether to provide fine waypoint discretization
             for line segments

        Returns:
            path: sequence of states between q1 and q2
            length: length of local path
        """
        if resolution is None:
            resolution = self.check_resolution
        q1 = q1.reshape((1, -1))
        q2 = q2.reshape((1, -1))
        dist = np.linalg.norm(q2 - q1)
        if not interpolate_line or dist < resolution:
            return np.vstack((q1, q2)), dist
        q1_toward_q2 = (q2 - q1) / dist
        steps = np.hstack((np.arange(0, dist, resolution), np.array([dist]))).reshape(
            (-1, 1)
        )
        return q1 + q1_toward_q2 * steps, dist


class SE2Problem(PlanarProblem):
    def __init__(
        self, permissible_region, map_info=None, check_resolution=0.01, curvature=1.0
    ):
        super(SE2Problem, self).__init__(permissible_region, map_info, check_resolution)
        self.curvature = curvature
        self.extents = np.vstack((self.extents, np.array([[-np.pi, np.pi]])))

    def compute_distance(self, q1, q2):
        """Computes the length of the Dubins path between corresponding elements of two arrays of SE(2) states

        Note that q1 and q2 may be of shape (3,) or (N,3) so make sure to use numpy broadcasting.

        Args:
            q1: np.array corresponding to (one or more) SE(2) states of dimension 3
            q2: np.array corresponding to (one or more) SE(2) states of dimension 3

        Returns:
            costs: np.array with shape (N,)
        """
        q1_2d = np.atleast_2d(q1)
        q2_2d = np.atleast_2d(q2)
        ### BEGIN QUESTION 4 #####################
        costs = dubins.path_length(q1_2d, q2_2d, self.curvature)
        return costs
        ### END QUESTION 4 #######################

    def cost_to_go(self, q1, goal):
        """Computes the length of the Dubins path between each element of an array of SE(2) states and a goal SE(2) state (used by AStar).

        Args:
            q1: np.array with shape (N, 3), N states where each row is a SE(2) state of dimension (1,3)
            goal: np.array with shape (1, 3), a goal SE(2) state

        Returns:
            heuristic: np.array with shape (N,), where element at index i is the length of the Dubins path between
                       SE(2) state at index i of q1 and goal SE(2) state.
        """
        return self.compute_distance(q1, goal)

    def steer(self, q1, q2, resolution=None, interpolate_line=True):
        """Return a Dubins path connecting two SE(2) states.

        Args:
            q1, q2: np.arrays with shape (1, 3)
            resolution: the space between waypoints in the resulting path
            interpolate_line: whether to provide fine waypoint discretization
             for line segments

        Returns:
            path: sequence of states on Dubins path between q1 and q2
            length: length of local path
        """
        if resolution is None:
            resolution = self.check_resolution
        path, length = dubins.path_planning(
            q1,
            q2,
            self.curvature,
            resolution=resolution,
            interpolate_line=interpolate_line,
        )
        return path, length


class JointSpace(object):
    
    def __init__(self):
        self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        self.sv_srv.wait_for_service()
        self.rs = RobotState()
        self.rs.joint_state.name = ['waist','shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        self.rs.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.name = "JointSpace"

    def interpolate(self, start_config, goal_config, step_size):
        return (1 - step_size) * start_config + step_size * goal_config


    def compute_distance_rrt(self, start_config, goal_config):
        start_config = start_config.reshape((-1,6))
        goal_config = goal_config.reshape((-1,6))
        ### BEGIN QUESTION 6.1 #####################
        '''
        Computes the time spent in executing motion between two states (used by RRT).

        Args: 
            start_config: shape (1, 6), start joint configuration
            goal_config: shape(1, 6), goal joint configuration

        Returns:
            time: float scalar, time spent in executing motion between two states
        '''
        joint_velocities = np.array([1.0, 1.2, 0.8, 0.75, 1.05, 0.9])
        diff = np.abs(start_config - goal_config) % (2 * np.pi)
        diff = np.minimum(diff, 2 * np.pi - diff)
        times = diff / joint_velocities
        time = np.max(times)
        return time
        ### END QUESTION 6.1 #######################

        # joint_velocities = np.array([1.0, 1.2, 0.8, 0.75, 1.05, 0.9])
        # diff = np.abs(start_config - goal_config) % (2 * np.pi)
        # diff = np.minimum(diff, 2 * np.pi - diff)
        # times = diff / joint_velocities
        # time = np.sum(times)
        # return time



    def state_validity_checker(self, q):
        q = q.reshape((6,))
        self.rs.joint_state.position = q.tolist()
        return self.get_state_validity().valid

    def get_state_validity(self, group_name='widowx250_manipulator'):
        '''
        Given a RobotState and a group name and an optional Constraints
        return the validity of the State
        '''
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self.rs
        gsvr.group_name = group_name
        result = self.sv_srv.call(gsvr)
        # rospy.sleep(0.1)
        return result

    def arm_state_validity_checker(self, qs):
        """Return whether states are valid.
        Valid states are within the extents of the map and collision-free.
        Args:
            states: np.array with shape N x D (where D may be 2 or 3 or 6)
        Returns:
            valid: np.array of Booleans with shape N
        """
        valid = np.zeros((qs.shape[0],), dtype=bool)
        for i in range(qs.shape[0]):
            self.rs.joint_state.position = qs[i].tolist()
            valid[i] = self.get_state_validity().valid
        return valid

    def sample(self):
        # Sample random clear point from map
        lower_bound = np.array([-3.14, -1.88, -2.15, -3.14, -1.75, -3.14])
        upper_bound = np.array([3.14, 1.99, 1.61, 3.14, 2.15, 3.14])
        q_rand = np.random.uniform(low = lower_bound, high = upper_bound, size = (1, 6))
        while (not self.state_validity_checker(q_rand)):
            q_rand = np.random.uniform(low = lower_bound, high = upper_bound, size = (1, 6))
        return q_rand

    def check_edge_validity(self, config1, config2):
        config1.reshape((1, 6))
        config2.reshape((1, 6))
    
        valid = True
        t = 0.0
        while t <= 1.0:
            valid *= self.state_validity_checker(self.interpolate(config1, config2, t))
            t += 0.1
        return bool(valid)

    def compute_distance(self, start_config, goal_config):
        """Computes the distance between corresponding elements of two joint states

        Note that q1 and q2 may be of shape (6,) or (N,6) so make sure to use numpy broadcasting.

        Args:
            start_config: np.array corresponding to (one or more) joint states of dimension 6
            goal_config : np.array corresponding to (one or more) joint states of dimension 6

        Returns:
            costs: np.array with shape (N,)
        """
        start_config_2d = np.atleast_2d(start_config)
        goal_config_2d = np.atleast_2d(goal_config)
        distance = np.zeros(max(start_config_2d.shape[0], goal_config_2d.shape[0]))
        for i in range(6):
            # Angular distance calculation
            # Source: https://stackoverflow.com/questions/1878907/how-can-i-find-the-smallest-difference-between-two-angles-around-a-point 
            dist = start_config_2d[:,i] - goal_config_2d[:,i]
            dist = np.abs((dist + np.pi) % (2*np.pi) - np.pi)
            distance += dist
        return distance

    def cost_to_go(self, config, goal_config):
        config_2d = np.atleast_2d(config)
        goal_config_2d = np.atleast_2d(goal_config)
        ### BEGIN QUESTION 5 #####################
        ''' 
        Come up with an admissible heuristic from config to goal_config
            
        Args:
            config: np.array of shape (1, 6)
            goal_config: np.array of shape (1, 6)

        Returns:
            heuristic: scalar float, admissible heuristic that is non-negative and does not overestimate distance from config to goal_config
        '''
        diff = np.abs(config_2d - goal_config_2d) % (2 * np.pi)
        diff = np.minimum(diff, 2 * np.pi - diff)
        heuristic = np.sum(diff)
        return heuristic
        ### END QUESTION 5 #######################


