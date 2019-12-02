import rospy
import numpy as np

from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, Quaternion
from maneuver_navigation.msg import Goal as ManeuverNavigationGoal
from maneuver_navigation.msg import Feedback as ManeuverNavigationFeedback
from ropod_ros_msgs.msg import GoToFeedback, Status
import route_navigation.transformations as tf

class ManeuverNavConfigParams(object):
    '''Configuration parameters for the maneuver navigation component.
    '''
    def __init__(self, append_maneuver=False, precise_goal=False, use_line_planner=False):
        self.append_maneuver = append_maneuver
        self.precise_goal = precise_goal
        self.use_line_planner = use_line_planner

def get_feedback_msg_skeleton(action_id, action_type):
    '''Returns a GoToFeedback message with the following fields prefilled:
    * feedback.action_id
    * feedback.type
    * feedback.status.domain, and
    * feedback.status.module_code

    Keyword arguments:
    action_id: str -- ID of the action
    action_type: str -- elevator action type

    '''
    feedback_msg = GoToFeedback()
    feedback_msg.feedback.action_id = action_id
    feedback_msg.feedback.action_type = action_type
    feedback_msg.feedback.status.domain = Status.COMPONENT
    if action_type == 'GOTO':
        feedback_msg.feedback.status.module_code = Status.ROUTE_NAVIGATION
    elif 'ELEVATOR' in action_type:
        feedback_msg.feedback.status.module_code = Status.ELEVATOR_ACTION
    return feedback_msg

def has_timed_out(start_time, timeout):
    '''Returns True if the difference between the current time and the given
    start time is greater than or equal to the given timeout; returns False otherwise.

    Keyword arguments:
    start_time: float -- start time of an operation
    timeout: float -- operation timeout

    '''
    return (rospy.get_time() - start_time) >= timeout

def is_waypoint_achieved(feedback_topic, timeout_s):
    '''Returns True if the feedback message published on the given topic (which is assumed
    to be of type maneuver_navigation.msg.Feedback) indicates navigation success;
    returns False otherwise.

    Keyword arguments:
    feedback_topic: str -- topic on which navigation feedback is published
    timeout_s: float -- timeout (in seconds) to wait for a feedback message

    '''
    feedback_msg = rospy.wait_for_message(feedback_topic,
                                          ManeuverNavigationFeedback,
                                          timeout=timeout_s)
    return feedback_msg and feedback_msg.status == ManeuverNavigationFeedback.SUCCESS

def send_maneuver_nav_goal(goal_pub, frame_id, start_pose, goal_pose, nav_params):
    '''Sends a maneuver navigation goal from "start_pose" to "goal_pose"
    in the given frame using the given goal publisher.

    Keyword arguments:
    goal_pub: rospy.Publisher -- maneuver navigation goal publisher
    frame_id: str -- frame ID of the start and goal poses
    start_pose: geometry_msgs.msg.Pose -- start pose for the navigation
    goal_pose: geometry_msgs.msg.Pose -- goal pose for the navigation
    nav_params: ManeuverNavConfigParams -- configuration parameters for the
                                           navigation component

    '''
    if nav_params is None:
        nav_params = ManeuverNavConfigParams()

    # we remove any leading or trailing / from the frame_id
    frame_id = frame_id.strip('/')

    nav_goal = ManeuverNavigationGoal()
    nav_goal.start.header.frame_id = frame_id
    nav_goal.start.header.stamp = rospy.Time.now()
    nav_goal.start.pose = start_pose

    nav_goal.goal.header.frame_id = frame_id
    nav_goal.goal.header.stamp = rospy.Time.now()
    nav_goal.goal.pose = goal_pose

    nav_goal.conf.append_new_maneuver = nav_params.append_maneuver
    nav_goal.conf.precise_goal = nav_params.precise_goal
    nav_goal.conf.use_line_planner = nav_params.use_line_planner

    goal_pub.publish(nav_goal)

def get_min_angular_diff(angle1, angle2):
    '''Returns the minimum angular difference between angle1 and angle2

    Keyword arguments:
    angle1: float -- first angle in radians
    angle2: float -- second angle in radians

    '''
    return np.arctan2(np.sin(angle1-angle2), np.cos(angle1-angle2))

def get_yaw(quaternion):
    '''Returns the yaw orientation extracted from the input quaternion.

    Keyword arguments:
    quaternion: geometry_msgs.msg.Quaternion

    '''
    euler_orientation = tf.euler_from_quaternion([quaternion.x, quaternion.y,
                                                  quaternion.z, quaternion.w])
    return euler_orientation[2]

def get_quaternion_msg(yaw):
    '''Returns a geometry_msgs.msg.Quaternion message representing the quaternion
    that represents the Euler orientation (0, 0, yaw)

    Keyword arguments:
    yaw: float -- yaw orientation

    '''
    return Quaternion(*tf.quaternion_from_euler(0.0, 0.0, yaw))

def publish_waypoint_array(pose_array_pub, frame_id, areas):
    '''Publishes a geometry_msgs.msg.PoseArray message created from the waypoint list.

    Keyword arguments:
    pose_array_pub: rospy.Publisher -- publisher for geometry_msgs.msg.PoseArray messages
    frame_id: str -- frame ID of the poses
    areas: Sequence[ropod_ros_msgs.Area] -- list of area returned from route_planner

    '''
    waypoint_vis_msg = PoseArray()
    waypoint_vis_msg.header.frame_id = frame_id
    waypoint_vis_msg.header.stamp = rospy.Time.now()
    waypoint_vis_msg.poses = [sub_area.waypoint_pose
                              for area in areas
                              for sub_area in area.sub_areas]
    pose_array_pub.publish(waypoint_vis_msg)
