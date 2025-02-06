#!/usr/bin/env python3

import threading
import rospy
import actionlib
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped, Pose
from std_msgs.msg import Empty
from tf import TransformListener
import tf
import math
import rospkg
import csv
from nav_msgs.msg import Odometry

# Path for saving and retrieving the pose.csv file 
output_file_path = rospkg.RosPack().get_path('follow_waypoints') + "/saved_path/pose.csv"
waypoints = []

# Change Pose to the correct frame 
def changePose(waypoint, target_frame):
    if waypoint.header.frame_id == target_frame:
        return waypoint  # Already in correct frame
    
    if not hasattr(changePose, 'listener'):
        changePose.listener = tf.TransformListener()
    
    tmp = PoseStamped()
    tmp.header.frame_id = waypoint.header.frame_id
    tmp.pose = waypoint.pose.pose
    
    try:
        changePose.listener.waitForTransform(
            target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose = changePose.listener.transformPose(target_frame, tmp)
        ret = PoseWithCovarianceStamped()
        ret.header.frame_id = target_frame
        ret.pose.pose = pose.pose
        return ret
    except tf.Exception:
        rospy.logwarn(f"Can't transform pose to {target_frame} frame")
        return None

class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id', 'map')
        self.odom_topic = rospy.get_param('~odom_topic', '/lego_loam/odom')
        self.distance_tolerance = rospy.get_param('~waypoint_distance_tolerance', 0.1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        self.current_pose = Pose()
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def execute(self, userdata):
        global waypoints
        
        for waypoint in waypoints:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose = waypoint.pose.pose
            
            rospy.loginfo(f'Executing move_base goal to: {waypoint.pose.pose.position.x}, {waypoint.pose.pose.position.y}')
            self.client.send_goal(goal)
            
            distance = float('inf')
            rate = rospy.Rate(10)
            timeout = rospy.Time.now() + rospy.Duration(30)
            
            while distance > self.distance_tolerance and not rospy.is_shutdown():
                distance = math.sqrt(pow(waypoint.pose.pose.position.x - self.current_pose.position.x, 2) +
                                     pow(waypoint.pose.pose.position.y - self.current_pose.position.y, 2))
                if rospy.Time.now() > timeout:
                    rospy.logwarn("Timeout reached while trying to reach waypoint.")
                    break
                rate.sleep()
        return 'success'

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id', 'map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses

class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
        self.poseArray_publisher = rospy.Publisher('/waypoints', PoseArray, queue_size=1)
        threading.Thread(target=self.wait_for_path_reset, daemon=True).start()

    def initialize_path_queue(self):
        global waypoints
        waypoints = []
        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
    
    def wait_for_path_reset(self):
        global waypoints
        while not rospy.is_shutdown():
            rospy.wait_for_message('/path_reset', Empty)
            rospy.loginfo('Received path RESET message')
            self.initialize_path_queue()
            rospy.Rate(1).sleep()
    
    def execute(self, userdata):
        global waypoints
        self.initialize_path_queue()
        rospy.loginfo("Waiting for waypoints...")
        while not rospy.is_shutdown():
            try:
                pose = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped, timeout=1)
                rospy.loginfo("Received new waypoint")
                transformed_pose = changePose(pose, "map")
                if transformed_pose:
                    waypoints.append(transformed_pose)
                    self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
            except rospy.ROSException:
                continue
        return 'success'

class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
    
    def execute(self, userdata):
        rospy.loginfo('##### REACHED FINISH GATE #####')
        return 'success'

def main():
    rospy.init_node('follow_waypoints')
    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('GET_PATH', GetPath(), transitions={'success': 'FOLLOW_PATH'}, remapping={'waypoints': 'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(), transitions={'success': 'PATH_COMPLETE'}, remapping={'waypoints': 'waypoints'})
        StateMachine.add('PATH_COMPLETE', PathComplete(), transitions={'success': 'GET_PATH'})
    sm.execute()

if __name__ == '__main__':
    main()
