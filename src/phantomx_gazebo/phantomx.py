import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class PhantomX:
    """Client ROS class for manipulating PhantomX in Gazebo"""

    def __init__(self, ns='/phantomx/'):
        '''
        Initialize the hexapod. Setup namespace, joints, initial conditions, and nodes/topics
        '''
        self.ns = ns
        self.joints = None
        self.angles = None

        self._sub_joints = rospy.Subscriber(
            ns + 'joint_states', JointState, self._cb_joints, queue_size=1)
        rospy.loginfo('Waiting for joints to be populated...')  #the sub-joints subscribes to JointState info in the topic /phantomx/joint_states
        
        while not rospy.is_shutdown():  #Wait until ros shuts down or until the joints are populated
            if self.joints is not None:
                break
            rospy.sleep(0.1)
            rospy.loginfo('Waiting for joints to be populated...')
        rospy.loginfo('Joints populated')

        rospy.loginfo('Creating joint command publishers')
        self._pub_joints = {}   #create joint publishers
        for j in self.joints:   #set each joint publisher. /phantomx/<name of joint>_position_controller/command
            p = rospy.Publisher(
                ns + j + '_position_controller/command', Float64, queue_size=1)
            self._pub_joints[j] = p

        rospy.sleep(1)

        self._pub_cmd_vel = rospy.Publisher(ns + 'cmd_vel', Twist, queue_size=1)    #publisher for the velocity of the whole robot. /phantomx/cmd_vel
        self._pub_lidar_pos = rospy.Publisher(ns + 'hokuyo_tilt_position_controller/command', Float64, queue_size=1)    #publisher for the lidar positions (redundant)

    def set_lidar_theta(self,t):
        '''
        Publish the pitch angle of the lidar.
        '''
        self._pub_lidar_pos.publish(t)
        
    def set_walk_velocity(self, x, y, t):
        '''
        Publish the velocity of the whole robot body.
        Input: x,y and angular yaw velocity
        '''
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = t
        self._pub_cmd_vel.publish(msg)

    def _cb_joints(self, msg):
        '''
        Set each joint angle to the input message angle. If no joints are populated, fill them with the message name.
        Input: Float64 
        '''
        if self.joints is None:
            self.joints = msg.name
        self.angles = msg.position

    def get_angles(self):
        '''
        Get the dictionary of the joint and angle tuple
        Returns: Dictionary of joint angles for the tuple
        '''
        if self.joints is None:
            return None
        if self.angles is None:
            return None
        return dict(zip(self.joints, self.angles))

    def set_angles(self, angles):
        '''
        Publishes the angles to the joints 
        Input: Dictionary of Float64 angles
        '''
        for j, v in angles.items():
            if j not in self.joints:
                rospy.logerror('Invalid joint name "' + j + '"')
                continue
            self._pub_joints[j].publish(v)  #Publish to specific joint with message v from angles

    def set_angles_slow(self, stop_angles, delay=2):
        '''
        Set the angles over the course of the delay duration
        Input: Float64 the ending angles
        '''
        start_angles = self.get_angles()
        start = time.time()
        stop = start + delay    #ending time
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            t = time.time()
            if t > stop:    # stop if you reach the time it takes to stop
                break
            ratio = (t - start) / delay #what ratio between the current time, start tim, and the overall time it should take to complete the task.
            angles = interpolate(stop_angles, start_angles, ratio)  #move to an angle based on time ratio between start/stop angle
            self.set_angles(angles) 
            r.sleep()


def interpolate(anglesa, anglesb, coefa):
    '''
    Inputs: Final Angle, Starting Angle, time ratio between the two
    Returns: Tuple of angles based on time
    '''
    z = {}
    joints = anglesa.keys()
    for j in joints:
        z[j] = anglesa[j] * coefa + anglesb[j] * (1 - coefa)    #if the ratio between start/stop is 1, the angle should be at its final position
    return z


def get_distance(anglesa, anglesb):
    '''
    Get the average distance between the start/stop angle of each joint.
    Return: Float64 average distance
    '''
    d = 0
    joints = anglesa.keys()
    if len(joints) == 0:
        return 0
    for j in joints:
        d += abs(anglesb[j] - anglesa[j])
    d /= len(joints)
    return d
