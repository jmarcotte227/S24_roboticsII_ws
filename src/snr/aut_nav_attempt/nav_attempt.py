## This code is basically just the tracking node that will stop at nothing to find his blu (or thats the plan anyways)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np
import math
import pickle
from sensor_msgs.msg import scan_data 

## Functions for quaternion and rotation matrix conversion
## The code is adapted from the general_robotics_toolbox package
## Code reference: https://github.com/rpiRobotics/rpi_general_robotics_toolbox_py
def hat(k):
    """
    Returns a 3 x 3 cross product matrix for a 3 x 1 vector

             [  0 -k3  k2]
     khat =  [ k3   0 -k1]
             [-k2  k1   0]

    :type    k: numpy.array
    :param   k: 3 x 1 vector
    :rtype:  numpy.array
    :return: the 3 x 3 cross product matrix
    """

    khat=np.zeros((3,3))
    khat[0,1]=-k[2]
    khat[0,2]=k[1]
    khat[1,0]=k[2]
    khat[1,2]=-k[0]
    khat[2,0]=-k[1]
    khat[2,1]=k[0]
    return khat

def q2R(q):
    """
    Converts a quaternion into a 3 x 3 rotation matrix according to the
    Euler-Rodrigues formula.
    
    :type    q: numpy.array
    :param   q: 4 x 1 vector representation of a quaternion q = [q0;qv]
    :rtype:  numpy.array
    :return: the 3x3 rotation matrix    
    """
    
    I = np.identity(3)
    qhat = hat(q[1:4])
    qhat2 = qhat.dot(qhat)
    return I + 2*q[0]*qhat + 2*qhat2
######################

def euler_from_quaternion(q):
    w=q[0]
    x=q[1]
    y=q[2]
    z=q[3]
    # euler from quaternion
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    return [roll,pitch,yaw]

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        self.get_logger().info('Tracking Node Started')
        
        # Current object pose
        self.obj_pose = None
        
        # ROS parameters
        self.declare_parameter('world_frame_id', 'odom')

        # Create a transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher for the control command
        self.pub_control_cmd = self.create_publisher(Twist, '/track_cmd_vel', 10)
        # Create a subscriber to the detected object pose
        self.sub_detected_obj_pose = self.create_subscription(PoseStamped, '/detected_color_object_pose', self.detected_obj_pose_callback, 10)
    
        # Create timer, running at 100Hz
        self.timer = self.create_timer(0.01, self.timer_update)

        # Create variable to store pose error
        self.pose_x_error = []
        self.pose_y_error = []

        self.counter = 0
    
    def detected_obj_pose_callback(self, msg):
        #self.get_logger().info('Received Detected Object Pose')
        
        odom_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        center_points = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        # TODO: Filtering
        # You can decide to filter the detected object pose here
        # For example, you can filter the pose based on the distance from the camera
        # or the height of the object

        # uncommented the next two lines, not sure what else to do here
        if np.linalg.norm(center_points) > 3 or center_points[2] > 0.7:
            return
        
        try:
            # Transform the center point from the camera frame to the world frame
            transform = self.tf_buffer.lookup_transform(odom_id,msg.header.frame_id,rclpy.time.Time(),rclpy.duration.Duration(seconds=0.1))
            t_R = q2R(np.array([transform.transform.rotation.w,transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z]))
            cp_world = t_R@center_points+np.array([transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z])
        except TransformException as e:
            self.get_logger().error('Transform Error: {}'.format(e))
            return
        
        # Get the detected object pose in the world frame
        self.obj_pose = cp_world
        
    def get_current_object_pose(self):
        
        odom_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        # Get the current robot pose
        try:
            # from base_footprint to odom
            transform = self.tf_buffer.lookup_transform('base_footprint', odom_id, rclpy.time.Time())
            robot_world_x = transform.transform.translation.x
            robot_world_y = transform.transform.translation.y
            robot_world_z = transform.transform.translation.z
            robot_world_R = q2R([transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z])
            #object_pose = robot_world_R@self.cp_pose+np.array([robot_world_x,robot_world_y,robot_world_z])
            # Changed variable name in above line, duplicated below
            object_pose = robot_world_R@self.obj_pose+np.array([robot_world_x,robot_world_y,robot_world_z])
        except TransformException as e:
            self.get_logger().error('Transform error: ' + str(e))
            return
        
        return object_pose
    ##from yahboom
    def registerScan(self, scan_data):
    if not isinstance(scan_data, LaserScan): return
    ranges = np.array(scan_data.ranges)
   # self.Right_warning = 0
    #self.Left_warning = 0
    #self.front_warning = 0

   # for i in range(len(ranges)):
   # angle = (scan_data.angle_min + scan_data.angle_increment) * RAD2DEG
        #The angle of radar information is a radian system, and here it is converted into an angle for calculation
        #if 160 > angle > 180 - self.LaserAngle:#The angle is based on the structure of the radar to set the judgment range
         #   if ranges[i] < self.ResponseDist*1.5: 
        #range[i] is the result of radar scanning, which in this case refers to distance information
          #      self.Right_warning += 1
        #if - 160 < angle < self.LaserAngle - 180:
         #   if ranges[i] < self.ResponseDist*1.5:
          #      self.Left_warning += 1
       # if abs(angle) > 160:
        #    if ranges[i] <= self.ResponseDist*1.5:
         #       self.front_warning += 1
        #if self.Joy_active or self.Switch == True:
         #   if self.Moving == True:
          #      self.pub_vel.publish(Twist())
           #     self.Moving = not self.Moving
            #        return
        #self.Moving = True

    def timer_update(self, ranges):
        ################### Write your code here ###################
        
        # Now, the robot stops if the object is not detected
        # But, you may want to think about what to do in this case
        # and update the command velocity accordingly
       
        if self.obj_pose is None:
           return 
        try:
            cmd_vel = Twist()
            cmd_vel.linear.x = 1.0
            cmd_vel.angular.z = 0.0
            self.pub_control_cmd.publish(cmd_vel)
        except ranges[539] < 0.5:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 1.0
            self.pub_control_cmd.publish(cmd_vel)
            return
        
        # Get the current object pose in the robot base_footprint frame
        # Changed to make this pose relative to robot 
        current_object_pose = self.get_current_object_pose()
        
        # TODO: get the control velocity command
        cmd_vel = self.controller(current_object_pose)
        
        # publish the control command
        self.pub_control_cmd.publish(cmd_vel)
        #################################################
    

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    tracking_node = TrackingNode()
    rclpy.spin(tracking_node)
    # Destroy the node explicitly
    tracking_node.destroy_node()
    # Shutdown the ROS client library for Python
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()