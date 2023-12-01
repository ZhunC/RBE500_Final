import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math
from geometry_msgs.msg import Pose

class ForwardKinematicsOMX(Node):

    def __init__(self):
        super().__init__('forward_kinematics_omx')
        self.publisher = self.create_publisher(
            Pose, 
            'position_fksub', 
            10)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.fk_callback,
            10)
        self.subscription 
        self.publisher

    def fk_callback(self, msg):
        
        q1 = msg.position[0]
        q2 = msg.position[1]
        q3 = msg.position[2]
        q4 = msg.position[3]
        qg = msg.position[4]

        phi = q2 + q3 + q4

        p_x = math.cos(q1)*(133.4*math.cos(phi) + 148*math.cos(q2 + q3) + 128*math.sin(q2))
        p_y = math.sin(q1)*(133.4*math.cos(phi) + 148*math.cos(q2 + q3) + 128*math.sin(q2))
        p_z = 128*math.cos(q2) - 148*math.sin(q2 + q3) - 133.4*math.sin(phi) + 96.3264
        r11 = 0.5*math.cos(phi) + 0.5*math.cos(phi - q1)
        r12 = -0.5*math.sin(phi) - 0.5*math.sin(phi - q1)
        r13 = math.sin(q1)
        r21 = 0.5*math.sin(phi) - 0.5*math.sin(phi - q1)
        r22 = 0.5*math.cos(phi) - 0.5*math.cos(phi - q1)
        r23 = math.cos(q1)
        r31 = -1*math.sin(phi)
        r32 = -1*math.cos(phi)
        r33 = 0

        w = 0.5*(1 + r11 + r22 + r33)**0.5
        x = (r32 - r23)/(4*w)
        y = (r13 - r31)/(4*w)
        z = (r21 - r12)/(4*w)


        p = [p_x, p_y, p_z]
        # self.msg = /* msg_type */()
        # self.get_logger().info('Publishing message')
        # self./* pub_name */.publish(msg)
        self.get_logger().info('End-effector position is: %f, %f, %f \nEnd-effector orientation is: \n|%f, %f, %f|\n|%f, %f, %f|\n|%f, %f, %f|' % (p_x, p_y, p_z, r11, r12, r13, r21, r22, r23, r31, r32, r33))
        msg = Pose()
        msg.position.x = p_x
        msg.position.y = p_y
        msg.position.z = p_z
        msg.orientation.w = w
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        # pose.position = [p_x, p_y, p_z]
        # pose.orientation = [w, x, y, z]


        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    forward_kinematics_omx = ForwardKinematicsOMX()

    rclpy.spin(forward_kinematics_omx)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    forward_kinematics_omx.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()