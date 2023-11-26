import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import math

class ForwardKinematicsOMX(Node):

    def __init__(self):
        super().__init__('forward_kinematics_omx')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.fk_callback,
            10)
        self.subscription 

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
        
        self.get_logger().info('End-effector pose is: %f, %f, %f' % (p_x, p_y, p_z))


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