import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import Qincsrv
from sensor_msgs.msg import JointState
import numpy as np
from numpy.linalg import pinv
from scipy.optimize import fsolve


class joint_incremental_service(Node):
    def __init__(self):
        super().__init__('q_inc_srv')
        self.subscription = self.create_subscription(JointState, 'joint_states', self.joint_callback,10)
        self.srv = self.create_service(Qincsrv, 'q_inc_service', self.q_inc_callback)
        """
        Qinc:
        request is in form of q1d, q2d, q3d, q4d, q1, q2, q3, q4
        response is in form of q1new, q2new, q3new, q4new
        """
        self.subscription

    def joint_callback(self, msg):
        self.q1 = msg.position[0]
        self.q2 = msg.position[1]
        self.q3 = msg.position[2]
        self.q4 = msg.position[3]
        print(self.q2)

    def q_inc_callback(self, request, response):
        # given q_old and q_dot, returns q_new
        # q_new (4-by-1) = del_t (scalar) * q_dot + q_old
        # default sample time 0.05s
        del_t = 0.25
        print(request)
        response.q1new = del_t*(request.q1d) + self.q1
        response.q2new = del_t*(request.q2d) + self.q2
        response.q3new = del_t*(request.q3d) + self.q3
        response.q4new = del_t*(request.q4d) + self.q4
        print('new q4 is: ', response.q4new)
        self.q1 = response.q1new
        self.q2 = response.q2new
        self.q3 = response.q3new
        self.q4 = response.q4new
        return response

                                                           
def main():
    rclpy.init()

    q_inc_srv = joint_incremental_service()

    rclpy.spin(q_inc_srv)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
