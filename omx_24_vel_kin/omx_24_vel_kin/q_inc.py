import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import Qinc
import numpy as np
from numpy.linalg import pinv
from scipy.optimize import fsolve


class joint_incremental_service(Node):
    def __init__(self):
        super().__init__('q_inc_srv')
        self.srv = self.create_service(Qinc, 'q_inc_service', self.q_inc_callback)
        """
        Qinc:
        request is in form of q1d, q2d, q3d, q4d, q1, q2, q3, q4
        response is in form of q1new, q2new, q3new, q4new
        """

    def q_inc_callback(self, request, response):
        # given q_old and q_dot, returns q_new
        # q_new (4-by-1) = del_t (scalar) * q_dot + q_old
        # default sample time 0.05s
        del_t = 0.05 
        response.q1new = del_t*(request.q1d)+request.q1
        response.q2new = del_t*(request.q2d)+request.q2
        response.q3new = del_t*(request.q3d)+request.q3
        response.q4new = del_t*(request.q4d)+request.q4
        return response

                                                                



def main():
    rclpy.init()

    q_inc_srv = joint_incremental_service()

    rclpy.spin(q_inc_srv)

    rclpy.shutdown()

if __name__ == '__main__':
    main()