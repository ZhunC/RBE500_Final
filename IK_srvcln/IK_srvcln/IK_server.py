import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import IK_srv
import sys
import numpy as np
from scipy.optimal import fsolve
from std_msgs import float32MultiArray


class IK_service(Node):
    def __init__(self):
        super().__init__('IK_srv')
        self.srv = self.create_service(IK_srv, 'IK_service', self.IK_callback)

    def IK_callback(self, request, response):
        self.get_logger().info('Incoming request received.')
        H05 = self.array_to_matrix(request.Transformation_Matrix)
        joint_states = self.Inverse_Kinematics(H05)
        self.get_logger().info('Finished calculating joint states.')
        response.Joint_States = joint_states
        return response
    
    def array_to_matrix(array):
        # converts a 16-element array to a 4-by-4 matrix
        matrix = [[],[],[],[]]
        for i in range(0, 15):
            fraction = int(i / 4) # this is the row number
            remainder = i % 4 # this is the column number
            matrix[fraction].append(array[i])
        return matrix

    def Inverse_Kinematics(self, H05):
        # This is the core functionality of the ik server.
        # It takes in the desired homogenous transformation matrix H05, which is a list [[e00, e01, e02, e03],[...],[...],[...]]
        # where exx is the element of the matrix in x+1 row and y+1 column
        # and retuns joint states q1, q2, q3, q4, in degrees.
        q1 = np.atan2(-1*H05[0][2],H05[1][2])
        q234 = np.atan2(-1*H05[2][1],-1*H05[2][1])
        [q2, q3] = fsolve(self.q2_q3_solver, [15*np.pi/180,15*np.pi/180]) # the initial guess for q2 and q3 are both 15 degrees,...
        # because the kinematic chain should be curving down anyways
        q4 = q234 - q2 - q3
        q_deg = [i*180.0/np.pi for i in [q1, q2, q3, q4]]
        return q_deg

    def q2_q3_solver(x, t1, t234, x_desired, z_desired):
        # auxillary function needed to run scipy.optimal.fsolve
        y1 = (np.cos(t1)*(667*np.cos(t234) + 740*np.cos(x[1] + x[2]) + 640*np.cos(x[1])))/5 - x_desired
        y2 = 48163/500 - 148*np.sin(x[1] + x[2]) - 128*np.sin(x[1]) - (667*np.sin(t234))/5 - z_desired
        return [y1,y2]
                                                                



def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

