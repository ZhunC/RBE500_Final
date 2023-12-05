import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import E2Jsrv, J2Esrv
from sensor_msgs.msg import JointState
import numpy as np
from numpy.linalg import pinv
from scipy.optimize import fsolve


class velkin_service(Node):
    def __init__(self):
        super().__init__('velkin_srv')
        self.subscription = self.create_subscription(JointState, 'joint_states', self.joint_callback,10)
        self.E2Jsrv = self.create_service(E2Jsrv, 'E2J_service', self.E2J_callback)
        """ 
        E2J request is in form of: vx, vy, vz, q1, q2, q3, q4
        returns: q1_dot, q2_dot, q3_dot
        """
        self.J2Esrv = self.create_service(J2Esrv, 'J2E_service', self.J2E_callback)
        """ 
        J2E request is in form of: q1dot, q2dot, q3dot, q1, q2, q3, q4
        returns: vx, vy, vj
        remember to update the service types
        """
        # self.subscription = self.create_subscription(JointState,'joint_states', self.jointstate_callback,10)
        self.subscription

    def joint_callback(self, msg):
        self.q1 = msg.position[0]
        self.q2 = msg.position[1]
        self.q3 = msg.position[2]
        self.q4 = msg.position[3]
        print(self.q2)

    def E2J_callback(self, request, response):
        self.get_logger().info('Incoming request for conversion to joint velocities received.')

        # extracting from request
        ee_vel = np.matrix([request.vx, request.vy, request.vz, request.wx, request.wy, request.wz]).T
        # individual transformation matrices
        # data type is np.matrix to allow direct matrix multiplication with operator '*'
        self.H01 = self.DH(0, 0, 36.076, 0)
        self.H12 = self.DH(0, -np.pi/2, 60.25, self.q1) 
        self.H23 = self.DH(128, 0, 0, self.q2 - np.pi/2) 
        self.H34 = self.DH(148, 0, 0, self.q3 + np.pi/2) 
        self.H45 = self.DH(133.4, 0, 0, self.q4) 

        # individual joint matrices
        self.H02 = self.H01*self.H12
        self.H03 = self.H02*self.H23
        self.H04 = self.H03*self.H34
        self.H05 = self.forward_kin(self.q1, self.q2, self.q3, self.q4)
        print('H04 is ', self.H04)
        
        # form jacobian 
        self.J = self.Jacobian()

        # compute joint velocities  
        joint_vel = pinv(self.J)*ee_vel

        print('Jacobian is ',pinv(self.J))

        # assign into responses
        response.q1d = float(joint_vel[1])
        response.q2d = float(joint_vel[2])
        response.q3d = float(joint_vel[3])
        response.q4d = float(joint_vel[4])


        # for i in range(0,3):
        #     for j in range(0,3):
        #         transformation_matrix[i][j] = transformation_array[4*i + j]
        # H05 = self.array_to_matrix(transformation_array)
        # self.H05 = self.transformation_matrix # edit: commented this out and changed to the following line
        # print(self.H05[1][1])
        
        # joint_states = self.Inverse_Kinematics()
        # self.get_logger().info('Finished calculating joint states.')
        # response.q1 = self.q1*180.0/np.pi
        # response.q2 = (self.q2 + np.pi/2)*180.0/np.pi
        # response.q3 = (self.q3 - np.pi/2)*180.0/np.pi
        # response.q4 = self.q4*180.0/np.pi
        print('response is',response)
        return response
    
    def Jacobian(self):
        """
        Given self, which has all values as its attributes
        Find the 6-by-5 Jacobian matrix in the data form of np.matrix 
        """
        # linear parts
        linear1 = np.matrix([0,0,0]).T
        linear2 = np.cross([0,0,1], (self.H05[0:2,3]-self.H01[0:2,3]).T).T
        linear3 = np.cross([-np.sin(self.q1),np.cos(self.q1),0], (self.H05[0:2,3]-self.H02[0:2,3]).T).T
        linear4 = np.cross([-np.sin(self.q1),np.cos(self.q1),0], (self.H05[0:2,3]-self.H03[0:2,3]).T).T
        linear5 = np.cross([-np.sin(self.q1),np.cos(self.q1),0], (self.H05[0:2,3]-self.H04[0:2,3]).T).T
        linear = np.concatenate([linear1, linear2, linear3, linear4, linear5], axis=1)

        angular1 = np.matrix([0, 0, 0]).T
        angular2 = np.matrix([0, 0, 1]).T
        angular3 = np.matrix([-np.sin(self.q1),np.cos(self.q1),0]).T
        angular4 = np.matrix([-np.sin(self.q1),np.cos(self.q1),0]).T
        angular5 = np.matrix([-np.sin(self.q1),np.cos(self.q1),0]).T
        angular = np.concatenate([angular1, angular2, angular3, angular4, angular5], axis=1)

        J = np.concatenate([linear, angular], axis = 0)
        print(J)

        return J
    
    def DH(self, a, alpha, d, theta):
        """
        The Python version of Chris' DH.m
        Given the four DH parameters
        Return the 4-by-4 transformation matrix in the data form of np.matrix 
        """
        result = np.matrix([[np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],\
                            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],\
                            [0,           np.sin(alpha),            np.cos(alpha),             d],\
                            [0,           0,                     0,                      1]])
        return result

    def J2E_callback(self, request, response):
        self.get_logger().info('Incoming request for conversion to ee velocities received.')

        # extracting from request
        qd = np.matrix([0, request.q1d, request.q2d, request.q3d, request.q4d]).T

         # individual transformation matrices
        # data type is np.matrix to allow direct matrix multiplication with operator '*'
        self.H01 = self.DH(0, 0, 36.076, 0)
        self.H12 = self.DH(0, -np.pi/2, 60.25, self.q1) 
        self.H23 = self.DH(128, 0, 0, self.q2 - np.pi/2) 
        self.H34 = self.DH(148, 0, 0, self.q3 + np.pi/2) 
        self.H45 = self.DH(133.4, 0, 0, self.q4) 

        # individual joint matrices
        self.H02 = self.H01*self.H12
        self.H03 = self.H02*self.H23
        self.H04 = self.H03*self.H34
        self.H05 = self.forward_kin(self.q1, self.q2, self.q3, self.q4)
        print('H04 = ',self.H04)
        
        # form jacobian 
        self.J = self.Jacobian()


        # compute joint velocities  
        ee_vel = self.J*qd

        # assign into responses
        response.vx = float(ee_vel[0])
        response.vy = float(ee_vel[1])
        response.vz = float(ee_vel[2])
        response.wx = float(ee_vel[3])
        response.wy = float(ee_vel[4])
        response.wz = float(ee_vel[5])
        print('response is ', response)

        # for i in range(0,3):
        #     for j in range(0,3):
        #         transformation_matrix[i][j] = transformation_array[4*i + j]
        # H05 = self.array_to_matrix(transformation_array)
        # self.H05 = self.transformation_matrix # edit: commented this out and changed to the following line
        # print(self.H05[1][1])
        
        # joint_states = self.Inverse_Kinematics()
        # self.get_logger().info('Finished calculating joint states.')
        # response.q1 = self.q1*180.0/np.pi
        # response.q2 = (self.q2 + np.pi/2)*180.0/np.pi
        # response.q3 = (self.q3 - np.pi/2)*180.0/np.pi
        # response.q4 = self.q4*180.0/np.pi
        return response
    

    
    def forward_kin(self, q1, q2, q3, q4):

        phi = q2 + q3 + q4
        p_x = np.cos(q1)*(133.4*np.cos(phi) + 148*np.cos(q2 + q3) + 128*np.sin(q2))
        p_y = np.sin(q1)*(133.4*np.cos(phi) + 148*np.cos(q2 + q3) + 128*np.sin(q2))
        p_z = 128*np.cos(q2) - 148*np.sin(q2 + q3) - 133.4*np.sin(phi) + 96.3264
        r11 = 0.5*np.cos(phi) + 0.5*np.cos(phi - q1)
        r12 = -0.5*np.sin(phi) - 0.5*np.sin(phi - q1)
        r13 = np.sin(q1)
        r21 = 0.5*np.sin(phi) - 0.5*np.sin(phi - q1)
        r22 = 0.5*np.cos(phi) - 0.5*np.cos(phi - q1)
        r23 = np.cos(q1)
        r31 = -1*np.sin(phi)
        r32 = -1*np.cos(phi)
        r33 = 0

        return np.matrix([[r11, r12, r13, p_x], [r21, r22, r23, p_y], [r31, r32, r33, p_z], [0, 0, 0, 1]])
    

    # functions below this line until main function are unused
    # marked out for simplicity
    # ------------------------------------------------------------------------------------------------------------------------------------



    def pose_to_matrix(self, quaternions, displacements):
        return None
    
    # def array_to_matrix(array):
    #     # converts a 16-element array to a 4-by-4 matrix
    #     matrix = [[],[],[],[]]
    #     for i in range(0, 15):
    #         fraction = int(i / 4) # this is the row number
    #         remainder = i % 4 # this is the column number
    #         matrix[fraction].append(array[i])
    #     return matrix



    def jointstate_callback(self, msg):
        q1 = msg.position[0]
        q2 = msg.position[1]
        q3 = msg.position[2]
        q4 = msg.position[3]
        t_matrix = self.forward_kin(q1, q2, q3, q4)
        self.t_matrix = t_matrix



    def IK(self):
        # This is the core functionality of the ik server.
        # It takes in the desired homogenous transformation matrix H05, which is a list [[e00, e01, e02, e03],[...],[...],[...]]
        # where exx is the element of the matrix in x+1 row and y+1 column
        # and retuns joint states q1, q2, q3, q4, in degrees.
        self.q1 = np.arctan2(-1*self.H05[0][2],self.H05[1][2])
        self.q234 = np.arctan2(-1*self.H05[2][0],-1*self.H05[2][1])
        [self.q2, self.q3] = fsolve(self.q2_q3_solver, [-90*np.pi/180,90*np.pi/180]) # the initial guess for q2 and q3 are both 15 degrees,...
        # because the kinematic chain should be curving down anyways
        self.q4 = self.q234 - self.q2 - self.q3

        return 
        # q_deg = [i*180.0/np.pi for i in [self.q1, self.q2, self.q3, self.q4]]

    def q2_q3_solver(self, x):
        # auxillary function needed to run scipy.optimal.fsolve
        x_desired = self.H05[0][3]
        z_desired = self.H05[2][3]
        y1 = (np.cos(self.q1)*(667*np.cos(self.q234) + 740*np.cos(x[0] + x[1]) + 640*np.cos(x[0])))/5 - x_desired
        y2 = 48163/500 - 148*np.sin(x[0] + x[1]) - 128*np.sin(x[0]) - (667*np.sin(self.q234))/5 - z_desired
        return [y1, y2]
    # def q2_q3_solver(self, x, t1, t234, x_desired, z_desired):
    #     # auxillary function needed to run scipy.optimal.fsolve
    #     y1 = (np.cos(t1)*(667*np.cos(t234) + 740*np.cos(x[1] + x[2]) + 640*np.cos(x[1])))/5 - x_desired
    #     y2 = 48163/500 - 148*np.sin(x[1] + x[2]) - 128*np.sin(x[1]) - (667*np.sin(t234))/5 - z_desired
    #     return [y1, y2

def main():
    rclpy.init()

    VK_srv = velkin_service()

    rclpy.spin(VK_srv)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
