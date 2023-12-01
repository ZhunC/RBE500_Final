import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import E2Jsrv, J2Esrv
import numpy
from numpy.linalg import pinv


class velkin_service(Node):
    def __init__(self):
        super().__init__('velkin_srv')
        self.E2Jsrv = self.create_service(E2Jsrv, 'E2J_service', self.E2J_callback)
        self.J2Esrv = self.create_service(J2Esrv, 'J2E_service', self.J2E_callback)
        self.subscription = self.create_subscription(JointState,'joint_states', self.jointstate_callback,10)

    def E2J_callback(self, request, response):
        self.get_logger().info('Incoming request for conversion to joint velocities received.')
        ee_vel = [request.vx, request.vy, request.vz]
        J = Jacobian(ee_vel)

        # for i in range(0,3):
        #     for j in range(0,3):
        #         transformation_matrix[i][j] = transformation_array[4*i + j]
        # H05 = self.array_to_matrix(transformation_array)
        self.H05 = transformation_matrix
        print(self.H05[1][1])
        
        joint_states = self.Inverse_Kinematics()
        self.get_logger().info('Finished calculating joint states.')
        response.q1 = self.q1*180.0/np.pi
        response.q2 = (self.q2 + np.pi/2)*180.0/np.pi
        response.q3 = (self.q3 - np.pi/2)*180.0/np.pi
        response.q4 = self.q4*180.0/np.pi
        return response
    
    # def array_to_matrix(array):
    #     # converts a 16-element array to a 4-by-4 matrix
    #     matrix = [[],[],[],[]]
    #     for i in range(0, 15):
    #         fraction = int(i / 4) # this is the row number
    #         remainder = i % 4 # this is the column number
    #         matrix[fraction].append(array[i])
    #     return matrix

    def J2E_callback(self, request, response):

    def jointstate_callback(self, msg):

        q1 = msg.position[0]
        q2 = msg.position[1]
        q3 = msg.position[2]
        q4 = msg.position[3]
        t_matrix = self.forward_kin(q1, q2, q3, q4)
        self.t_matrix = t_matrix

    def forward_kin(self, q1, q2, q3, q4):

        phi = q2 + q3 + q4
        p_x = numpy.cos(q1)*(133.4*numpy.cos(phi) + 148*numpy.cos(q2 + q3) + 128*numpy.sin(q2))
        p_y = numpy.sin(q1)*(133.4*numpy.cos(phi) + 148*numpy.cos(q2 + q3) + 128*numpy.sin(q2))
        p_z = 128*numpy.cos(q2) - 148*numpy.sin(q2 + q3) - 133.4*numpy.sin(phi) + 96.3264
        r11 = 0.5*numpy.cos(phi) + 0.5*numpy.cos(phi - q1)
        r12 = -0.5*numpy.sin(phi) - 0.5*numpy.sin(phi - q1)
        r13 = numpy.sin(q1)
        r21 = 0.5*numpy.sin(phi) - 0.5*numpy.sin(phi - q1)
        r22 = 0.5*numpy.cos(phi) - 0.5*numpy.cos(phi - q1)
        r23 = numpy.cos(q1)
        r31 = -1*numpy.sin(phi)
        r32 = -1*numpy.cos(phi)
        r33 = 0

        return numpy.array([[r11, r12, r13, p_x], [r21, r22, r23, p_y], [r31, r32, r33, p_z], [0, 0, 0, 1]])


    def Jacobian(self):
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
    #     return [y1, y2]
                                                                



def main():
    rclpy.init()

    IK_srv = IK_service()

    rclpy.spin(IK_srv)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
