import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys
import numpy as np
from scipy.optimal import fsolve


class IK_service(Node):
    def __init__(self):
        super().__init__('IK_service')
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')
        self.send_request()

    def send_request(self):
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        request.joint_position.position = [0.0, 0.0, 0.0, 0.0, 0.0]
        request.path_time = 5.0

        self.future = self.client.call_async(request)
    
     
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
                                                                




def main(args=None):
    rclpy.init(args=args)

    basic_robot_control = BasicRobotControl()

    while rclpy.ok():
        rclpy.spin_once(basic_robot_control)
        if basic_robot_control.future.done():
            try:
                response = basic_robot_control.future.result()
            except Exception as e:
                basic_robot_control.get_logger().error('Service call failed %r' % (e,))
            break

    basic_robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

