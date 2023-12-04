import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition, E2Jsrv, Qinc
import sys

class MoveInStraightLine(Node):
    def __init__(self):
        super().__init__('move_robot_on_y_axis')
        
        self.subscription = self.create_subscription(JointState, 'joint_states', self.joints_callback,10)

        self.client_e2j = self.create_client(E2Jsrv, 'E2J_service')
        while not self.client_e2j.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('E2J service not available, waiting again...')
        # self.send_request_e2j()

        self.client_incremental = self.create_client(Qinc, 'q_inc_service')
        while not self.client_incremental.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Qinc service not available, waiting again...')
        # self.send_request_qinc()       
        
        self.client_setposition = self.create_client(SetJointPosition, 'goal_joint_space_path')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Move service not available, waiting again...')
        # self.send_request_move()

        self.subscription

    def joint_callback(self, msg):
        self.q1 = msg.position[0]
        self.q2 = msg.position[1]
        self.q3 = msg.position[2]
        self.q4 = msg.position[3]

    def send_request_e2j(self, vx, vy, vz, wx, wy, wz):
        request = E2Jsrv.Request()
        request.vx = vx
        request.vy = vy
        request.vz = vz
        request.wx = wx
        request.wy = wy
        request.wz = wz
        request.q1 = self.q1
        request.q2 = self.q2
        request.q3 = self.q3
        request.q4 = self.q4
        self.joint_vel = self.client_e2j.call_async(request)
        

    def send_request_qinc(self, q1d, q2d, q3d, q4d):
        request = Qinc.Request()
        request.q1d = q1d
        request.q1d = q2d
        request.q1d = q3d
        request.q1d = q4d
        request.q1 = self.q1
        request.q2 = self.q2
        request.q3 = self.q3
        request.q4 = self.q4

        self.new_joints = self.client.call_async(request)
        

    def send_request_move(self, q1new, q2new, q3new, q4new):
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        request.joint_position.position = [q1new, q2new, q3new, q4new, 0.0]
        request.path_time = 5.0

        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    move_robot_on_y_axis = MoveInStraightLine()
    i = 0
    while rclpy.ok() and (i<100):
        #rclpy.spin_once(move_robot_on_y_axis)
        
        # get joint values at the current iteration
        move_robot_on_y_axis.joint_callback()

        # desired e.e. velocities
        vx = 0; vy = 1; vz = 0; wx = 0; wy = 0; wz = 0

        # calculate joint velocities
        move_robot_on_y_axis.send_request_e2j(vx, vy, vz, wx, wy, wz)
        response = move_robot_on_y_axis.joint_vel.result()

        # calculate joint positions for the next time instance
        move_robot_on_y_axis.send_request_qinc(response.q1d, response.q2d, response.q3d, response.q4d)
        response = move_robot_on_y_axis.new_joints.result()

        # send the new joint positions to the robot
        move_robot_on_y_axis.send_request_move(response.q1new, response.q2new, response.q3new, response.q4new)

        if move_robot_on_y_axis.future.done():
            try:
                response = move_robot_on_y_axis.future.result()
                move_robot_on_y_axis.get_logger().error('Iteration', i, ' is finished.')
            except Exception as e:
                move_robot_on_y_axis.get_logger().error('Service call failed %r' % (e,))
            break
        i += 1

    move_robot_on_y_axis.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
