import rclpy
from rclpy.node import Node

from open_manipulator_msgs.srv import SetJointPosition, E2Jsrv, Qincsrv
import sys, time

class MoveInStraightLine(Node):
    def __init__(self):
        super().__init__('move_robot_on_y_axis')
        self.client_e2j = self.create_client(E2Jsrv, 'E2J_service')
        while not self.client_e2j.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('E2J service not available, waiting again...')
        #self.send_request_e2j()

        self.client_incremental = self.create_client(Qincsrv, 'q_inc_service')
        while not self.client_incremental.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Qinc service not available, waiting again...')
        # self.send_request_qinc()       
        
        self.client_setposition = self.create_client(SetJointPosition, 'goal_joint_space_path')
        while not self.client_setposition.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Move service not available, waiting again...')
        # self.send_request_move()

    def send_request_e2j(self, vx, vy, vz, wx, wy, wz):
        request = E2Jsrv.Request()
        print('type of response of e2j is ', type(request))
        request.vx = vx
        request.vy = vy
        request.vz = vz
        request.wx = wx
        request.wy = wy
        request.wz = wz
        self.joint_vel = self.client_e2j.call_async(request)
        rclpy.spin_until_future_complete(self, self.joint_vel)
        return self.joint_vel.result()
        # while not self.joint_vel.done():
        #     time.sleep(0.5)
        #     print('Waiting for E2J')
        # return self.joint_vel.result()

    def send_request_qinc(self, q1d, q2d, q3d, q4d):
        request = Qincsrv.Request()
        request.q1d = q1d
        request.q1d = q2d
        request.q1d = q3d
        request.q1d = q4d

        self.new_joints = self.client_incremental.call_async(request)
        rclpy.spin_until_future_complete(self, self.new_joints)
        return self.new_joints.result()
        # while not self.new_joints.done():
        #     time.sleep(0.5)
        #     print('Waiting for qinc')
        # return self.new_joints.result()
        

    def send_request_move(self, q1new, q2new, q3new, q4new):
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        request.joint_position.position = [q1new, q2new, q3new, q4new, 0.0]
        request.path_time = 5.0
        # print(request)
        self.future = self.client_setposition.call_async(request)
        rclpy.spin_until_future_complete(self, self.new_joints)


def main(args=None):
    rclpy.init(args=args)

    move_robot_on_y_axis = MoveInStraightLine()
         # desired e.e. velocities
    vx = 0.0; vy = 10.0; vz = 0.0; wx = 0.0; wy = 0.0; wz = 0.0
    #response_e2j = move_robot_on_y_axis.send_request_e2j(vx, vy, vz, wx, wy, wz)
    print('bs')

    while rclpy.ok():
        
        response_e2j = move_robot_on_y_axis.send_request_e2j(vx, vy, vz, wx, wy, wz)
        response_qinc = move_robot_on_y_axis.send_request_qinc(response_e2j.q1d, response_e2j.q2d, response_e2j.q3d, response_e2j.q4d)
        response_move = move_robot_on_y_axis.send_request_move(response_qinc.q1new, response_qinc.q2new, response_qinc.q3new, response_qinc.q4new)
        
        # if move_robot_on_y_axis.future.done():
        #     try:
        #         response_e2j = move_robot_on_y_axis.send_request_e2j(vx, vy, vz, wx, wy, wz)
        #         response_qinc = move_robot_on_y_axis.send_request_qinc(response_e2j.q1d, response_e2j.q2d, response_e2j.q3d, response_e2j.q4d)
        #         response_move = move_robot_on_y_axis.send_request_move(response_qinc.q1new, response_qinc.q2new, response_qinc.q3new, response_qinc.q4new)

        #     except Exception as e:
        #         move_robot_on_y_axis.get_logger().error('Service call failed %r' % (e,))
        #     break
        # else:
        #     print('yo')
    # i = 0
    # while rclpy.ok() and (i<100):
    #     #rclpy.spin_until_future_complete(move_robot_on_y_axis)
    #     rclpy.spin_once(move_robot_on_y_axis)
    #     # get joint values at the current iteration
    #     #move_robot_on_y_axis.joint_callback
    #     print(move_robot_on_y_axis.q1)

    #     # desired e.e. velocities
    #     vx = 0.0; vy = 1.0; vz = 0.0; wx = 0.0; wy = 0.0; wz = 0.0

    #     #if move_robot_on_y_axis.future.done():
    #     # calculate joint velocities
    #     response = move_robot_on_y_axis.send_request_e2j(vx, vy, vz, wx, wy, wz)
        
    #     # response = move_robot_on_y_axis.joint_vel.result()

    #     print(response)

    #     # calculate joint positions for the next time instance
    #     response = move_robot_on_y_axis.send_request_qinc(response.q1d, response.q2d, response.q3d, response.q4d)
    #     # response = move_robot_on_y_axis.new_joints.result()

    #     # send the new joint positions to the robot
    #     move_robot_on_y_axis.send_request_move(response.q1new, response.q2new, response.q3new, response.q4new)

    #     if move_robot_on_y_axis.future.done():
    #         try:
    #             response = move_robot_on_y_axis.future.result()
    #             move_robot_on_y_axis.get_logger().error('Iteration', i, ' is finished.')
    #         except Exception as e:
    #             move_robot_on_y_axis.get_logger().error('Service call failed %r' % (e,))
    #         break
    #     i += 1

    move_robot_on_y_axis.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()