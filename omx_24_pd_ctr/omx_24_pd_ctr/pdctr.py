import rclpy
from rclpy.node import Node

from dynamixel_sdk_custom_interfaces.srv import GetPosition
from dynamixel_sdk_custom_interfaces.msg import SetCurrent
import sys
import time

class pd_control_current(Node):
    def __init__(self):
        super().__init__('pd_control')
        
        self.cli = self.create_client(GetPosition, 'get_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')

        self.publisher = self.create_publisher(SetCurrent, 'set_current', 10)

        self.timer = self.create_timer(0.01, self.signalpub_callback)

        self.current_state = 0
        self.prev_error = 0
        self.client_futures = []


    def signalpub_callback(self, goal = 2450, Kp = 0.13, Kd = 0.09):
        del_t = 0.1


        start_time = time.time()
        request = GetPosition.Request()
        request.id = 14
        
        self.future = self.cli.call_async(request)
        self.client_futures.append(self.future)

        self.error = goal - self.current_state
        
        self.error_dot = (self.error - self.prev_error)/(del_t)

        control = Kp*self.error + Kd*self.error_dot
        self.prev_error = self.error
    
        end_time = time.time()
        del_t = end_time - start_time
        msg = SetCurrent()
        msg.id = 14
        # print('current value is now %f' % control)
        msg.current = int(control)
        self.publisher.publish(msg)


    def spin(self):
            while rclpy.ok():
                rclpy.spin_once(self)
                incomplete_futures = []
                for future in self.client_futures:
                    if future.done():
                        result = future.result()
                        if isinstance(result, GetPosition.Response):
                            self.current_state = result.position
                            print(time.time(),self.current_state)
                    else:
                        incomplete_futures.append(future)
                
                self.client_futures = incomplete_futures

def main():
    rclpy.init()

    pd_control = pd_control_current()

    pd_control.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
