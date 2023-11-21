import rclpy
from std_srvs.srv import SetBool
from rclpy.node import Node
class ManagerClient(Node):
    def __init__(self):
        super().__init__("Controller_manager_client")
        self.client = self.create_client(SetBool, "controller_switch")
    def Switch_callback(self):
        pass
    def switch_controller(self, mode):
        message = SetBool.Request()
        message.data = mode
        print("Waiting for service....")
        self.client.wait_for_service()
        self.future = self.client.call_async(message)
        print("Message sent")
        rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result().message)



def main():
    rclpy.init()
    manager = ManagerClient()
    while True:
        command = input("Command: ")
        if command == "ON":
            manager.switch_controller(True)
        else:
            manager.switch_controller(False)


if __name__ == '__main__':
    main()
