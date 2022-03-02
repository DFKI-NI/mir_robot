import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MirRestAPIClient(Node):

    def __init__(self):
        super().__init__('mir_restapi_client_sync_time')
        self.create_api_clients()

        self.restAPI_setTime.wait_for_service()

        self.sync_time()
        rclpy.shutdown()
        exit()

    def create_api_clients(self):
        self.restAPI_setTime = self.create_client(
            Trigger,
            'mir_100_sync_time')

        self.restAPI_getStatus = self.create_client(
            Trigger,
            'mir_100_get_status')

    def call_trigger_service(self, client):
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60)
        if not future.done():
            self.get_logger().error("timeout")
        else:
            # service done
            self.get_logger().info("service executed")
            res = future.result()
            if res.success:
                self.get_logger().info(res.message)
            else:
                self.get_logger().error(res.message)

    def sync_time(self):
        self.call_trigger_service(self.restAPI_setTime)

    def get_status(self):
        self.call_trigger_service(self.restAPI_getStatus)


def main(args=None):
    rclpy.init(args=args)
    MirRestAPIClient()


if __name__ == '__main__':
    main()
