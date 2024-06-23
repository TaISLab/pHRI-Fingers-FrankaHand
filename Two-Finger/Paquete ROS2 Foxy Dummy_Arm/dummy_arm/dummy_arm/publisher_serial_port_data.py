import rclpy
import serial
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

# SerialPort = serial.Serial('/dev/ttyACM0', 115200)

class SerialPublisher(Node):

    def __init__(self):
        super().__init__('serial_publisher')
        self.publisher_ = self.create_publisher(String, 'DummyArmReadings_String', 10)
        self.publisherf_ = self.create_publisher(Float32MultiArray, 'DummyArmReadings_FloatArray', 10)
        self.publisheracf_ = self.create_publisher(Float32MultiArray, 'DummyArmReadings_AccFloatArray', 10)
        self.timer = self.create_timer(0.008, self.publish_serial_data)
        self.serial_port = serial.Serial('/dev/ttyACM1', 115200)
        self.errcount = 0

    def publish_serial_data(self):
        msg = String()
        jointmsg = Float32MultiArray()
        accmsg = Float32MultiArray()
        
        try:
            jointStringArray = self.serial_port.readline().decode("utf-8").split(",     ", 1)[1].split(",     ")
            # x = np.array(jointStringArray)
            # y = x.astype(np.float32)
            # print(x.astype(np.float32))
            # print(type(x.astype(np.float32)))
            # jointmsg.data = x.astype(np.float32)
            # jointmsg.data = [ 2.09, 4.5, 3.5, 23.5, 98.34, -89.0]
            jointmsg.data = [
                float(jointStringArray[0]),
                float(jointStringArray[1]),
                float(jointStringArray[2]),
                float(jointStringArray[3]),
                float(jointStringArray[4]),
                float(jointStringArray[5])
            ]
            accmsg.data = [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            ]
            # accmsg.data = self.serial_port.readline().decode("utf-8").split(",     ", 1)[1].split(",     ")
            msg.data = '%s' % self.serial_port.readline().decode("utf-8").split(",     ", 1)[1].split("\r\n")[0]
            # print(msg.data)
            self.publisher_.publish(msg)
            self.publisherf_.publish(jointmsg)
            self.publisheracf_.publish(accmsg)
        except:
            print("...")
        # self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    publisher = SerialPublisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
