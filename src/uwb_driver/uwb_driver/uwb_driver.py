# Node to handle parameters from GCS.
#
# Lorenzo Bianchi <lnz.bnc@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# March 9, 2023

# This is free software.
# You can redistribute it and/or modify this file under the
# terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
#
# This file is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this file; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.

import serial, threading, time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from traxxas1_interfaces.msg import Uwb, UwbArray

class DWM1001_API_COMMANDS:
    LEC          = b'lec\n'     # Show measurement and position in CSV format
    RESET        = b'reset\n'   # Reset dev board
    SHELL_MODE   = b'\x0D\x0D'  # Send double ENTER

class UWBDriver(Node):
    serial_is_open = False

    def __init__(self):
        # Initialize ROS2 Node
        super().__init__('uwb_driver')

        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[('port', 'port'),
                        ('id', 0),
                        ('topic', 'topic'),
                       ])

        self.dwm_port = self.get_parameter('port').value
        self.id = self.get_parameter('id').value
        self.topic = self.get_parameter('topic').value

        self.get_logger().info(f'port: {self.dwm_port}')
        self.get_logger().info(f'id: {self.id}')
        self.get_logger().info(f'topic: {self.topic}')

        self.uwb_pub = self.create_publisher(UwbArray, f'{self.topic}', 0)

        try:
            self.serialPortDWM1001 = serial.Serial(
                port = self.dwm_port,
                baudrate = 115200,
                timeout = 2
            )

            self.get_logger().info(f'Opened port {self.serialPortDWM1001.name}')

            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SHELL_MODE)
            time.sleep(1)

            # Send 'lec' command to get distances in CSV format
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            time.sleep(1)

            # Clean output from unnecessary lines
            i = 0
            serialReadLine = b''
            while not serialReadLine.decode().startswith('DIST'):
                i += 1
                # Read from serial port
                serialReadLine = self.serialPortDWM1001.read_until(expected=b'\r\n')

            self.get_logger().info('Starting thread to read anchors distances')
            self.stop_thread = False
            self.serial_thread = threading.Thread(target=self.read_serial)
            self.serial_thread.start()

            self.serial_is_open = True

        except:
            self.get_logger().error(f'Can\'t open port {self.dwm_port}')

    def read_serial(self):
        while not self.stop_thread:
            try:
                msg_array = UwbArray()
                msg_array.header.stamp = self.get_clock().now().to_msg()

                # Read from serial port
                serialReadLineEnc = self.serialPortDWM1001.read_until(expected=b'\r\n')
                serialReadLine = serialReadLineEnc.decode()[:-2]
                splits = serialReadLine.split(',')
                print(splits)

                n_anchors = int(splits[1])
                msg_array.anchor_num = n_anchors
                splits = splits[2:]

                for i in range(n_anchors):
                    msg = Uwb()
                    msg.header = msg_array.header

                    data = splits[i*6 : (i+1)*6]

                    msg.id_str = data[1]
                    msg.x = float(data[2])
                    msg.y = float(data[3])
                    msg.z = float(data[4])
                    msg.dist = float(data[5])

                    msg_array.uwbs.append(msg)

                if len(splits) > 2+n_anchors*6:
                    i = n_anchors*6+1
                    msg_array.x_est = float(splits[i])
                    msg_array.y_est = float(splits[i+1])
                    msg_array.z_est = float(splits[i+2])
                    msg_array.quality_factor = int(float(splits[i+3]))

                self.uwb_pub.publish(msg_array)

            except:
                self.get_logger().error('Error inside thread. Press CTRL+C to quit...')
                #return

def main(args=None):
    rclpy.init(args=args)

    uwb_driver = UWBDriver()
    if uwb_driver.serial_is_open:
        executor = MultiThreadedExecutor(num_threads=1)
        executor.add_node(uwb_driver)
        try:
            executor.spin()
        except:
            executor.shutdown()

            uwb_driver.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            uwb_driver.stop_thread = True
            time.sleep(2)
            uwb_driver.destroy_node()
    else:
        executor.shutdown()
        uwb_driver.destroy_node()


if __name__ == '__main__':
    main()