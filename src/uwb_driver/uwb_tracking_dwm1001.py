import time, serial

class DWM1001_API_COMMANDS:
    LEC          = b'lec\n'     # Show measurement and position in CSV format
    RESET        = b'reset\n'   # Reset dev board
    SHELL_MODE   = b'\x0D\x0D'  # Send double ENTER
    
class UWBDriver(Node):
    def __init__(self) :
        # Initialize ROS2 Node
        super().__init__('uwb_driver')

        self.uwb_pub = self.create_publisher(Uwb, 'uwb_distances', 0)

        # Serial port settings
        self.dwm_port = '/dev/ttyACM1'  #rospy.get_param('~port')
        self.verbose = True             #rospy.get_param('~verbose', True)

        try:
            self.serialPortDWM1001 = serial.Serial(
                port = self.dwm_port,
                baudrate = 115200
            )
        except:
            print(f'Error: {self.dwm_port} can\'t be opened')
            return -1

        # Check if serial port is opened
        if self.serialPortDWM1001.isOpen():
            print(f'Opened port {self.serialPortDWM1001.name}')

            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SHELL_MODE)
            time.sleep(1)

            # Send 'lec' command to get distances in CSV format
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)

            # Clean output from unnecessary lines
            serialReadLine = b''
            while not serialReadLine.decode().startswith('DIST'):
                # Read from serial port
                serialReadLine = self.read_until(terminator='\r\n')

            print('Reading anchors distances\n')
        else:
            print('Can\'t open port {self.serialPortDWM1001.name}')
            raise Exception

    def read_until(self, terminator='\n'):
        lent = len(terminator)
        line = bytearray()
        while True:
            c = self.serialPortDWM1001.read(1)
            if c:
                line += c
                if line[-lent:].decode() == terminator:
                    break
            else:
                break
        return bytes(line)

    def run(self):
        try:
            while True:
                # Read from serial port
                serialReadLineEnc = self.read_until(terminator='\r\n')
                serialReadLine = serialReadLineEnc.decode()[:-2]
                splits = serialReadLine.split(',')
                print(splits)
                n_anchors = int(splits[1])
                splits = splits[2:]
                for i in range(n_anchors):
                    data = splits[i*6 : (i+1)*6]
                    print(f'\t{data}')
        except KeyboardInterrupt:
            print('\nCTRL+C pressed')
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        except:
            print('USB cable disconnected')

if __name__ == '__main__':
    try:
        dwm1001 = DWM1001Localizer()
        dwm1001.run()
    except:
        print('Killing node')