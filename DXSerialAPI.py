import serial
from utils import *
import warnings

class DXSerialAPI(serial.Serial):
    """
    This class implements a serial API for the communication with DX motors (Dynamixel)
    This API is especially made for the DX-117 but can be easily adapted for any Dynamixel motors, the communication protocol is standart
    This is a simple slave master protocol
    Please be careful an read the documentation of the devices you want to use, you can do great arm to your motors if you send random instruction without being cautious
    documentation : http://www.besttechnology.co.jp/modules/knowledge/?Dynamixel%E9%80%9A%E4%BF%A1%E3%83%97%E3%83%AD%E3%83%88%E3%82%B3%E3%83%AB
    """
    
    def __init__(self, PORT_NAME, BAUDRATE, TIME_OUT=0.001, verbose_messages=False):
        """
        :param PORT_NAME: name of the port you want to open
        :param BAUDRATE: (int) Baud rate such as 9600 or 115200 etc.
        :param TIME_OUT: (float) Set a read timeout value.
        """

        # __init__ of the parent class
        super(DXSerialAPI, self).__init__(PORT_NAME, BAUDRATE, timeout=TIME_OUT)

        # basic instruction set definition
        self.INSTRUCTION_SET = {'PING' : 0x01, 'READ_DATA' : 0x02, 'WRITE_DATA' : 0x03, 'REG_WRITE' : 0x04, 'ACTION' : 0x05, 'RESET' : 0x06, 'SYNC_WRITE' : 0x83}
        self.INSTRUCTION_SET_reversed = {'0x01' : 'PING', '0x02' : 'READ_DATA', '0x03' : 'WRITE_DATA', '0x04' : 'REG_WRITE', '0x05' : 'ACTION', '0x06' : 'RESET', '0x83' : 'SYNC_WRITE'}


        # constants of the motors
        self.ANGLE_UNIT = 0.29  # conversion unit between bytes and degrees
        self.SPEED_UNIT = 0.111  # conversion unit between bytes and rpm
        self.TORQUE_UNIT = 1 / 1023 * 100  # conversion unit between bytes and torque (in %)
        self.VOLTAGE_UNIT = 0.1 # convertion unit between bytes and volt

        #options
        self.verbose_messages = verbose_messages # if set to True all messages sended will also be printed

    # ------------- HELPERS -------------
    def _send_message(self, device_id, instruction, *args):
        """
        The general message write, please use specific functions to send messages
        Send a message following the DX motors protocol: [0xFF 0xFF ID LENGTH INSTRUCTION PARAMETER 1 ... PARAMETER N CHECKSUM]
        checksum = ~ (ID + LENGTH + INSTRUCTION + PARAMETER 1 + ... + PARAMETER N) If the calculated value is 255 or more, the lower 8 bits become the value of CHECKSUM.
        :param device_id: the id of the motor you want to control
        :param instruction: from the instruction set
        :param kwargs: the other instructions, following the documentation of dynamixel
        """
        length = len(args) + 2 # length of the message
        message = [0xFF, 0xFF, device_id, length, instruction]
        checksum = device_id + length + instruction

        for arg in args:
            message.append(arg)
            checksum += arg

        checksum = ~checksum % 256 # we take only the lower 8 bits
        message.append(checksum)

        message = bytearray(message)

        self.write(message)

        if self.verbose_messages:
            # message base 10
            print('message b10 :', end=' ')
            for x in message: print(x, end='\\')
            # message readable
            print('\nmessage --- :', end=' ')
            print('0xFF\\0xFF\\id:{}\\len:{}\\{}'.format(device_id, length, self.INSTRUCTION_SET_reversed['0x{:02x}'.format(instruction)]), end='\\')
            for x in args: print(x, end='\\')
            print('checksum:{}'.format(checksum))
            # message hexadecimal
            print('message b16 :', message, '\n')

        return 0

    def _print_message(self, device_id, instruction, *args):
        """
        Will do the same that _send_message but only print the message in the console instead of sending it
        """
        print(args)
        length = len(args) + 2  # length of the message
        message = [0xFF, 0xFF, device_id, length, instruction]
        checksum = device_id + length + instruction

        for arg in args:
            message.append(arg)
            checksum += arg

        checksum = ~checksum % 256  # we take only the lower 8 bits
        message.append(checksum)

        message = bytearray(message)
        print('message b10 :', end=' ')
        for x in message: print(x, end= '\\')
        print('\nmessage b16 :',message, '\n')
        return 0

    def _receive_message(self):
        """
        The function receive the status packet informations that some the devive will returns to the host as a response after receiving the instruction packet
        The packet structure is as follows : 0xFF 0xFF ID LENGTH ERROR PARAMETER 1 ... PARAMETER N CHECKSUM
        0xFF 0xFF : The 2 bytes located at the head means the beginning (header) of the packet, and 2 bytes are fixed to 0xFF in hexadecimal.
        ID : This is the ID of Dynamixel that returns the packet.
        LENGTH : It returns "Parameter number (N) + 2".
        ERROR : Indicates the state of the alarm that occurred during device operation. If there is a corresponding alarm, that bit becomes 1.
        PARAMETER 1 to N : Return additional information according to the instruction.

        Please read the dynamixel documentation for more details
        :return:
        """

        x = self.read_until(terminator=b'\xff\xff')                             # the begining of the message
        id = self.read()                                                        # the id of the device
        length = self.read()                                                    # the number of optional parameters N + 2
        error = self.read()                                                     # error ?
        parameters = self.read(size=int.from_bytes(length,byteorder='big') -2 ) # optional parameters
        checksum = int.from_bytes(self.read(),byteorder='big')                  # checksum


        # error test
        if id != b'' and error != b'\x00' and error != b'':
            warnings.warn('The motor {} has responded the error number {}'.format(id,error))

        return id, length, error, parameters, checksum

    # -----------------------------------

    # ---------- INSTRUCTIONS -----------
    def _PING(self, device_id):
        """
        It is used to confirm the existence of Dynamixel having a specific ID in the node
        :param device_id: id of the motor you want
        :return True if the device exists or False if not, return also the message
        """
        self._send_message(device_id, self.INSTRUCTION_SET['PING'])
        message = self._receive_message()
        death = True
        for x in message:
            if x != b'': death = False

        return not(death), message

    def _READ_DATA(self, device_id, *args):
        """
        Read from control table
        :param device_id: id of the motor you want
        :param args1: Start address of data to be read
        :param args2: Length of data to be read
        :return response: the response of the device interogated
        """
        if len(args) != 2:
            raise ValueError('READ_DATA() takes 2 optional arguments but {} were given'.format(len(args)))
            return 1
        else:
            self._send_message(device_id, self.INSTRUCTION_SET['READ_DATA'], *args)
            #self._print_message(device_id, self.INSTRUCTION_SET['READ_DATA'], *args)
            id, length, error, parameters, checksum = self._receive_message()
            return id, length, error, parameters, checksum

    def _WRITE_DATA(self, device_id, *args):
        """
        Write to the control table (immediately reflected)
        :param device_id: id of the motor you want
        :param args1: Start address of data to be read
        :param args2: First data to be written
        :param argsN+1: Nth data to be written
        """
        if len(args) < 2:
            raise ValueError('WRITE_DATA() takes at least 2 optional arguments but {} were given'.format(len(args)))
            return 1
        else:
            self._send_message(device_id, self.INSTRUCTION_SET['WRITE_DATA'], *args)
            return 0

    def _REG_WRITE(self, device_id, *args):
        """
        Write (hold) to control table
        :param device_id: id of the motor you want
        :param args1: Start address of data to be read
        :param args2: First data to be written
        :param argsN+1: Nth data to be written
        """
        if len(args) < 2:
            raise ValueError('REG_WRITE() takes at least 2 optional arguments but {} were given'.format(len(args)))
            return 1
        else:
            self._send_message(device_id, self.INSTRUCTION_SET['REG_WRITE'], *args)
            return 0

    def _ACTION(self, device_id):
        """
        Execute the instruction stored in REG_WRITE.
        :param device_id: id of the motor you want
        no extra parameters
        """
        self._send_message(device_id, self.INSTRUCTION_SET['ACTION'])
        return 0

    def _RESET(self, device_id):
        """
        Reset the value of the control table of the device to the factory default value.
        :param device_id: id of the motor you want
        no extra parameters
        """
        self._send_message(device_id, self.INSTRUCTION_SET['RESET'])
        return 0

    def _SYNC_WRITE(self, *args):
        """
        Batch writing of multiple devices to the same control table (immediately reflected)
        :param args1: Start address of data to be written
        :param args2: Length of data to be written
        :param args3: ID of the first Dynamixel
        :param args4: First data to be written by the first Dynamixel
        :param args5: Second data to be written by the first Dynamixel

        :param args L + 3: The Lth data to be written by the first Dynamixel
        :param args L + 4: ID of the second Dynamixel
        :param args L + 5: First data to be written by the second Dynamixel
        :param args L + 6: Second data to be written by the second Dynamixel
        .
        :param args2 L + 4: L-th data to be written by the second Dynamixel
        """
        if len(args) < 4:
            raise ValueError('SYNC_WRITE() takes at least 4 optional arguments but {} were given'.format(len(args)))
            return 1
        else:
            self._send_message(0xFE, self.INSTRUCTION_SET['SYNC_WRITE'], *args)
            return 0
    # -----------------------------------

    # ------------- READERS -------------
    def read_moving_speed(self, id):
        """
        This function read the moving speed of the motor (id)
        :param id: id of the motor
        :return moving_speed: the moving speed read in the motor in rpm
        """
        raw_response = self._READ_DATA(id, 0x20, 2)
        return round(int.from_bytes(raw_response[3], byteorder='little') * self.SPEED_UNIT, 0)

    def read_moving_speed_byte(self, id):
        """
        This function read the moving speed of the motor (id)
        :param id: id of the motor
        :return moving_speed: the moving speed read in the motor
        """
        raw_response = self._READ_DATA(id, 0x20, 2)
        return int.from_bytes(raw_response[3], byteorder='little')

    def read_present_position(self, id):
        """
        This function read the moving speed of the motor (id)
        :param id: id of the motor
        :return position: the current position of the motor in degree
        """
        raw_response = self._READ_DATA(id, 0x24, 2)
        return round(int.from_bytes(raw_response[3], byteorder='little') * self.ANGLE_UNIT, 0)

    def read_present_position_byte(self, id):
        """
        This function read the moving speed of the motor (id)
        :param id: id of the motor
        :return position: the current position of the motor in degree
        """
        raw_response = self._READ_DATA(id, 0x24, 2)
        return int.from_bytes(raw_response[3], byteorder='little')

    def read_goal_position(self, id):
        """
                This function read the moving speed of the motor (id)
                :param id: id of the motor
                :return position: the current position of the motor in degree
                """
        raw_response = self._READ_DATA(id, 0x1E, 2)
        return round(int.from_bytes(raw_response[3], byteorder='little') * self.ANGLE_UNIT, 0)

    def read_goal_position_byte(self, id):
        """
        This function read the moving speed of the motor (id)
        :param id: id of the motor
        :return position: the current position of the motor in degree
        """
        raw_response = self._READ_DATA(id, 0x1E, 2)
        return int.from_bytes(raw_response[3], byteorder='little')

    def read_present_speed(self, id):
        """
        This function read the moving speed of the motor (id) (the speed that the motor has now)
        :param id: id of the motor
        :return present_speed: the present moving speed of the motor in rpm
        """
        raw_response = self._READ_DATA(id, 0x26, 2)
        return round(int.from_bytes(raw_response[3], byteorder='little') * self.SPEED_UNIT, 0)

    def read_present_speed_byte(self, id):
        """
        This function read the moving speed of the motor (id) (the speed that the motor has now)
        :param id: id of the motor
        :return present_speed: the present moving speed of the motor in rpm
        """
        raw_response = self._READ_DATA(id, 0x26, 2)
        return int.from_bytes(raw_response[3], byteorder='little')

    def read_internal_temperature(self, id):
        """
        This function read the internal temperature of the motor (id)
        :param id: id of the motor
        :return internal_temp: the internal temperature of the motor (id) in Celsius
        """
        raw_response = self._READ_DATA(id, 0x2B, 1)
        return int.from_bytes(raw_response[3], byteorder='little')

    def read_present_voltage(self, id):
        """
        This function read the present voltage of the motor (id) (the voltage that is applied now)
        :param id: id of the motor
        :return voltage: the voltage of the motor (id) in Volt
        """
        raw_response = self._READ_DATA(id, 0x2A, 1)
        return int.from_bytes(raw_response[3], byteorder='little') * self.VOLTAGE_UNIT

    def read_angle_limit_clockwise(self,id):
        """
        This function read the clockwise angle limit
        :param id: id of the motor
        :return angle: clockwise Angle Limit in degree
        """
        raw_response = self._READ_DATA(id, 0x06, 2)
        return round(int.from_bytes(raw_response[3], byteorder='little') * self.ANGLE_UNIT, 0)

    def read_angle_limit_counterclockwise(self,id):
        """
        This function read the counterclockwise angle limit
        :param id: id of the motor
        :return angle: conterclockwise Angle Limit in degree
        """
        raw_response = self._READ_DATA(id, 0x08, 2)
        return round(int.from_bytes(raw_response[3], byteorder='little') * self.ANGLE_UNIT, 0)

    def read_angle_limit_clockwise_byte(self,id):
        """
        This function read the clockwise angle limit
        :param id: id of the motor
        :return angle: clockwise Angle Limit in degree
        """
        raw_response = self._READ_DATA(id, 0x06, 2)
        return int.from_bytes(raw_response[3], byteorder='little')

    def read_angle_limit_counterclockwise_byte(self,id):
        """
        This function read the counterclockwise angle limit
        :param id: id of the motor
        :return angle: conterclockwise Angle Limit in degree
        """
        raw_response = self._READ_DATA(id, 0x08, 2)
        return int.from_bytes(raw_response[3], byteorder='little')

    def read_max_torque_limit(self,id):
        """
        This function read the max torque limit
        :param id: id of the motor
        :return angle: clockwise Angle Limit in %
        """
        raw_response = self._READ_DATA(id, 0x0E, 2)
        return round(int.from_bytes(raw_response[3], byteorder='little') * self.TORQUE_UNIT, 0)

    def read_max_torque_limit_byte(self,id):
        """
        This function read the max torque limit
        :param id: id of the motor
        :return angle: clockwise Angle Limit
        """
        raw_response = self._READ_DATA(id, 0x0E, 2)
        return int.from_bytes(raw_response[3], byteorder='little')
    # -----------------------------------

    # ------------- SETTERS -------------
    def set_moving_speed(self, id, moving_speed):
        """
        This function set the speed of the motor (id)
        /!\ this function return the response but also MODIFY THE SETTINGS OF THE MOTOR, be extra cautious
        :param id: id of the motor
        :param moving_speed: between 0 and 110 (1023 = 0x3FF) if 0 the motor will go the the max speed without control, the byte unit is around 0.111rpm
        :return response: the responce given be the motor (id)
        """
        if 0 <= moving_speed <= 110:
            if moving_speed == 0:
                moving_speed_byte = 0
            else:
                moving_speed_byte = int(moving_speed / self.SPEED_UNIT)

            var = pseudo_conversion(moving_speed_byte)
            self._WRITE_DATA(id, 0x20, *var)
            response = self._receive_message()
            return response

        else:
            raise ValueError('the moving speed should be between [0 and 110] rmp')
        return 0

    def set_moving_speed_byte(self, id, moving_speed):
        """
        This function is the same that set_moving_speed but you choose the byte and not the speed in rpm, this function will be more precise
        /!\ this function return the response but also MODIFY THE SETTINGS OF THE MOTOR, be extra cautious
        :param id: id of the motor
        :param moving_speed: between 0 and 1023 = 0x3FF if 0 the motor will go the the max speed without control, the byte unit is around 0.111rpm
        :return response: the responce given be the motor (id)
        """
        if 0 <= moving_speed <= 0x3FF:
            var = pseudo_conversion(moving_speed)
            self._WRITE_DATA(id, 0x20, *var)
            response = self._receive_message()
            return response

        else:
            raise ValueError('the moving speed should be between [0 and 1023] rmp')
        return 0

    def set_goal_position(self, id, position):
        """
        This function set the goal position of the motor (id)
        /!\ this function return the response but also MODIFY THE SETTINGS OF THE MOTOR, be extra cautious
        :param id: id of the motor
        :param position: between 0 and 300 (1023 = 0x3FF), the byte unit is around 0.29 degree
        :return response: the responce given be the motor (id)
        """
        if 0 <= position <= 300:
            if position == 0:
                position_byte = 0
            else:
                position_byte = int(position / self.ANGLE_UNIT)

            var = pseudo_conversion(position_byte)
            self._WRITE_DATA(id, 0x1E, *var)
            response = self._receive_message()
            return response

        else:
            raise ValueError('the position should be between [0 and 300] degree')
        return 0

    def set_goal_position_byte(self, id, position):
        """
        This function is the same that set_goal_position but you choose the byte and not the speed in rpm, this function will be more precise
        /!\ this function return the response but also MODIFY THE SETTINGS OF THE MOTOR, be extra cautious
        :param id: id of the motor
        :param position: between 0 and 1023 = 0x3FF, the byte unit is around 0.29 degree
        :return response: the responce given be the motor (id)
        """
        if 0 <= position <= 0x3FF:
            var = pseudo_conversion(position)
            self._WRITE_DATA(id, 0x1E, *var)
            response = self._receive_message()
            return response

        else:
            raise ValueError('the position should be between [0 and 1023] degree')
        return 0

    def set_max_torque(self, id, torque):
        """
        This function set the torque limit of the motor
        /!\ this function return the response but also MODIFY THE SETTINGS OF THE MOTOR, be extra cautious
        This function write in the EEPROM of the motor, what is written here will not be forget
        :param id: id of the motor
        :param moving_speed: between 0 and 100% (1023 = 0x3FF)
        :return response: the responce given be the motor (id)
        """
        if 0 <= torque <= 100:
            torque_byte = int(torque / self.TORQUE_UNIT)

            var = pseudo_conversion(torque_byte)
            self._WRITE_DATA(id, 0x0E, *var)
            response = self._receive_message()
            return response

        else:
            raise ValueError('the moving speed should be between [0 and 100] %')
        return 0

    def set_max_torque_byte(self, id, torque):
        """
        This function set the torque limit of the motor
        /!\ this function return the response but also MODIFY THE SETTINGS OF THE MOTOR, be extra cautious
        This function write in the EEPROM of the motor, what is written here will not be forget
        :param id: id of the motor
        :param moving_speed: between 0 and 1023 = 0x3FF
        :return response: the responce given be the motor (id)
        """
        if 0 <= torque <= 0x3FF:

            var = pseudo_conversion(torque)
            self._WRITE_DATA(id, 0x0E, *var)
            response = self._receive_message()
            return response

        else:
            raise ValueError('the moving speed should be between [0 and 1023 = 0x3FF]')
        return 0

    def set_LED_on(self, id):
        """
        This function turn on the LED
        /!\ this function return the response but also MODIFY THE SETTINGS OF THE MOTOR, be extra cautious
        :param id: id of the motor
        :return response: the responce given be the motor (id)
        """
        self._WRITE_DATA(id, 0x19, 1)
        response = self._receive_message()
        return response

    def set_LED_off(self, id):
        """
        This function turn off the LED
        /!\ this function return the response but also MODIFY THE SETTINGS OF THE MOTOR, be extra cautious
        :param id: id of the motor
        :return response: the responce given be the motor (id)
        """
        self._WRITE_DATA(id, 0x19, 0)
        response = self._receive_message()
        return response

    # -----------------------------------

    # ----------- REG SETTERS -----------
    def set_reg_goal_position(self, id, position):
        """
        This function set the goal position of the motor (id)
        /!\ this function return the response but also MODIFY THE SETTINGS OF THE MOTOR, be extra cautious
        :param id: id of the motor
        :param position: between 0 and 300 (1023 = 0x3FF), the byte unit is around 0.29 degree
        :return response: the responce given be the motor (id)
        """
        if 0 <= position <= 300:
            if position == 0:
                position_byte = 0
            else:
                position_byte = int(position / self.ANGLE_UNIT)

            var = pseudo_conversion(position_byte)
            self._REG_WRITE(id, 0x1E, *var)
            response = self._receive_message()
            return response

        else:
            raise ValueError('the position should be between [0 and 300] degree')
        return 0

    def set_reg_goal_position_byte(self, id, position):
        """
        This function is the same that set_goal_position but you choose the byte and not the speed in rpm, this function will be more precise
        /!\ this function return the response but also MODIFY THE SETTINGS OF THE MOTOR, be extra cautious
        :param id: id of the motor
        :param position: between 0 and 1023 = 0x3FF, the byte unit is around 0.29 degree
        :return response: the responce given be the motor (id)
        """
        if 0 <= position <= 0x3FF:
            var = pseudo_conversion(position)
            self._REG_WRITE(id, 0x1E, *var)
            response = self._receive_message()
            return response

        else:
            raise ValueError('the position should be between [0 and 1023] degree')
        return 0

    # -----------------------------------