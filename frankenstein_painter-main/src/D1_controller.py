# *****************************************************************************
# Python sample programm demonstrating the communication/control of two
# dryve D1 via Modbus TCP as gateway (ethernet-based)
# Version 1.1

# Dual axis control for point-to-point movement and 2-axis gantry robot
# The supplied dryve D1 configuration must be loaded:
# "20211112_D1-2-PC-ModbusTCP(GW)-12-V1.1-1.txt" and
# "20211112_D1-2-PC-ModbusTCP(GW)-12-V1.1-2.txt"
# The inital parameters feed rate, available stroke, mode of homing and
# homing offset have to be set according to your system in the GUI before
# starting the program.
# The IP address of the dryve D1 must match with the IP address stated in the
# __main__ part below.
# All movement parameters can be adapted, e.g. speed, acceleration or position

# Please use the latest firmware available at www.igus.eu/D1!!!

# No support is provided for this sample program.
# No responsibility/liability will be assumed for the test program.
# *****************************************************************************

# let's start by importing some useful modules
import socket  # for the network connection
import struct  # to convert integer numbers to bytes
import time  # to add little delays here and there
from typing import List  # for better documentation

# *****************************************************************************
# Program configuration options  -  on = True  -   off = False
# *****************************************************************************

# turn verbose print output on of off
DEBUG = True
# turn sligt movement delay on or off
PAUSE = True

# *****************************************************************************
# Telegram message definitions
# refer to chapter "RX/TX Telegram Example" in the operating manual
# *****************************************************************************

# here are some SDO object codes that are used in this program
# for a full list have a look at the manual in section
# 6.5.20 "Objekte zur Bewegungssteuerung"
CONTROL_WORD = [96, 64]  # 6040h
STATUS_WORD = [96, 65]  # 6041h
SET_MODE = [96, 96]  # 6060h ("modes of operation")
GET_MODE = [96, 97]  # 6061h ("modes of operation display")
TARGET_POSITION = [96, 122]  # 607Ah
PROFILE_VELOCITY = [96, 129]  # 6081h
PROFILE_ACCELERATION = [96, 131]  # 6083h
FEED_CONSTANT_FEED = [96, 146, 1]  # 6092h sub 01 ("feed constant feed")
FEED_CONSTANT_SHAFT_REV = [96, 146, 2]  # 6092h sub 02 ("feed constant shaft revolutions")
HOMING_SPEED_SWITCH = [96, 153, 1]  # 6099h sub 01 ("search velocity for switch")
HOMING_SPEED_ZERO = [96, 153, 2]  # 6099h sub 02 ("search velocity for zero")
HOMING_ACCELERATION = [96, 154]  # 609Ah
SI_UNIT_POSITION = [96, 168]  # 60A8h

# here are some control words that we use
# we will need to pass them all as bytearrays
CMD_ENABLE_OPERATION = bytearray([15, 0])
CMD_SHUT_DOWN = bytearray([6, 0])
CMD_START_MOVEMENT = bytearray([31, 0])
CMD_SWITCH_ON = bytearray([7, 0])


def create_telegram(read_or_write: str,
                    sdo_code: List[int],
                    num_data_bytes: int,
                    data_array: bytearray = bytearray()) -> bytearray:
    """Helper function to create a well-formed data telegram.

    This method does not perform any semantic checks (yet), so you will have
    to do that yourself. Telegrams can be sent or received; this method covers
    both use cases.

    Args:
        read_or_write: the string 'read' for a reading operation and
                       the string 'write' for a writing operation.
        sdo_code: a list of two or three integers representing the SDO object.
        num_data_bytes: number of data bytes (0 to 4) to be sent or received.
        data_array(optional): the actual data as bytearray if applicable.
    """

    # bytes 0 and 1 are the transaction identifier, which we are not using
    # bytes 2 and 3 are the protocol identifier with the only option being 0
    start = bytearray([0, 0, 0, 0])

    # bytes 4 and 5 say how many more bytes are coming (13 to 17)
    # will be updated once we have all the other data
    length = bytearray([0, 13])

    # byte 6 is unit identifier (always 0)
    # byte 7 is a function code (for modbus always 43)
    # byte 8 is the MEI type (for modbus always 13)
    modbus = bytearray([0, 43, 13])

    # byte 9 says if we want to read or write
    # byte 10 is an unused protocol option
    # byte 11 is an unused node id
    if read_or_write == 'read':
        protocol = bytearray([0, 0, 0])
    if read_or_write == 'write':
        protocol = bytearray([1, 0, 0])

    # bytes 12 and 13 are the sdo object index code
    # byte 14 is the object sub-index if applicable
    if len(sdo_code) == 2:  # no sub-index
        sdo_code.append(0)
    sdo = bytearray(sdo_code)

    # bytes 15 and 16 are an unused address field
    # byte 17 has something to do with the sdo object but is not used
    fluff = bytearray([0, 0, 0])

    # byte 18 says how many bytes of data are following
    # bytes 19 to 22 are the data
    data = bytearray([num_data_bytes]) + data_array

    # update length now that we have everything
    payload = modbus + protocol + sdo + fluff + data
    length[1] = len(payload)

    # build final message
    return start + length + payload


def dummy(x: int) -> bytearray:
    """Create an empty bytearray with length x.

    Dummy data can be useful when creating base messages for set... commands.
    Dummy data is overwritten with real data before the message is sent.
    """
    return bytearray(x)

# a dictionary of pre-generated messages that can be used used in this program
MESSAGES = {
    # commands to change the state of the linear unit
    'switch on':
        create_telegram('write', CONTROL_WORD, 2, CMD_SWITCH_ON),
    'enable operation':
        create_telegram('write', CONTROL_WORD, 2, CMD_ENABLE_OPERATION),
    'start movement':
        create_telegram('write', CONTROL_WORD, 2, CMD_START_MOVEMENT),
    'shut down':
        create_telegram('write', CONTROL_WORD, 2, CMD_SHUT_DOWN),

    # messages to send data to the unit
    # dummy data is replaced with real data before message is sent
    'set mode':
        create_telegram('write', SET_MODE, 1, dummy(1))[:-1],
    'set feed rate':
        create_telegram('write', FEED_CONSTANT_FEED, 4, dummy(4))[:-4],
    'set shaft revolutions':
        create_telegram('write', FEED_CONSTANT_SHAFT_REV, 4, dummy(4))[:-4],
    'set homing speed for switch':
        create_telegram('write', HOMING_SPEED_SWITCH, 4, dummy(4))[:-4],
    'set homing speed for zero':
        create_telegram('write', HOMING_SPEED_ZERO, 4, dummy(4))[:-4],
    'set homing acceleration':
        create_telegram('write', HOMING_ACCELERATION, 4, dummy(4))[:-4],
    'set target position':
        create_telegram('write', TARGET_POSITION, 4, dummy(4))[:-4],
    'set profile velocity':
        create_telegram('write', PROFILE_VELOCITY, 4, dummy(4))[:-4],
    'set profile acceleration':
        create_telegram('write', PROFILE_ACCELERATION, 4, dummy(4))[:-4],

    # messages to get data and information from the unit
    'get status':
        create_telegram('read', STATUS_WORD, 2),
    'get mode':
        create_telegram('read', GET_MODE, 1),  # "mode of operation"
    'get feed rate':
        create_telegram('read', FEED_CONSTANT_FEED, 4),
    'get shaft revolutions':
        create_telegram('read', FEED_CONSTANT_SHAFT_REV, 1),
    'get SI unit position':
        create_telegram('read', SI_UNIT_POSITION, 4),
}

# a dictionary of anything that the unit might send back to us as an answer
RESPONSES = {
    # handshake responses just return the sender's message but without data
    'handshake for control word':
        create_telegram('write', CONTROL_WORD, 0),
    'handshake for set mode':
        create_telegram('write', SET_MODE, 0),
    'handshake for set feed rate':
        create_telegram('write', FEED_CONSTANT_FEED, 0),
    'handshake for set shaft revolutions':
        create_telegram('write', FEED_CONSTANT_SHAFT_REV, 0),
    'handshake for set homing speed for switch':
        create_telegram('write', HOMING_SPEED_SWITCH, 0),
    'handshake for set homing speed for zero':
        create_telegram('write', HOMING_SPEED_ZERO, 0),
    'handshake for set homing acceleration':
        create_telegram('write', HOMING_ACCELERATION, 0),
    'handshake for set target position':
        create_telegram('write', TARGET_POSITION, 0),
    'handshake for set profile velocity':
        create_telegram('write', PROFILE_VELOCITY, 0),
    'handshake for set profile acceleration':
        create_telegram('write', PROFILE_ACCELERATION, 0),

    # replies to status requests are a bit more varied
    'operation enabled v1':  # 39 = 0010 0111
        create_telegram('read', STATUS_WORD, 2, bytearray([39, 6])),
    'operation enabled target reached':
        create_telegram('read', STATUS_WORD, 2, bytearray([39, 22])),
    'operation enabled v3':
        create_telegram('read', STATUS_WORD, 2, bytearray([39, 2])),
    'operation enabled v4':
        create_telegram('read', STATUS_WORD, 2, bytearray([39, 18])),
    'operation enabled v5':
        create_telegram('read', STATUS_WORD, 2, bytearray([39, 4])),
    'ready to switch on v1':  # 33 = 0010 0001
        create_telegram('read', STATUS_WORD, 2, bytearray([33, 6])),
    'ready to switch on v2':
        create_telegram('read', STATUS_WORD, 2, bytearray([33, 22])),
    'ready to switch on v3':
        create_telegram('read', STATUS_WORD, 2, bytearray([33, 2])),
    'switched on v1':  # 35 = 0010 0011
        create_telegram('read', STATUS_WORD, 2, bytearray([35, 6])),
    'switched on v2':
        create_telegram('read', STATUS_WORD, 2, bytearray([35, 22])),
    'switched on v3':
        create_telegram('read', STATUS_WORD, 2, bytearray([35, 2])),
    'switch on disabled v1':  # 64 = 1000 0000
        create_telegram('read', STATUS_WORD, 2, bytearray([64, 4])),
    'switch on disabled v2':
        create_telegram('read', STATUS_WORD, 2, bytearray([64, 6])),
}

# *****************************************************************************
# Controller description as Python class called "D1".

# Python classes typically have attributes and functions. Attributes describe
# something that a class HAS, while functions describe something it can DO.
# An attribute of the Controller is, for example, its IP address.
# A function of the Controller is, for example, sending a command to the unit.

# The cool thing about classes is that they are like a template. You can
# create multiple objects from a class that inherit its characteristics.
# Down below in __main__, we create two objects from the class D1. They can
# both DO the same things (like sending a command), but they HAVE different
# attributes (check the IP address for example). And each object, in our case
# the controller for the X-axis and the controller for the Y-axis, works
# independently. So when the controller for the X-axis sends a command to its
# unit, it will use a different IP address than the controller for the Y-axis.
# Such is the magic of classes.

# An object also doesn't have to DO everything that the class can DO. The only
# function that will be executed is the __init__ function, which is run when
# an object is first created in the __main__ code. All other functions of the
# object can be called afterwards depending on what you want the object to do.
# *****************************************************************************


class D1:

    def __init__(self, ip_address: str, port: int = 502,
                 label: str = "axis_name", multi_move: bool = False):
        """This function is called when a new D1 controller object is created.

        The controller can establish an ethernet connection and send telegrams
        with movement instructions to the corresponding linear unit.

        Args:
            ip_address (string): the IP address of the linear unit.
            port (number): the port of the linear unit. Default is 502.
            label (string): a name to identify the axis.
            multi_move (True or False): allow simultaneous movements of two
            controllers or not. Default is False.

        You only need to provide an IP address to create a controller object.
        Port, label, and multi_move will then use the default values.
        """

        # remember the settings for this controller
        self.ip_address = ip_address
        self.port = port
        self.label = label
        self.multi_move = multi_move
        self.mode = 0  # keep track of current mode

        # initialise this controller for the state machine
        # for more information see chapter "State Machine Visualisation after
        # Boot Up" in the operating manual
        self.set_shutdown()
        self.set_switch_on()
        self.set_enable_operation()

        # get the SI unit position to calculate multiplication factor
        # for more information see SDO object 60A8h in the manual
        si = self.send_telegram(MESSAGES['get SI unit position'])
        # note: the following two lines of code use slicing intentionally
        # because from_bytes() expects an iterable/list
        # get type of movement (linear or rotary) from byte 2
        si_type = int.from_bytes(si[21:22], byteorder='big')
        # get exponent from byte 3
        exponent = int.from_bytes(si[22:], byteorder='little', signed=True)
        # calculate the multiplication factor when linear movement is set
        if si_type == 1:  # 01h = linear
            self.SI_unit_factor = (10 ** -3) / (10 ** exponent)
        # calculate the multiplication factor when rotary movement is set
        if si_type == 65:  # 41h = rotary
            self.SI_unit_factor = 10 ** -exponent

        # set shaft revolutions to 1 - needs a 4-byte array
        # TODO: do we not need to set the feed rate as well?
        self.send_telegram(MESSAGES['set shaft revolutions'] + bytearray([1, 0, 0, 0]))

        # Read the feed rate from the D1 and convert it to integer [mm]
        # (this is not used anywhere, just an artefact from the original code)
        fr = self.send_telegram(MESSAGES['get feed rate'])[19:]
        self.feed_rate = (int.from_bytes(fr, byteorder='little')) / self.SI_unit_factor

    def get_msg_name(self, msg_data: bytearray) -> str:
        """Helper function to get message name from message dictionary.

        Args:
            msg_data (bytearray): the content of the telegram message.

        Returns:
            the message name as string if the message is in one of the two
            dictionaries defined at the top of this file (MESSAGES/RESPONSES),
            otherwise a string representation of the original data.
        """

        # first search through message dict
        for msg, data in MESSAGES.items():
            if data == msg_data:
                return msg

        # now search through responses dict
        for msg, data in RESPONSES.items():
            if data == msg_data:
                return msg

        # still not found? return original data as string
        return str(list(msg_data))

    def get_status(self) -> bytearray:
        """This is a shortcut to ask for the current status."""
        return self.send_telegram(MESSAGES['get status'])

    # TODO: this implementation does not follow the example in the manual
    def homing(self, switch_search_speed: int, acceleration: int) -> None:
        """Function "Homing" - execute a homing

        Mode of homing and offset has to be set in the GUI beforehand.

        Args:
            Switch search speed as integer value in mm/s
            Acceleration as integer value in mm/s²
        """

        # set Homing mode
        if DEBUG:
            print(self.label, "requesting Homing mode")
        self.set_mode(6)

        # multiply velocity and acceleration by SI unit factor and convert to
        # bytearray for network transport
        sss = self.multiply_by_si_and_convert(switch_search_speed)
        acc = self.multiply_by_si_and_convert(acceleration)

        # set homing speed ("search velocity for switch") and acceleration
        self.send_telegram(MESSAGES['set homing speed for switch'] + sss)
        self.send_telegram(MESSAGES['set homing acceleration'] + acc)

        # Enable Operation to set bit 4 of the controlword to low again
        self.send_telegram(MESSAGES['enable operation'])
        # Write Controlword 6040h Command: Start Movement; high flank of bit 4
        self.send_telegram(MESSAGES['start movement'])

        # check if homing is done yet
        while self.get_status() != RESPONSES['operation enabled target reached']:
            if DEBUG:
                print(self.label, "waiting for Homing to end")
            if PAUSE:
                time.sleep(0.1)  # delay in seconds between checking

    def multiply_by_si_and_convert(self, x: int) -> bytearray:
        """Multiplies a value by SI unit factor and converts it to bytearray.

        This function takes a value that needs to be sent to the linear unit
        (e.g., speed/velocity, acceleration, position), multiplies it by the
        SI unit factor of the linear unit, and then converts it to a bytearray
        that can be sent as data in a telegram to the unit.
        """
        # "<i" = use little endian format and convert integer
        return bytearray(struct.pack("<i", int(x * self.SI_unit_factor)))

    # TODO: this implementation does not follow the example in the manual
    def profile_pos_mode(self, position: int, velocity: int, acceleration: int) -> None:
        """Function "Profile Position Mode"

        Move to an absolute position with given velocity and acceleration.

        Args:
            position as integer value in mm
            velocity as integer value in mm/s
            acceleration as integer value in mm/s²
        """

        # set Profile Position mode
        if DEBUG:
            print(self.label, "requesting profile position mode")
        self.set_mode(1)

        # multiply velocity, acceleration, and position by SI unit factor and
        # convert to bytearray for network transport
        vel = self.multiply_by_si_and_convert(velocity)
        acc = self.multiply_by_si_and_convert(acceleration)
        pos = self.multiply_by_si_and_convert(position)

        # set profile velocity, profile acceleration, and target position
        self.send_telegram(MESSAGES['set profile velocity'] + vel)
        self.send_telegram(MESSAGES['set profile acceleration'] + acc)
        self.send_telegram(MESSAGES['set target position'] + pos)

        # Enable Operation to set bit 4 of the controlword to low again
        self.send_telegram(MESSAGES['enable operation'])
        # Write Controlword 6040h Command: Start Movement; high flank of bit 4
        self.send_telegram(MESSAGES['start movement'])

        # A new movement only starts after completing a previous movement
        if not self.multi_move:
            while self.get_status() != RESPONSES['operation enabled target reached']:
                if DEBUG:
                    print(self.label, "waiting for target reached")
                if PAUSE:
                    time.sleep(0.1)  # delay in seconds between checking

    def send_telegram(self, data: bytearray) -> bytearray:
        """Send a telegram message to the linear unit and return the reply.

        Create a network socket, send message, and return response.

        Args:
            data (bytearray): the telegram message that will be sent.

        Returns:
            response (bytearray): a bytearray with the response.
        """

        # print some information on the request
        if DEBUG:
            print("Request from %s:" % self.label)
            print(self.get_msg_name(data))

        # connect, send, receive - and catch any network errors
        success = False
        while not success:
            try:
                with socket.create_connection((self.ip_address, self.port)) as s:
                    if s.sendall(data) is None:
                        # no data left => all data was sent successfully
                        # we're expecting a response with no more than 24 bytes
                        response = bytearray(s.recv(24))
                        success = True
            except OSError as msg:
                print("Communication error in send_telegram():", msg)
                print("Trying to send the message again...")

        # print some information on the response
        if DEBUG:
            print("Response to %s:" % self.label)
            print(self.get_msg_name(response))

        return response

    def set_enable_operation(self) -> None:
        """Tell the linear unit to enable operation.

        For more information see chapter "State Machine Visualisation after
        Boot Up" in the operating manual.
        """
        self.send_telegram(MESSAGES['enable operation'])
        expected_responses = [RESPONSES['operation enabled v1'],
                              RESPONSES['operation enabled target reached'],
                              RESPONSES['operation enabled v3'],
                              RESPONSES['operation enabled v4'],
                              RESPONSES['operation enabled v5']]
        # wait until one of the expected responses is returned
        while self.get_status() not in expected_responses:
            if DEBUG:
                print(self.label, "waiting for operation enabled")
            if PAUSE:
                time.sleep(0.1)  # delay in seconds between checking

    def set_mode(self, mode: int) -> None:
        """Function to change the mode of operation.

        Possible values:
        1 - Profile Position Mode
        3 - Profile Velocity Mode
        6 - Homing Mode
        8 - Cyclic Synchronous Position Mode
        For further information see operating manual chapter "Homing" ff.
        """

        if self.mode == mode:
            # we are already in this mode, no need to request a change
            if DEBUG:
                print("already in this mode")
            return

        if DEBUG:
            print("changing mode")

        # convert mode to bytearray
        bmode = bytearray([mode])

        # add mode to base message and request mode change from linear unit
        self.send_telegram(MESSAGES['set mode'] + bmode)

        # response looks like 'get mode' but contains the mode we requested
        expected_response = MESSAGES['get mode'] + bmode
        # update message length because we now have one data byte
        expected_response[5] += 1

        # wait for the mode to be set
        while self.send_telegram(MESSAGES['get mode']) != expected_response:
            if DEBUG:
                print(self.label, "waiting for mode to be set")
            if PAUSE:
                time.sleep(0.1)  # delay in seconds between checking

        # finally, update our internal mode
        self.mode = mode

    def set_shutdown(self) -> None:
        """Tell the linear unit to shut down.

        For more information see chapter "State Machine Visualisation after
        Boot Up" in the operating manual.
        """
        self.send_telegram(MESSAGES['shut down'])
        expected_responses = [RESPONSES['ready to switch on v1'],
                              RESPONSES['ready to switch on v2'],
                              RESPONSES['ready to switch on v3']]
        # wait until one of the expected responses is returned
        while self.get_status() not in expected_responses:
            if DEBUG:
                print(self.label, "waiting for shutdown")
            if PAUSE:
                time.sleep(0.1)  # delay in seconds between checking

    def set_switch_on(self) -> None:
        """Tell the linear unit to switch on.

        For more information see chapter "State Machine Visualisation after
        Boot Up" in the operating manual.
        """
        self.send_telegram(MESSAGES['switch on'])
        expected_responses = [RESPONSES['switched on v1'],
                              RESPONSES['switched on v2'],
                              RESPONSES['switched on v3']]
        # wait until one of the expected responses is returned
        while self.get_status() not in expected_responses:
            if DEBUG:
                print(self.label, "waiting for switch on")
            if PAUSE:
                time.sleep(0.1)  # delay in seconds between checking

    def wait_for_ready(self) -> None:
        """Function to wait for movement to be finished.

        Checks the status if target was reached and a new setpoint is given,
        also if operation enabled.
        """
        while self.get_status() != RESPONSES['operation enabled target reached']:
            if DEBUG:
                print(self.label, "waiting for ready")
            if PAUSE:
                time.sleep(0.1)  # delay in seconds between checking

# *****************************************************************************
# Main part of the program
# *****************************************************************************

if __name__ == '__main__':
    """Start Main Program

    Create two new objects of our controller, one for X and one for Y axis.
    Then perform homing operation on both.
    Then start an endless loop to make them move to certain positions.
    If the user presses Ctrl + c ("Keyboard Interrupt"), the loop is stopped
    and the controllers are shut down.
    """

    # Set up the communication and initialisation of the D1 controller
    # Define D1 as axis (ip_address, port, label, multi_move)
    x_axis = D1("169.254.0.1", 502, "Linear X-Axis", False)
    y_axis = D1("169.254.0.2", 502, "Linear Y-Axis", False)

    # Homing of X and Y axis; only once to allow absolute movement
    # function wants switch search speed in mm/s and acceleration in mm/s²
    x_axis.homing(60, 600)
    y_axis.homing(60, 600)
    # wait for 100 ms before starting new movement
    time.sleep(0.1)

    try:
        while True:
            # function profile_pos_mode wants position in mm, velocity in mm/s
            # and acceleration in mm/s²
            # move X axis to absolute position 0 mm with 60 mm/s and 600 mm/s²
            x_axis.profile_pos_mode(0, 60, 600)
            # move Y axis to absolute position 14 mm with 30 mm/s and 600 mm/s²
            y_axis.profile_pos_mode(14, 30, 600)
            # wait for 250 ms before starting new movement
            time.sleep(0.25)

            # move Y axis to absolute position 0 mm with 30 mm/s and 600 mm/s²
            y_axis.profile_pos_mode(0, 30, 600)
            # move X axis to absolute position 85 mm with 60 mm/s and 600 mm/s²
            x_axis.profile_pos_mode(85, 60, 600)
            # wait for 1 s before starting new movement
            time.sleep(1)

            # Enable multi axis move
            x_axis.multi_move = True
            y_axis.multi_move = True

            # move X axis to absolute position 0 mm with 60 mm/s and 600 mm/s²
            x_axis.profile_pos_mode(0, 60, 600)
            # move Y axis to absolute position 14 mm with 10 mm/s and 600 mm/s²
            y_axis.profile_pos_mode(14, 10, 600)
            # wait for X axis to complete the movement and reach the target
            x_axis.wait_for_ready()
            # wait for Y axis to complete the movement and reach the target
            y_axis.wait_for_ready()
            # wait for 100 ms before starting new movement
            time.sleep(0.1)

            # move X axis to absolute position 85 mm with 60 mm/s and 600 mm/s²
            x_axis.profile_pos_mode(85, 60, 600)
            # move Y axis to absolute position 0 mm with 10 mm/s and 600 mm/s²
            y_axis.profile_pos_mode(0, 10, 600)
            # wait for X axis to complete the movement and reach the target
            x_axis.wait_for_ready()
            # wait for Y axis to complete the movement and reach the target
            y_axis.wait_for_ready()
            # wait for 100 ms before starting new movement
            time.sleep(0.1)

            # Disable multi axis move
            x_axis.multi_move = False
            y_axis.multi_move = False

    except KeyboardInterrupt as ki:
        # stop machine when pressing Ctrl + c
        x_axis.set_shutdown()
        y_axis.set_shutdown()
