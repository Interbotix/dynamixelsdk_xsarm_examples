import time
from dynamixel_sdk import *

ADDR_DRIVE_MODE           = 10
ADDR_OPERATING_MODE       = 11
ADDR_SECONDARY_ID         = 12
ADDR_HOMING_OFFSET        = 20
ADDR_MAX_POS_LIMIT        = 48
ADDR_MIN_POS_LIMIT        = 52
ADDR_TORQUE_ENABLE        = 64
ADDR_POSITION_P_GAIN      = 84
ADDR_GOAL_PWM             = 100
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY     = 112
ADDR_GOAL_POSITION        = 116
ADDR_PRESENT_LOAD         = 126
ADDR_PRESENT_POSITION     = 132
ADDR_PRESENT_TEMP         = 146

LEN_DRIVE_MODE            = 1
LEN_OPERATING_MODE        = 1
LEN_SECONDARY_ID          = 1
LEN_HOMING_OFFSET         = 4
LEN_MAX_POS_LIMIT         = 4
LEN_MIN_POS_LIMIT         = 4
LEN_TORQUE_ENABLE         = 1
LEN_POSITION_P_GAIN       = 2
LEN_GOAL_PWM              = 2
LEN_PROFILE_ACCELERATION  = 4
LEN_PROFILE_VELOCITY      = 4
LEN_GOAL_POSITION         = 4
LEN_PRESENT_LOAD          = 2
LEN_PRESENT_POSITION      = 4
LEN_PRESENT_TEMP          = 1

PI                        = 3.14159265

portHandler   = None
packetHandler = None

### @brief Helper function to check error messages and print them if need be
### @param dxl_comm_result - if nonzero, there is an error in the communication
### @param dxl_error - if nonzero, there is an error in something related to the data
### @return <bool> - true if no error occurred; false otherwise
def checkError(dxl_comm_result, dxl_error):
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return False
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        return False
    return True

### @brief Writes data to the specified register for a given motor
### @param id - Dynamixel ID to write data to
### @param address - register number
### @param data - data to write
### @param length - size of register in bytes
### @return <bool> - true if data was successfully written; false otherwise
def itemWrite(id, address, data, length):
    if length == 1:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, address, data)
    elif length == 2:
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, address, data)
    elif length == 4:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, address, data)
    else:
        print("Invalid data length...")
        return False
    return checkError(dxl_comm_result, dxl_error)

### @brief Sequentially writes the same data to the specified register for a group of motors
### @param ids - vector of IDs to write data to
### @param address - register number
### @param data - data to write (could be a list [index matches index in 'ids' list] or a single value [for all ids])
### @param length - size of register in bytes
### @return <bool> - true if data was successfully written; false otherwise
def itemWriteMultiple(ids, address, data, length):
    if type(data) != list:
        for id in ids:
            success = itemWrite(id, address, data, length)
            if success != True:
                return False
    else:
        for id, dat in zip(ids, data):
            success = itemWrite(id, address, dat, length)
            if success != True:
                return False
    return True

### @brief Reads data from the specified register for a given motor
### @param id - Dynamixel ID to read data from
### @param address - register number
### @param length - size of register in bytes
### @return state - variable to store the requested data
### @return <bool> - true if data was successfully retrieved; false otherwise
### @details - DynamixelSDK uses 2's complement so we need to check to see if 'state' should be negative (hex numbers)
def itemRead(id, address, length):
    if length == 1:
        state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, id, address)
    elif length == 2:
        state, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, id, address)
        if state > 0x7fff:
            state = state - 65536
    elif length == 4:
        state, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, address)
        if state > 0x7fffffff:
            state = state - 4294967296
    else:
        print("Invalid data length...")
        return 0, False
    return state, checkError(dxl_comm_result, dxl_error)

### @brief Sequentially reads data from the specified register for a group of motors
### @param id - vector of Dynamixel IDs to read data from
### @param address - register number
### @param length - size of register in bytes
### @return states - list to store the requested data
### @return <bool> - true if data was successfully retrieved; false otherwise
def itemReadMultiple(ids, address, length):
    states = []
    for id in ids:
        state, success = itemRead(id, address, length)
        if success != True:
            return [], False
        states.append(state)
    return states, True

### @brief Writes data to a group of motors synchronously
### @param groupSyncWrite - groupSyncWrite object
### @param ids - list of Dynamixel IDs to write to
### @param commands - list of commands to write to each respective motor
### @param length - size of register in bytes
### @return <bool> - true if data was successfully written; false otherwise
def syncWrite(groupSyncWrite, ids, commands, length):
    groupSyncWrite.clearParam()
    for id, cmd in zip(ids, commands):
        param = []
        if length == 4:
            param.append(DXL_LOBYTE(DXL_LOWORD(cmd)))
            param.append(DXL_HIBYTE(DXL_LOWORD(cmd)))
            param.append(DXL_LOBYTE(DXL_HIWORD(cmd)))
            param.append(DXL_HIBYTE(DXL_HIWORD(cmd)))
        elif length == 2:
            param.append(DXL_LOBYTE(cmd))
            param.append(DXL_HIBYTE(cmd))
        else:
            param.append(cmd)
        dxl_addparam_result = groupSyncWrite.addParam(id, param)
        if dxl_addparam_result != True:
            print("ID:%03d groupSyncWrite addparam failed" % id)
            return False
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return False
    return True

### @brief Reads data from a group of motors synchronously
### @param groupSyncRead - groupSyncRead object
### @param ids - list of Dynamixel IDs to read from
### @param address - register number
### @param length - size of register in bytes
### @return states - list to store the requested data
### @return <bool> - true if data was successfully retrieved; false otherwise
### @details - DynamixelSDK uses 2's complement so we need to check to see if 'state' should be negative (hex numbers)
def syncRead(groupSyncRead, ids, address, length):
    groupSyncRead.clearParam()
    for id in ids:
        dxl_addparam_result = groupSyncRead.addParam(id)
        if dxl_addparam_result != True:
            print("ID:%03d groupSyncRead addparam failed", id)
            return [], False

    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return [], False

    states = []
    for id in ids:
        state = groupSyncRead.getData(id, address, length)
        if length == 2 and state > 0x7fff:
            state = state - 65536
        elif length == 4 and state > 0x7fffffff:
            state = state - 4294967296
        states.append(state)
    return states, True

### @brief Calibrates a dual-motor joint so that both motors show the same Present Position value
### @param master_id - in a dual-motor joint setup, this is the ID with the lower value
### @param slave_id - in a dual-motor joint setup, this is the ID with the higher value
### @details - note that the Drive mode must be set beforehand for both motors (specifically if CW or CCW is positive)
def calibrateDualJoint(master_id, slave_id):
    itemWrite(slave_id, ADDR_HOMING_OFFSET, 0, LEN_HOMING_OFFSET)
    master_position,_ = itemRead(master_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    slave_position,_ = itemRead(slave_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    slave_drive_mode,_ = itemRead(slave_id, ADDR_DRIVE_MODE, LEN_DRIVE_MODE)
    slave_forward = (slave_drive_mode % 2 == 0)
    print("Master Position: %d" % master_position)
    print("Slave Position: %d" % slave_position)
    if slave_forward:
        homing_offset = master_position - slave_position
    else:
        homing_offset = slave_position - master_position
    print("Setting Slave Homing Offset Register to: %d" % homing_offset)
    itemWrite(slave_id, ADDR_HOMING_OFFSET, homing_offset, LEN_HOMING_OFFSET)

### @brief Initializes the port that the U2D2 is connected to
### @param port_name - name of the port
### @baudrate - desired baudrate in bps (should be the same as the motors)
### @return <bool> - true if the port was initialized; false otherwise
def initPort(port_name, baudrate):
    global portHandler, packetHandler
    portHandler = PortHandler(port_name)
    packetHandler = PacketHandler(2.0)

    if portHandler.openPort():
        print("Successfully opened the port at %s!" % port_name)
    else:
        print("Failed to open the port at %s!", port_name)
        return False

    if portHandler.setBaudRate(baudrate):
        print("Succeeded to change the baudrate to %d bps!" % baudrate)
    else:
        print("Failed to change the baudrate to %d bps!" % baudrate)
        return False

    return True

### @brief Ping the desired motors to verify their existence
### @param ids - vector of Dynamixel IDs to ping
### @return <bool> - true if all IDs were pinged successfully
def ping(ids):
    for id in ids:
        model_num, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, id)
        success = checkError(dxl_comm_result, dxl_error)
        if success:
            print("Pinged ID: %03d successfully! Model Number: %d" % (id,model_num))
        else:
            return False
    return True

### @brief Convert a raw Position register value into radians
### @param value - desired raw value to convert
### @return <float> - value in radians
def convertValue2Radian(value):
    return (value - 2047.5)/2047.5 * PI

### @brief Convert a value in radians to the raw Position register value
### @param radian - desired value in radians to convert
### @return <float> - raw register value
def convertRadian2Value(radian):
    return round(radian/PI * 2047.5 + 2047.5)

def main():
  ## WX200 ARM EEPROM CONFIGS
    all_ids = [1,2,3,4,5,6,7]
    drive_modes = [4,4,5,5,5,4,4]
    shadow_ids = [255,255,2,255,255,255,255]
    min_position_limits = [0,0,0,0,0,0,0]
    max_position_limits = [4095,4095,4095,4095,4095,4095,4095]
    operating_modes = [3,3,3,3,3,3,16]
    position_p_gains = [800, 1500, 2000, 1000, 640]

    ## WX200 ARM 'Command' IDs
    arm_ids = [1,2,4,5,6]
    gripper_id = 7
    shoulder_master_id = 2
    shoulder_slave_id = 3

  ## Initialize the port, ping the motors, and create syncWrite and syncRead objects
  ## It's faster and better design to read/write motors with the 'sync' objects than to command each motor sequentially
  ## Commanding each motor sequentially should only be done for 'non-realtime sensitive' registers - like torquing on/off, setting EEPROM registers, etc...
    if not initPort("/dev/ttyDXL", 1000000):
        return
    if not ping(all_ids):
        portHandler.closePort()
        return
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    ## Initialize register values
    itemWriteMultiple(all_ids, ADDR_TORQUE_ENABLE, 0, LEN_TORQUE_ENABLE)
    itemWriteMultiple(all_ids, ADDR_DRIVE_MODE, drive_modes, LEN_DRIVE_MODE)
    itemWriteMultiple(all_ids, ADDR_SECONDARY_ID, shadow_ids, LEN_SECONDARY_ID)
    itemWriteMultiple(all_ids, ADDR_MIN_POS_LIMIT, min_position_limits, LEN_MIN_POS_LIMIT)
    itemWriteMultiple(all_ids, ADDR_MAX_POS_LIMIT, max_position_limits, LEN_MAX_POS_LIMIT)
    itemWriteMultiple(all_ids, ADDR_OPERATING_MODE, operating_modes, LEN_OPERATING_MODE)
    itemWriteMultiple(arm_ids, ADDR_POSITION_P_GAIN, position_p_gains, LEN_POSITION_P_GAIN)
    itemWriteMultiple(arm_ids, ADDR_PROFILE_VELOCITY, 1500, LEN_PROFILE_VELOCITY)
    itemWriteMultiple(arm_ids, ADDR_PROFILE_ACCELERATION, 750, LEN_PROFILE_ACCELERATION)
    calibrateDualJoint(shoulder_master_id, shoulder_slave_id)
    itemWriteMultiple(all_ids, ADDR_TORQUE_ENABLE, 1, LEN_TORQUE_ENABLE)

    ## Read current arm joint positions
    positions, success = syncRead(groupSyncRead, arm_ids, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    print(positions)

    ## Write home and sleep positions to the arm joints in sequence
    home_positions = [2048] * 5
    sleep_positions = [2048, 803, 3099, 2570, 2048]
    syncWrite(groupSyncWrite, arm_ids, home_positions, LEN_GOAL_POSITION)
    time.sleep(2)
    syncWrite(groupSyncWrite, arm_ids, sleep_positions, LEN_GOAL_POSITION)
    time.sleep(2)

    ## Command the gripper to open for 2 seconds, then close for 2 seconds
    itemWrite(gripper_id, ADDR_GOAL_PWM, 350, LEN_GOAL_PWM)
    time.sleep(2)
    itemWrite(gripper_id, ADDR_GOAL_PWM, -350, LEN_GOAL_PWM)
    time.sleep(2)
    itemWrite(gripper_id, ADDR_GOAL_PWM, 0, LEN_GOAL_PWM)

    ## Read the present temperature of all the motors
    temps, success = itemReadMultiple(all_ids, ADDR_PRESENT_TEMP, LEN_PRESENT_TEMP)
    for id, temp in zip(all_ids, temps):
        print("ID: %03d Temperature is: %d degrees Celsius." % (id, temp))

    ## Read the present loads/currents of all motors
    loads, success = itemReadMultiple(all_ids, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD)
    print(loads)

    portHandler.closePort()

if __name__=='__main__':
    main()
