#include <cmath>
#include <unistd.h>
#include <dynamixel_sdk.h>

#define ADDR_DRIVE_MODE             10
#define ADDR_OPERATING_MODE         11
#define ADDR_SECONDARY_ID           12
#define ADDR_HOMING_OFFSET          20
#define ADDR_MAX_POS_LIMIT          48
#define ADDR_MIN_POS_LIMIT          52
#define ADDR_TORQUE_ENABLE          64
#define ADDR_POSITION_P_GAIN        84
#define ADDR_GOAL_PWM              100
#define ADDR_PROFILE_ACCELERATION  108
#define ADDR_PROFILE_VELOCITY      112
#define ADDR_GOAL_POSITION         116
#define ADDR_PRESENT_LOAD          126
#define ADDR_PRESENT_POSITION      132
#define ADDR_PRESENT_TEMP          146

#define PI                         3.14159265

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

/// @brief Helper function to check error messages and print them if need be
/// @param dxl_error - if nonzero, there is an error in something related to the data
/// @param dxl_comm_result - if nonzero, there is an error in the communication
/// @return <bool> - true if no error occurred; false otherwise
bool checkError(uint8_t dxl_error, int dxl_comm_result)
{
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    return false;
  }
  return true;
}

/// @brief Writes data to the specified register for a given motor
/// @param id - Dynamixel ID to write data to
/// @param address - register number
/// @param data - data to write (user must provide the correct type!!!)
/// @return <bool> - true if data was successfully written; false otherwise
template <typename T>
bool itemWrite(uint8_t id, uint16_t address, T data)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result;
  if (sizeof(T) == 1)
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, address, data, &dxl_error);
  else if (sizeof(T) == 2)
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, address, data, &dxl_error);
  else if (sizeof(T) == 4)
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, address, data, &dxl_error);
  else
  {
    printf("Invalid type...");
    return false;
  }
  return checkError(dxl_error, dxl_comm_result);
}

/// @brief Sequentially writes the same data to the specified register for a group of motors
/// @param id - vector of IDs to write data to
/// @param address - register number
/// @param data - data to write (user must provide the correct type!!!)
/// @return <bool> - true if data was successfully written; false otherwise
template <typename T>
bool itemWriteMultiple(std::vector<uint8_t> *ids, uint16_t address, T data)
{
  for (auto const& id : *ids)
  {
    if (itemWrite(id, address, data) != true)
      return false;
  }
  return true;
}

/// @brief Sequentially writes different data to the specified register for a group of motors
/// @param id - vector of IDs to write data to
/// @param address - register number
/// @param data - data to write for each motor (user must provide the correct type!!!)
/// @return <bool> - true if data was successfully written; false otherwise
template <typename T>
bool itemWriteMultiple(std::vector<uint8_t> *ids, uint16_t address, std::vector<T> *data)
{
  for (size_t i{0}; i < ids->size(); i++)
  {
    if (itemWrite(ids->at(i), address, data->at(i)) != true)
      return false;
  }
  return true;
}

/// @brief Reads data from the specified register for a given motor
/// @param id - Dynamixel ID to read data from
/// @param address - register number
/// @param state - variable to store the requested data (user must provide the correct type!!!)
/// @return <bool> - true if data was successfully retrieved; false otherwise
template <typename T>
bool itemRead(uint8_t id, uint16_t address, T *state)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result;

  if (sizeof(T) == 1)
  {
    uint8_t raw_data;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, address, &raw_data, &dxl_error);
    *state = raw_data;
  }
  else if (sizeof(T) == 2)
  {
    uint16_t raw_data;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, address, &raw_data, &dxl_error);
    *state = raw_data;
  }
  else if (sizeof(T) == 4)
  {
    uint32_t raw_data;
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, address, &raw_data, &dxl_error);
    *state = raw_data;
  }
  else
  {
    printf("Invalid data type.");
    return false;
  }
  return checkError(dxl_error, dxl_comm_result);
}

/// @brief Sequentially reads data from the specified register for a group of motors
/// @param id - vector of Dynamixel IDs to read data from
/// @param address - register number
/// @param states - vector to store the requested data for each motor (user must provide the correct type!!!)
/// @return <bool> - true if data was successfully retrieved; false otherwise
template <typename T>
bool itemReadMultiple(std::vector<uint8_t> *ids, uint16_t address, std::vector<T> *states)
{
  T data;
  states->reserve(ids->size());
  for (auto const& id : *ids)
  {
    if (itemRead(id, address, &data) != true)
      return false;
    states->push_back(data);
  }
  return true;
}

/// @brief Writes data to a group of motors synchronously
/// @param groupSyncWrite - groupSyncWrite object
/// @param ids - vector of Dynamixel IDs to write to
/// @param commands - vector of commands to write to each respective motor (user must provide the correct type!!!)
/// @return <bool> - true if data was successfully written; false otherwise
template <typename T>
bool syncWrite(dynamixel::GroupSyncWrite *groupSyncWrite, std::vector<uint8_t> *ids, std::vector<T> *commands)
{
  groupSyncWrite->clearParam();
  bool dxl_addparam_result = false;
  uint8_t param[sizeof(T)] = {0};

  for (size_t i{0}; i < commands->size(); i++)
  {
    T data = commands->at(i);
    if (sizeof(T) == 4)
    {
      param[0] = DXL_LOBYTE(DXL_LOWORD(data));
      param[1] = DXL_HIBYTE(DXL_LOWORD(data));
      param[2] = DXL_LOBYTE(DXL_HIWORD(data));
      param[3] = DXL_HIBYTE(DXL_HIWORD(data));
    }
    else if (sizeof(T) == 2)
    {
      param[0] = DXL_LOBYTE(data);
      param[1] = DXL_HIBYTE(data);
    }
    else
    {
      param[0] = data;
    }
    dxl_addparam_result = groupSyncWrite->addParam(ids->at(i), param);
    if (dxl_addparam_result != true)
    {
      printf("ID:%03d groupSyncWrite addparam failed", ids->at(i));
      return false;
    }
  }

  int dxl_comm_result = groupSyncWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    return false;
  }
  return true;
}

/// @brief Reads data from a group of motors synchronously
/// @param groupSyncRead - groupSyncRead object
/// @param ids - vector of Dynamixel IDs to read from
/// @param address - register number
/// @param states - vector to store the requested data for each motor (user must provide the correct type!!!)
/// @return <bool> - true if data was successfully read; false otherwise
template <typename T>
bool syncRead(dynamixel::GroupSyncRead *groupSyncRead, std::vector<uint8_t> *ids, uint16_t address, std::vector<T> *states)
{
  groupSyncRead->clearParam();
  bool dxl_addparam_result = false;
  for (auto const& id : *ids)
  {
    dxl_addparam_result = groupSyncRead->addParam(id);
    if (dxl_addparam_result != true)
    {
      printf("ID:%03d groupSyncRead addparam failed", id);
      return false;
    }
  }

  bool dxl_comm_result = groupSyncRead->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    return false;
  }

  uint16_t data_length = sizeof(T);
  states->reserve(ids->size());
  for (auto const& id : *ids)
    states->push_back(groupSyncRead->getData(id, address, data_length));
  return true;
}

/// @brief Calibrates a dual-motor joint so that both motors show the same Present Position value
/// @param master_id - in a dual-motor joint setup, this is the ID with the lower value
/// @param slave_id - in a dual-motor joint setup, this is the ID with the higher value
/// @details - note that the Drive mode must be set beforehand for both motors (specifically if CW or CCW is positive)
void calibrateDualJoint(uint8_t master_id, uint8_t slave_id)
{
  uint8_t slave_drive_mode;
  int32_t master_position, slave_position, homing_offset;
  itemWrite(slave_id, ADDR_HOMING_OFFSET, (int32_t)0);
  itemRead(master_id, ADDR_PRESENT_POSITION, &master_position);
  itemRead(slave_id, ADDR_PRESENT_POSITION, &slave_position);
  itemRead(slave_id, ADDR_DRIVE_MODE, &slave_drive_mode);
  bool slave_forward = (slave_drive_mode % 2 == 0);
  printf("Master Position: %d\nSlave Position: %d\n", master_position, slave_position);
  if (slave_forward)
    homing_offset = master_position - slave_position;
  else
    homing_offset = slave_position - master_position;
  printf("Setting Slave Homing Offset Register to: %d.\n", homing_offset);
  itemWrite(slave_id, ADDR_HOMING_OFFSET, homing_offset);
}

/// @brief Initializes the port that the U2D2 is connected to
/// @param port_name - name of the port
/// @baudrate - desired baudrate in bps (should be the same as the motors)
/// @return <bool> - true if the port was initialized; false otherwise
bool initPort(const char *port_name, const int baudrate)
{
  portHandler = dynamixel::PortHandler::getPortHandler(port_name);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

  // Open port
  if (portHandler->openPort())
    printf("Successfully opened the port at %s!\n", port_name);
  else
  {
    printf("Failed to open the port at %s!\n", port_name);
    return false;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(baudrate))
    printf("Succeeded to change the baudrate to %d bps!\n", baudrate);
  else
  {
    printf("Failed to change the baudrate to %d bps!\n", baudrate);
    return false;
  }

  return true;
}

/// @brief Ping the desired motors to verify their existence
/// @param ids - vector of Dynamixel IDs to ping
/// @return <bool> - true if all IDs were pinged successfully
bool ping(std::vector<uint8_t> ids)
{
  bool ping_successful = true;
  for (auto const& id : ids)
  {
    uint8_t error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    if (packetHandler->ping(portHandler, id, &error) == COMM_SUCCESS)
      printf("Pinged ID: %03d successfully.\n", id);
    else
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      ping_successful = false;
    }
  }
  return ping_successful;
}

/// @brief Convert a raw Position register value into radians
/// @param value - desired raw value to convert
/// @return <float> - value in radians
float convertValue2Radian(int32_t value)
{
  return (value - 2047.5)/2047.5 * PI;
}

/// @brief Convert a value in radians to the raw Position register value
/// @param radian - desired value in radians to convert
/// @return <int32_t> - raw register value
int32_t convertRadian2Value(float radian)
{
  return round(radian/PI * 2047.5 + 2047.5);
}

int main()
{
  // WX200 ARM EEPROM CONFIGS
  std::vector<uint8_t> all_ids = {1,2,3,4,5,6,7};                               // Note that each index in the following vectors correspond to the ID in the matching index in the 'all_ids' vector
  std::vector<uint8_t> drive_modes = {4,4,5,5,5,4,4};                           // '4' means CCW is positive and Time-Based profile is being used; '5' means CCW is negative and Time-Based profile is being used
  std::vector<uint8_t> shadow_ids = {255,255,2,255,255,255,255};                // '255' means shadow id is disabled; '0 - 252' means follow the motor with the specified ID
  std::vector<uint32_t> min_position_limits = {0,0,0,0,0,0,0};
  std::vector<uint32_t> max_position_limits = {4095,4095,4095,4095,4095,4095,4095};
  std::vector<uint8_t> operating_modes = {3,3,3,3,3,3,16};                      // '3' is position mode while '16' is PWM mode
  std::vector<uint16_t> position_p_gains = {800, 1500, 2000, 1000, 640};        // Proportional Gain in the internal PID control loop in each motor

  // WX200 ARM 'Command' IDs
  std::vector<uint8_t> arm_ids = {1,2,4,5,6};                                   // WX200 Arm Motor IDs (excludes slaves since they will automatically follow the masters and excludes grippers)
  uint8_t gripper_id = 7;                                                       // WX200 Gripper Motor ID
  uint8_t shoulder_master_id = 2;                                               // WX200 Shoulder Master ID (in dual-joint setup)
  uint8_t shoulder_slave_id = 3;                                                // WX200 Shoulder Slave ID (in dual-joint setup)

  // Initialize the port, ping the motors, and create syncWrite and syncRead objects
  // It's faster and better design to read/write motors with the 'sync' objects than to command each motor sequentially
  // Commanding each motor sequentially should only be done for 'non-realtime sensitive' registers - like torquing on/off, setting EEPROM registers, etc...
  if (!initPort("/dev/ttyDXL", 1000000))
    return 1;
  if (!ping(all_ids))
    return 1;
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);

  // Initialize register values
  itemWriteMultiple(&all_ids, ADDR_TORQUE_ENABLE, (uint8_t)0);                  // torque off all motors so that we can write to the EEPROM registers
  itemWriteMultiple(&all_ids, ADDR_DRIVE_MODE, &drive_modes);                   // set the Drive_Mode register for all motors
  itemWriteMultiple(&all_ids, ADDR_SECONDARY_ID, &shadow_ids);                  // set the Shadow ID register for all motors
  itemWriteMultiple(&all_ids, ADDR_MIN_POS_LIMIT, &min_position_limits);        // set the Min Position Limit for all motors
  itemWriteMultiple(&all_ids, ADDR_MAX_POS_LIMIT, &max_position_limits);        // set the Max Position Limit for all motors
  itemWriteMultiple(&all_ids, ADDR_OPERATING_MODE, &operating_modes);           // set the Operating Mode register for all motors
  itemWriteMultiple(&arm_ids, ADDR_POSITION_P_GAIN, &position_p_gains);         // raise the internal Position P Gain in the motors' PID loops
  itemWriteMultiple(&arm_ids, ADDR_PROFILE_VELOCITY, (uint32_t)1500);           // set the Profile Velocity register for all arm motors to 1500ms (each movement takes 1.5 second)
  itemWriteMultiple(&arm_ids, ADDR_PROFILE_ACCELERATION, (uint32_t)750);        // set the Profile Acceleration register for all arm motors to 750ms (each movement should take 0.75 sec to acclerate and 0.75 sec to decelerate)
  calibrateDualJoint(shoulder_master_id, shoulder_slave_id);                    // Calibrate the Homing Offset Register in the Slave motor until its Present Position register is equivalent to the Master's
  itemWriteMultiple(&all_ids, ADDR_TORQUE_ENABLE, (uint8_t)1);                  // torque all motors on

  // Read current arm joint positions
  std::vector<int32_t> positions;
  syncRead(&groupSyncRead, &arm_ids, ADDR_PRESENT_POSITION, &positions);
  for (size_t i{0}; i < arm_ids.size(); i++)
    printf("Current Position of ID: %03d is: %4d ticks or %6.3f radians.\n", arm_ids.at(i), positions.at(i), convertValue2Radian(positions.at(i)));

  // Write home and sleep positions to the arm joints in sequence
  std::vector<int32_t> home_positions(5,2048);
  std::vector<int32_t> sleep_positions = {2048, 803, 3099, 2570, 2048};
  syncWrite(&groupSyncWrite, &arm_ids, &home_positions);
  sleep(2);
  syncWrite(&groupSyncWrite, &arm_ids, &sleep_positions);
  sleep(2);

  // Command the gripper to open for 2 seconds, then close for 2 seconds
  itemWrite(gripper_id, ADDR_GOAL_PWM, (int16_t)350);
  sleep(2);
  itemWrite(gripper_id, ADDR_GOAL_PWM, (int16_t)-350);
  sleep(2);
  itemWrite(gripper_id, ADDR_GOAL_PWM, (int16_t)0);

  // Read the present temperature of all the motors
  std::vector<uint8_t> temps;
  itemReadMultiple(&all_ids, ADDR_PRESENT_TEMP, &temps);
  for (size_t i{0}; i < all_ids.size(); i++)
    printf("ID: %03d Temperature is: %d degrees Celsius.\n", all_ids.at(i), temps.at(i));

  // Read the present loads/currents of all motors
  std::vector<int16_t> loads;
  itemReadMultiple(&all_ids, ADDR_PRESENT_LOAD, &loads);
  for (size_t i{0}; i < all_ids.size(); i++)
    printf("ID: %03d Raw Load/Current is: %d.\n", all_ids.at(i), loads.at(i));

  portHandler->closePort();
  return 0;
}
