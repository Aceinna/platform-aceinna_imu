/** ***************************************************************************
 * @file   commandTable.c table of commands, calbacks, and help strings
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  Table of commands available to be sent from the commandLine.c shell
 ******************************************************************************/
#include "commandLine.h"
#include "commands.h"

#define COMMAND_TABLE_END {"",0,0,""}

//Command table
// {char *name, tShellCallback callback, uint32_t callbackData, const *help }
const tCommand gCommands[] =
{
  { "ver",    &CmdVersion, 0, "Display firmware version"},

  { "blink", &CmdBlinkLED, 0, "Blink LED4 <numBlinks, 3> <ms between blinks, 250>" },

  { "dispdata", &CmdDisplayDebugMessages_SensorData, 0, "Command line input to display (1) or suppress (0) streaming sensor data over USART <DisplayDataFlag, 1>" },

  { "termbaud", &CmdUsartBaudRate, 0, "Set the USART baud rate <default, 1: 460800>" \
                                      "\r\n\t Other options --- 0: 921600" \
                                      "\r\n\t                   1: 460800" \
                                      "\r\n\t                   2: 230400" \
                                      "\r\n\t                   3: 115200" \
                                      "\r\n\t                   4: 57600>" \
                                      "\r\n\t                   5: 38400>" \
                                      "\r\n\t                   6: 19200>" },

  { "auto", &CmdAutoDataAquisitionMode, 0,
    "System will go into constant output mode, no parameters" },

  { "sample", &CmdChangeOutputDataRate, 0,
    "Change the output data rate, this modifies the timer as well as the gyro."},

  {"agm",    &CmdReadSensors, 0x00,
    "Read accel, gyro, mag sensor counts <num reads,  10> <ms between reads, asap> " \
        "\r\n\t\tNote: mags only updated as they are available and gyro used is " \
        "\r\n\t the one that was last initialized."},
  {"agmf",    &CmdReadSensors, 0x01,
    "Read accel (Gs),  gyro (dps), mag (Ga) <num reads,  10> <ms between reads, asap> "
        "\r\n\t\tNote: mags only updated as they are available and gyro used is " \
        "\r\n\t the one that was last initialized."},

  // Accelerometer commands
  {"ai",    &CmdAccelInit, 0,
     "Initialize the accel, <output range in Gs> "},
  {"a",    &CmdReadAccelerometer, 0,
    "Read accelerometer <num reads,  10> <ms between reads, asap>"},
  {"af",    &CmdReadAccelerometer, 1,
    "Read accelerometer in Gs <num reads,  10> <ms between reads, asap>"},
  {"ab",    &CmdApplyAccelSensorBias, 0,
     "Apply or remove the accelerometer bias in order to perform a self-test, <apply bias (1)/remove bias (0)>"},
  {"ast",    &CmdPerformAccelSelfTest, 0,
     "Perform an accelerometer self-test, <apply bias (1)/remove bias (0)>"},

  {"t",    &CmdReadTemperature, 0,
    "Read temperature <num reads, 10> <ms between reads, asap>"},
  {"tf",    &CmdReadTemperature, 1,
    "Read temperature in C <num reads, 10> <ms between reads, asap>"},

  // Maxim rate sensor commands
  {"gi",    &CmdGyroInit, 0,
     "Initialize the gyro, <output range in dps> <output data rate>"},
  {"g",    &CmdReadGyro, 0,
    "Read gryo <num reads, 10> <ms between reads, asap>"},
  {"gf",    &CmdReadGyro, 1,
    "Read gyro in degrees per sec <num reads, 10> <ms between reads, asap>"},
  {"gtemp", &CmdReadGyroTemp, 0,
    "Read gryo temperature <num reads, 10> <ms between reads, asap>"},
  {"gtempf", &CmdReadGyroTemp, 1,
    "Read gyro temp in C <num reads, 10> <ms between reads, asap>"},
  {"gb",    &CmdApplyRateSensorBias, 0,
     "Apply or remove the rate-sensor bias in order to perform a self-test, <apply bias (1)/remove bias (0)>"},

  // Magnetometer commands
  {"mi",    &CmdMagnetometerInit, 0,
    "Initialize the mag, <output range in milligauss> (Note: no odr, it goes as fast as possible.)"},
  {"m",    &CmdReadMagnetometer, 0,
    "Read magnetometer <num reads, default 10> <ms between reads, default asap>"},
  {"mf",    &CmdReadMagnetometer, 1,
    "Read magnetometer in Gauss <num reads, default 10> <ms between reads, default asap>"},

  // Inertial calibration output commands
  { "iCal",  &CmdInertialCalib, 0x00,
    "Inertial calibration: Read accel, gyro, gyro temperature, temperature sensor counts <num reads,  10> <ms between reads, asap> " \
        "\r\n\t\tNote: the gyro used is the one that was last initialized."},
  { "iCalf",  &CmdInertialCalib, 0x01,
    "Inertial calibration: Read accel, gyro, gyro temperature, temperature sensor counts <num reads,  10> <ms between reads, asap> " \
        "\r\n\t\tNote: the gyro used is the one that was last initialized."},

  // Temperature calibration output commands
  { "tCal",  &CmdInertialCalib, 0x10,
    "Temperature calibration: Read accel, gyro, magnetometer, gyro temperature sensor, " \
    "\r\n\t\tand board temperature sensor counts <num reads,  10> <ms between reads, asap> " \
        "\r\n\t\tNote: mags only updated as they are available and gyro used is " \
        "\r\n\t the one that was last initialized."},
   { "tCalf",  &CmdInertialCalib, 0x11,
    "Temperature calibration: Read accel, gyro, magnetometer, gyro temperature sensor, " \
    "\r\n\t\tand board temperature sensor counts <num reads,  10> <ms between reads, asap> " \
        "\r\n\t\tNote: mags only updated as they are available and gyro used is " \
        "\r\n\t the one that was last initialized."},

  // EEPROM write/read commands
  { "eeWrite", &CmdEeWrite, 0,
    "Write to the EEPROM using <xbow addres> <num bytes> <data in dec> "},
  { "eeRead", &CmdEeRead, 0,
    "Read from the EEPROM using <xbow addres> <num bytes>"},

  { "output", &CmdUserUsart, 0,
    "Print characters to the user uart <which> <character string>"},

  { "pin", &CmdGpioPin, 0,
    "Set pin <port> <pin> to <state>"},
#if 0
  {"g",    &CmdReadGyro, 0,
    "Read gyro <num reads, default 10> <ms between reads, default asap>"},

#endif // 0
  // mfg test style functions
  {"swtest", &CmdSelfTest, 0, "Run self test, verify existence of each sensor."},

  { "initSpi",     &CmdInitSPIPeripheral, 0, "Initialize the SPI peripheral" },
  { "gps",         &CmdParseGPS, 0, "Parse <GPS message>, show GPS data structure" },
  { "initGps",     &CmdGpsInit, 0, "Init GPS message handling" },
  { "internalGPS", &CmdGpsInternal, 0, "Set production bit, default 1, w/param to 0" },
  { "readGPS",     &CmdGpsRead, 0, "Read current GPS value" },
  { "handleGps",   &CmdGpsHandler, 0, "Run GPS handler for <num seconds>" },


  COMMAND_TABLE_END  //MUST BE LAST!!!
};

