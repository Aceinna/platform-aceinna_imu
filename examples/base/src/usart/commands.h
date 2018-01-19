
#ifndef COMMANDS_H
#define _COMMANDS_H

void CmdVersion(uint32_t data);

void CmdBlinkLED( uint32_t data );
void CmdDisplayDebugMessages_SensorData( uint32_t data );

// FIXME: remove after faster RS232 chips become available
void CmdUsartBaudRate( uint32_t data );

//static uint8_t _WriteSPI3Register(uint8_t address, uint8_t data);

void CmdWriteSingleSpiByte( uint32_t data );
void CmdInitSPIPeripheral( uint32_t data );

void CmdReadSensors(uint32_t data);

void CmdAccelInit(uint32_t data);
void CmdReadAccelerometer(uint32_t data);
void CmdApplyAccelSensorBias(uint32_t data);
void CmdPerformAccelSelfTest(uint32_t data);

void CmdReadTemperature(uint32_t data);
void CmdMagnetometerInit(uint32_t data);
void CmdReadMagnetometer(uint32_t data);

void CmdGyroInit(uint32_t data);
void CmdReadGyro(uint32_t data);
void CmdReadGyroTemp(uint32_t data);
void CmdApplyRateSensorBias(uint32_t data);
void CmdGyroSelfTest(uint32_t data);

void CmdSelfTest(uint32_t data);

void CmdInertialCalib(uint32_t data);
void CmdTemperatureCalib(uint32_t data);

void CmdAutoDataAquisitionMode(uint32_t data);
void CmdChangeOutputDataRate(uint32_t data);

void CmdUserUsart(uint32_t data);
void CmdGpioPin(uint32_t data);
void CmdEeRead(uint32_t data);
void CmdEeWrite(uint32_t data);

void CmdParseGPS(uint32_t data);
void CmdGpsInit(uint32_t data);
void CmdGpsHandler(uint32_t data);
void CmdGpsInternal(uint32_t data);
void CmdGpsRead(uint32_t data);


#endif /* COMMANDS_H */