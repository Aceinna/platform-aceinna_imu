/** ***************************************************************************
 * @file send_packet.c UCB callbacks for assembling then sending serial packets
 * to NAV-VIEW.
 * @brief add message to be handled to send_packet() including callback name
 *        then add callback fcn to load the data and send the message.
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief this is the serial send handler for the Nav View data link
 *****************************************************************************/
#include "stm32f2xx.h"
#include "dmu.h"
#include "ucb_packet.h"
#include "timer.h"
#include "extern_port.h"
#include "xbowsp_version.h"
#include "xbowsp_init.h"
#include "xbowsp_fields.h"
#include "scaling.h"
#include "calibrate.h"
#include "calc_airData.h"
#include "timer.h"
#include "EKF_Algorithm.h"
#include "qmath.h"
#include "UserCommunication_SPI.h" // SPI register definitions
#include "taskUserCommunication.h" // user com type
#include "driverGPS.h"

void _UcbIdentification(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbVersionData(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbVersionAllData(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbAngle1(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbAngle2(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbAngle5(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbAngleU(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbScaled0(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbScaled1(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbTest0(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbTest1(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbFactory1(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbFactory2(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbFactory3(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbMagCal1Complete(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbMagCal3Complete(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbNav0(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbNav1(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
void _UcbNav2(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);


/** ****************************************************************************
 * @name _UcbIdentification send ID packet
 * @brief
 * Trace: [SDD_UCB_TX_ID <-- SRC_UCB_TX_ID]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbIdentification (ExternPortTypeEnum port,
                         UcbPacketStruct    *ptrUcbPacket)

{
    uint8_t packetIndex = 0;
    uint8_t stringIndex = 0;

    const uint8_t PART_NUMBER_STRING [] = SOFTWARE_PART;

    /// serial number
    packetIndex = uint32ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 gCalibration.serialNumber);
	/// model string
	while ((stringIndex < N_VERSION_STR) &&
		   (gCalibration.versionString[stringIndex] != 0))
	{
		ptrUcbPacket->payload[packetIndex++] = (uint8_t)gCalibration.versionString[stringIndex++];
	}

	/// space between
	ptrUcbPacket->payload[packetIndex++] = ' ';
	stringIndex = 0;

	/// software part number
	while (stringIndex < SOFTWARE_PART_LEN) {
		ptrUcbPacket->payload[packetIndex++] = (uint8_t)PART_NUMBER_STRING[stringIndex++];
	}

	ptrUcbPacket->payload[packetIndex++] = 0;  ///< zero delimiter
	ptrUcbPacket->payloadLength          = packetIndex; ///< return packet length
	HandleUcbTx(port, ptrUcbPacket); ///< send identification packet
}

/** ****************************************************************************
 * @name _UcbVersionData send VR packet
 * @brief
 * Trace: [SDD_UCB_TX_VR <-- SRC_UCB_TX_VR]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbVersionData (ExternPortTypeEnum port,
                      UcbPacketStruct    *ptrUcbPacket)
{
	/// return packet length
	ptrUcbPacket->payloadLength = UCB_VERSION_DATA_LENGTH;

    /// 525 digital processor DUP version data - append
	ptrUcbPacket->payload[0] = (uint8_t)dupFMversion.major;
	ptrUcbPacket->payload[1] = (uint8_t)dupFMversion.minor;
	ptrUcbPacket->payload[2] = (uint8_t)dupFMversion.patch;
	ptrUcbPacket->payload[3] = (uint8_t)dupFMversion.stage;
	ptrUcbPacket->payload[4] = (uint8_t)dupFMversion.build;
	HandleUcbTx(port, ptrUcbPacket); /// send version data packet
}

/** ****************************************************************************
 * @name _UcbVersionAllData send VA (version all) packet
 * @brief
 * Trace: [SDD_UCB_TX_VA <-- SRC_UCB_TX_VA]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbVersionAllData (ExternPortTypeEnum port,
                         UcbPacketStruct    *ptrUcbPacket)
{
    uint8_t packetIndex = 0;

    // DUP IOUP are here to allow compatibility with NavView
	/// 525 digital processor DUP version data
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)dupFMversion.major;
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)dupFMversion.minor;
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)dupFMversion.patch;
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)dupFMversion.stage;
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)dupFMversion.build;

    /// 525 input output processor IOUP version data
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)ioupFMversion.major;
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)ioupFMversion.minor;
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)ioupFMversion.patch;
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)ioupFMversion.stage;
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)ioupFMversion.build;

    /// boot version data
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)bootFMversion.major;
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)bootFMversion.minor;
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)bootFMversion.patch;
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)bootFMversion.stage;
	ptrUcbPacket->payload[packetIndex++] = (uint8_t)bootFMversion.build;

	ptrUcbPacket->payloadLength = packetIndex; ///< return packet length
	HandleUcbTx(port, ptrUcbPacket); 	///< send version all data packet
}

/** ****************************************************************************
 * @name _UcbAngle1 send A1 packet (typical output packet used in 525
 *       configuration)
 * @brief load (SPI/UART) and send (UART) Angle 1 message
 * Trace: [SDD_UCB_TX_A1 <-- SRC_UCB_TX_A1]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbAngle1 (ExternPortTypeEnum port,
                 UcbPacketStruct   *ptrUcbPacket)
{
	uint16_t  packetIndex = 0;
    uint32_t  time        = TimeNow();
	Crc32Type payloadCrc;

	ptrUcbPacket->payloadLength = UCB_ANGLE_1_LENGTH; /// set packet length
    // xbowsp_fields.c
	/// roll angle, pitch angle, magnetometer yaw angle
	packetIndex = appendAttitudeTrue(ptrUcbPacket->payload, packetIndex);

    // Append raw or EKF derived data into the packet.  If sest to zero then the
    //   raw data is provided (FIXME: remove before release).
    if( gConfiguration.analogFilterClocks[0] & 0x8000 ) {
        /// X-angular rate, Y, Z
        packetIndex = appendRates(ptrUcbPacket->payload, packetIndex);

        /// X-accelerometer, Y, Z
        packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);
    } else {
        /// X-angular rate, Y, Z
        packetIndex = appendCorrectedRates(ptrUcbPacket->payload, packetIndex);

        /// X-accelerometer, Y, Z (according to the serial-interface spec, the
        ///   accelerometer signal is uncorrected).
//        packetIndex = appendCorrectedAccels(ptrUcbPacket->payload, packetIndex);
        packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);
    }

    /// magnetometer X, Y, Z
    packetIndex = appendMagReadings(ptrUcbPacket->payload, packetIndex);

    /// X rate temp
    packetIndex = appendRateTemp(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint32ToBuffer(ptrUcbPacket->payload, /// timer
                                 packetIndex,
                                 gAlgorithm.timer);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  gAlgorithm.bitStatus.BITStatus.all );
	/// compute Universal payload CRC
	payloadCrc = Crc32(ptrUcbPacket->payload,
                       (UCB_ANGLE_U_LENGTH - CRC_32_LENGTH),
                       CRC_32_INITIAL_SEED);
	/// Universal payload CRC
	Crc32TypeToBytes (payloadCrc, &(ptrUcbPacket->payload[packetIndex]));

    if( getUserCommunicationType() == UART_COMM ) {
        HandleUcbTx(port, ptrUcbPacket); /// send Angle 1 packet
    } // load only for SPI don't send
}


/** ****************************************************************************
 * @name _UcbAngle2 send A2 packet (typical output packet used in 525
 *       configuration)
 * @brief
 * Trace: [SDD_UCB_TX_A2 <-- SRC_UCB_TX_A2]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbAngle2 (ExternPortTypeEnum port,
                 UcbPacketStruct   *ptrUcbPacket)
{
	uint16_t  packetIndex = 0;
    uint32_t  time        = TimeNow();
	Crc32Type payloadCrc;

	ptrUcbPacket->payloadLength = UCB_ANGLE_2_LENGTH; /// set packet length
    // xbowsp_fields.c
	/// roll angle, pitch angle, magnetometer yaw angle
	packetIndex = appendAttitudeTrue(ptrUcbPacket->payload, packetIndex);
	/// X-angular rate, Y, Z
	packetIndex = appendCorrectedRates(ptrUcbPacket->payload, packetIndex);
	/// X-accelerometer, Y, Z
	packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);

    /// X,Y,Z rate temp (do not include board temp per Serial Interface Spec, i.e. do not
    ///   use appendTemps)
    packetIndex = appendRateTemp(ptrUcbPacket->payload, packetIndex);
    packetIndex = appendRateTemp(ptrUcbPacket->payload, packetIndex);
    packetIndex = appendRateTemp(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint32ToBuffer(ptrUcbPacket->payload, /// timer
                                 packetIndex,
                                 gAlgorithm.timer);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  gAlgorithm.bitStatus.BITStatus.all );
	/// compute Universal payload CRC
	payloadCrc = Crc32(ptrUcbPacket->payload,
                       (UCB_ANGLE_U_LENGTH - CRC_32_LENGTH),
                       CRC_32_INITIAL_SEED);
	/// Universal payload CRC
	Crc32TypeToBytes (payloadCrc, &(ptrUcbPacket->payload[packetIndex]));
	HandleUcbTx(port, ptrUcbPacket); /// send Angle 2 packet
}

/** ****************************************************************************
 * @name _UcbAngle4 send modified A4 DEBUG packet
 * @brief Used to load SPI message payload NO INFRASTRUCTURE CONNECTED FOR UCB
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
/* todo tm20160602 - need to fix this, if we support this packet
void _UcbAngle4 (ExternPortTypeEnum port,
                 UcbPacketStruct    *ptrUcbPacket)
{
	uint16_t packetIndex = 0;
    int16_t tmp;

	// set packet length
	ptrUcbPacket->payloadLength = UCB_ANGLE_4_LENGTH;

	// add roll angle, pitch angle, magnetometer yaw angle
	packetIndex = appendAttitudeTrue((uint8_t *)ptrUcbPacket->payload, packetIndex);

	// add X-angular rate, Y-angular rate, Z-angular rate
	packetIndex = appendCorrectedRates((uint8_t *)ptrUcbPacket->payload, packetIndex);
    //packetIndex = appendRates((char *)packetPtr.ucbPacketPtr->payload, packetIndex);

	// add X-accelerometer, Y-accelerometer, Z-accelerometer
	packetIndex = appendAccels((uint8_t *)ptrUcbPacket->payload, packetIndex);

	/// X-magnetometer, Y, Z
	packetIndex = appendMagReadings(ptrUcbPacket->payload, packetIndex);

    // X-Rate Temperature
    tmp = _qmul( TWO_POW16_OVER_128_q21, gAlgorithm.scaledSensors_q27[XRTEMP+0], 21, 27, 16 ) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );

    // Y-Rate Temperature
    tmp = _qmul( TWO_POW16_OVER_2_q15, gKalmanFilter.Quaternion_q30[1], 15, 30, 16 ) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );
    // Z-Rate Temperature
    tmp = _qmul( TWO_POW16_OVER_2_q15, gKalmanFilter.Quaternion_q30[2], 15, 30, 16 ) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );
    // mag Temperature
    tmp = _qmul( TWO_POW16_OVER_2_q15, gKalmanFilter.Quaternion_q30[3], 15, 30, 16 ) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );

    /// time ITOW X-angular, Y, Z rate
	packetIndex = appendRates(ptrUcbPacket->payload, packetIndex);

	/// add timer
    packetIndex = uint32ToBuffer(ptrUcbPacket->payload, /// timer
                             packetIndex,
                             gAlgorithm.timer);

    // NO XMIT: this is to load buffer for SPI
//	HandleUcbTx(port, ptrUcbPacket); /// send Angle U packet
} */

/** ****************************************************************************
 * @name _UcbAngle5 send A5 DEBUG packet
 * @brief Borrowing messages sent to NavView
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
/* todo tm20160602 - need to fix this, if we support this packet
void _UcbAngle5 (ExternPortTypeEnum port,
                 UcbPacketStruct    *ptrUcbPacket)
{
	uint16_t packetIndex = 0;
    int16_t tmp;

	Crc32Type payloadCrc;

	// set packet length
	ptrUcbPacket->payloadLength = UCB_ANGLE_5_LENGTH;

	// add roll angle, pitch angle, magnetometer yaw angle
	packetIndex = appendAttitudeTrue((uint8_t *)ptrUcbPacket->payload, packetIndex);

	// add X-angular rate, Y-angular rate, Z-angular rate
	packetIndex = appendCorrectedRates((uint8_t *)ptrUcbPacket->payload, packetIndex);
    //packetIndex = appendRates((char *)packetPtr.ucbPacketPtr->payload, packetIndex);

	// add X-accelerometer, Y-accelerometer, Z-accelerometer
	packetIndex = appendAccels((uint8_t *)ptrUcbPacket->payload, packetIndex);

    // X-Rate Temperature
    tmp = _qmul( TWO_POW16_OVER_128_q21, gAlgorithm.scaledSensors_q27[XRTEMP+0], 21, 27, 16 ) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );
    // prediction covariance
    tmp = _qmul( TWO_POW16_OVER_128_q21, doubleToQ30( gKalmanFilter.P[1][6] ), 21, 30, 16 ) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );
   //
    packetIndex = uint32ToBuffer(ptrUcbPacket->payload,
                 packetIndex,
                 0x000000001);

    // more covariance
    tmp = _qmul( TWO_POW16_OVER_512_q23, doubleToQ30( gKalmanFilter.P[1][6]*10000 ), 23, 30, 16 ) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );

	// add X-magnetometer, Y-magnetometer, Z-magnetometer
	packetIndex = appendMagReadings((uint8_t *)ptrUcbPacket->payload, packetIndex);

    // X-Rate Temperature
    tmp = _qmul( TWO_POW16_OVER_7PI_q19, gKalmanFilter.rateBias_q27[X_AXIS] << 5, 19, 27, 16) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );
    // X-Rate Temperature
    tmp = _qmul( TWO_POW16_OVER_7PI_q19, gKalmanFilter.rateBias_q27[Y_AXIS] << 5, 19, 27, 16) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );
    // X-Rate Temperature
    tmp = _qmul( TWO_POW16_OVER_7PI_q19, gKalmanFilter.rateBias_q27[Z_AXIS] << 5, 19, 27, 16) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );
    // X-Accel Temperature
    tmp = _qmul( TWO_POW16_OVER_2_q15, gKalmanFilter.Quaternion_q30[1], 15, 30, 16 ) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );
    // X-Rate Temperature
    tmp = _qmul( TWO_POW16_OVER_2_q15, gKalmanFilter.Quaternion_q30[2], 15, 30, 16 ) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );
    // X-Rate Temperature
    tmp = _qmul( TWO_POW16_OVER_2_q15, gKalmanFilter.Quaternion_q30[3], 15, 30, 16 ) >> 16;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              tmp );
//GOOD TO HERE

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              0x0001 );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                              packetIndex,
                              0x0002 );

    packetIndex = uint32ToBuffer(ptrUcbPacket->payload,
                     packetIndex,
                     (uint16_t)gKalmanFilter.P[6][6]);

    // X-Rate Temperature
    tmp = _qmul( TWO_POW16_OVER_20_q19, gAlgorithm.scaledSensors_q27[XRTEMP+0], 19, 27, 15 ) >> 15;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                              packetIndex,
                              tmp );
	/// add timer
    packetIndex = uint32ToBuffer(ptrUcbPacket->payload, /// timer
                         packetIndex,
                         gAlgorithm.timer);

	/// add BIT status
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  gAlgorithm.bitStatus.BITStatus.all );
    /// Universal payload CRC
    payloadCrc = Crc32(ptrUcbPacket->payload,
                       (UCB_ANGLE_5_LENGTH - CRC_32_LENGTH),
                       CRC_32_INITIAL_SEED);
	Crc32TypeToBytes (payloadCrc, &(ptrUcbPacket->payload[packetIndex]));
	/// send Angle 5 packet
	HandleUcbTx(port, ptrUcbPacket); /// send Angle U packet
}
*/

/** ****************************************************************************
 * @name _UcbAngleU send AU packet (typical output packet used in 525
 *       configuration)
 * @brief
 * Trace: [SDD_UCB_TX_AU <-- SRC_UCB_TX_AU]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbAngleU (ExternPortTypeEnum port,
                 UcbPacketStruct   *ptrUcbPacket)
{
	uint16_t  packetIndex = 0;
    uint32_t  time        = TimeNow();
	Crc32Type payloadCrc;

	ptrUcbPacket->payloadLength = UCB_ANGLE_U_LENGTH; /// set packet length
    // xbowsp_fields.c
	/// roll angle, pitch angle, magnetometer yaw angle
	packetIndex = appendAttitudeTrue(ptrUcbPacket->payload, packetIndex);
	/// X-angular rate, Y, Z
	packetIndex = appendCorrectedRates(ptrUcbPacket->payload, packetIndex);
	/// X-accelerometer, Y, Z
	packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);
	/// tangent X-rate, Y, Z
	packetIndex = appendTangentRates(ptrUcbPacket->payload, packetIndex);
	/// tangent X-accel, Y, Z
	packetIndex = appendTangentAccels(ptrUcbPacket->payload, packetIndex);

	/// compass heading
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 ( (int)(SCALE_BY_2POW16_OVER_2PI(gAlgorithm.compassHeading))) );

    packetIndex = uint32ToBuffer(ptrUcbPacket->payload, /// timer
                                 packetIndex,
                                 gAlgorithm.timer);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  gAlgorithm.bitStatus.BITStatus.all );
	/// compute Universal payload CRC
	payloadCrc = Crc32(ptrUcbPacket->payload,
                       (UCB_ANGLE_U_LENGTH - CRC_32_LENGTH),
                       CRC_32_INITIAL_SEED);
	/// Universal payload CRC
	Crc32TypeToBytes (payloadCrc, &(ptrUcbPacket->payload[packetIndex]));
	HandleUcbTx(port, ptrUcbPacket); /// send Angle U packet
}

/** ****************************************************************************
 * @name _UcbScaled0 send S0 packet
 * @brief Scaled sensor 0 message load (SPI / UART) send (UART) scaled and
 *        filtered sensor data
 * Trace: [SDD_UCB_TX_S3 <-- SRC_UCB_TX_S3]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbScaled0 (ExternPortTypeEnum port,
                  UcbPacketStruct    *ptrUcbPacket)
{
	uint16_t packetIndex = 0;

	/// set packet length
	ptrUcbPacket->payloadLength = UCB_SCALED_0_LENGTH;
    /// X-accelerometer, Y, Z
	packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);
	/// X-angular, Y, Z rate
	packetIndex = appendRates(ptrUcbPacket->payload, packetIndex);
	/// X-magnetometer, Y, Z
	packetIndex = appendMagReadings(ptrUcbPacket->payload, packetIndex);
	/// rate and board temperature
	packetIndex = appendTemps(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, // itow???
                                  packetIndex,
                                  gAlgorithm.counter );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  gAlgorithm.bitStatus.BITStatus.all );

	++gAlgorithm.counter; ///< packet counter

    if( getUserCommunicationType() == UART_COMM ) {
        HandleUcbTx(port, ptrUcbPacket); /// send Scaled 0 packet
    }
}

/** ****************************************************************************
 * @name _UcbScaled1 send S1 packet
 * @brief Sclaed sensor 1 load (SPI / UART) send (UART) filtered and scaled data
 * Trace: [SDD_UCB_TX_S1 <-- SRC_UCB_TX_S1]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbScaled1 (ExternPortTypeEnum port,
                  UcbPacketStruct    *ptrUcbPacket)
{
	uint16_t packetIndex = 0;

	ptrUcbPacket->payloadLength = UCB_SCALED_1_LENGTH;
    /// X-accelerometer, Y, Z
	packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);
	/// X-angular rate, Y, Z
	packetIndex = appendRates(ptrUcbPacket->payload, packetIndex);
#ifdef RUN_PROFILING

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                           packetIndex,
                           (uint16_t)SCALE_BY_2POW16_OVER_200(gEkfElapsedTime));
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                           packetIndex,
                           (uint16_t)SCALE_BY_2POW16_OVER_200(gEkfAvgTime));
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                           packetIndex,
                           (uint16_t)SCALE_BY_2POW16_OVER_200(gEkfMaxTime));

#else
	/// rate and board temperature
	packetIndex = appendTemps(ptrUcbPacket->payload, packetIndex);
#endif

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// packet counter
                                 packetIndex,
                                 gAlgorithm.counter );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                 packetIndex,
                                 gAlgorithm.bitStatus.BITStatus.all );

    if( getUserCommunicationType() == UART_COMM ) {
        HandleUcbTx(port, ptrUcbPacket); /// send Scaled 1 packet
    }
}

/** ****************************************************************************
 * @name _UcbTest0 send T0 packet
 * @brief
 * Trace: [SDD_UCB_TX_T0 <-- SRC_UCB_TX_T0]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbTest0 (ExternPortTypeEnum port,
                UcbPacketStruct    *ptrUcbPacket)
{
	uint8_t packetIndex = 0;

	/// set packet length
	ptrUcbPacket->payloadLength = UCB_TEST_0_LENGTH;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.BITStatus.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.hwBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware power BIT
                                 packetIndex,
                                 0X0000 ); // Place holder for NavView
	/// hardware environmental BIT
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                                  packetIndex,
                                  gAlgorithm.bitStatus.hwEnvBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.comBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com serial A BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.comSABIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com serial b BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.comSBBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.swBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software algorithm BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.swAlgBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software data BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.swDataBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware status
                                 packetIndex,
                                 gAlgorithm.bitStatus.hwStatus.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com status
                                 packetIndex,
                                 gAlgorithm.bitStatus.comStatus.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software status
                                 packetIndex,
                                 gAlgorithm.bitStatus.swStatus.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// SENSOR status
                                 packetIndex,
                                 gAlgorithm.bitStatus.sensorStatus.all );
	HandleUcbTx(port, ptrUcbPacket); /// send Test 0 packet
}

/** ****************************************************************************
 * @name _UcbTest1 send T1 packet
 * @brief
 * Trace: [SDD_UCB_TX_T1 <-- SRC_UCB_TX_T1]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbTest1 (ExternPortTypeEnum port,
                UcbPacketStruct    *ptrUcbPacket)
{
	uint8_t packetIndex = 0;

	/// set packet length
	ptrUcbPacket->payloadLength = UCB_TEST_1_LENGTH;

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.BITStatus.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.hwBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware power BIT
                                 packetIndex,
                                 0X0000 ); // Place holder for NavView

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware environmental BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.hwEnvBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware sensor BIT
                                 packetIndex,
                                 0x0000 ); // Place holder for NavView

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware internal comm BIT
                                 packetIndex,
                                 0x0000 ); // Place holder for NavView

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// comm BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.comBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// comm serial A BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.comSABIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// comm serial B BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.comSBBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.swBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software algorithm  BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.swAlgBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software data BIT
                                 packetIndex,
                                 gAlgorithm.bitStatus.swDataBIT.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware status
                                 packetIndex,
                                 gAlgorithm.bitStatus.hwStatus.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com status
                                 packetIndex,
                                 gAlgorithm.bitStatus.comStatus.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software status
                                 packetIndex,
                                 gAlgorithm.bitStatus.swStatus.all );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// sensor status
                                 packetIndex,
                                 gAlgorithm.bitStatus.sensorStatus.all );
	HandleUcbTx(port, ptrUcbPacket); /// send Test 1 packet
}

/** ****************************************************************************
 * @name _UcbFactory1 send F1 packet Factory (Raw) sensor data
 * @brief Raw data 1 load (SPI / UART) and send (UART) raw sensor counts
 * Trace: [SDD_UCB_TX_F1 <-- SRC_UCB_TX_F1]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbFactory1 (ExternPortTypeEnum port,
                   UcbPacketStruct    *ptrUcbPacket)
{
	uint16_t packetIndex = 0;

	ptrUcbPacket->payloadLength = UCB_FACTORY_1_LENGTH;
	packetIndex = appendInertialCounts(ptrUcbPacket->payload, packetIndex);
	packetIndex = appendAllTempCounts(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  gAlgorithm.bitStatus.BITStatus.all );

    if( getUserCommunicationType() == UART_COMM ) {
        HandleUcbTx(port, ptrUcbPacket); /// send Factory 1 packet
    }
}

/** ****************************************************************************
 * @name _UcbFactory2 send F2 packet Factory (Raw) sensor data
 * @brief Raw data 2 load (SPI / UART) and send (UART) raw sensor counts
 * Trace: [SDD_UCB_TX_F2 <-- SRC_UCB_TX_F2]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbFactory2 (ExternPortTypeEnum port,
                   UcbPacketStruct    *ptrUcbPacket)
{
	uint16_t packetIndex = 0;

	ptrUcbPacket->payloadLength = UCB_FACTORY_2_LENGTH;
	packetIndex = appendInertialCounts(ptrUcbPacket->payload, packetIndex);
	packetIndex = appendMagnetometerCounts(ptrUcbPacket->payload, packetIndex);
	packetIndex = appendAllTempCounts(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                 packetIndex,
                                 gAlgorithm.bitStatus.BITStatus.all );
	HandleUcbTx(port, ptrUcbPacket); /// send Factory 2 packet
}

/** ****************************************************************************
 * @name _UcbFactory3 send F3 packet
 * @brief this is a STUB sending out all ZEROS for message test
 * Trace: [SDD_UCB_TX_F3 <-- SRC_UCB_TX_F3]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 * @brief There are no A/D's on this system so the message is not needed
 ******************************************************************************/
void _UcbFactory3 (ExternPortTypeEnum port,
                   UcbPacketStruct    *ptrUcbPacket)
{
	uint16_t packetIndex = 0;
    uint8_t i;

	ptrUcbPacket->payloadLength = UCB_FACTORY_3_LENGTH;

    for (i = 0; i < UCB_FACTORY_3_LENGTH; i++)
    {
//        ptrUcbPacket->payload[packetIndex++] = i;
        ptrUcbPacket->payload[packetIndex++] = 0;
    }
#if 0
	packetIndex = appendInertialCounts(ptrUcbPacket->payload, packetIndex);
	packetIndex = appendMagnetometerCounts(ptrUcbPacket->payload, packetIndex);
	packetIndex = appendAllTempCounts(ptrUcbPacket->payload, packetIndex);
	/// add BIT status
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 gAlgorithm.bitStatus.BITStatus.all );
#endif
    if( getUserCommunicationType() == UART_COMM ) {
        HandleUcbTx(port, ptrUcbPacket); /// send Factory 2 packet
    }
}

// FIXME not used in 380 right now
/** ****************************************************************************
 * @name _UcbMagCal1Complete send CB packet (cal phase 1 complete with data)
 * @brief
 * Trace: [SDD_UCB_TX_CB <-- SRC_UCB_TX_CB]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbMagCal1Complete (ExternPortTypeEnum port,
                          UcbPacketStruct    *ptrUcbPacket)
{
	uint8_t packetIndex = 0;

	ptrUcbPacket->payloadLength = UCB_MAG_CAL_1_COMPLETE_LENGTH;

	/// mag roll and pitch offsets
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 gConfiguration.OffsetAnglesExtMag[0]  ); // Roll
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 gConfiguration.OffsetAnglesExtMag[1]  ); // Pitch
	HandleUcbTx(port, ptrUcbPacket); 	/// send phase 1 complete response

}

/** ****************************************************************************
 * @name _UcbMagCal3Complete send CD packet (cal phase 2 complete with data)
 * @brief
 * Trace: [SDD_UCB_TX_CD <-- SRC_UCB_TX_CD]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbMagCal3Complete (ExternPortTypeEnum port,
                          UcbPacketStruct    *ptrUcbPacket)
{
	uint8_t packetIndex = 0;

	ptrUcbPacket->payloadLength = UCB_MAG_CAL_3_COMPLETE_LENGTH;
	/// add requested calibration task
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 0X000B );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// X hard-iron bias
                                 packetIndex,
                                 gConfiguration.hardIronBias[0] );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// Y hard-iron bias
                                 packetIndex,
                                 gConfiguration.hardIronBias[1] );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// soft iron scale ratio
                                 packetIndex,
                                 gConfiguration.softIronScaleRatio );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// soft iron angle
                                 packetIndex,
                                 gConfiguration.softIronAngle );
	HandleUcbTx(port, ptrUcbPacket); /// send phase 2 complete response
}

/** ****************************************************************************
 * @name _UcbNav0 load UCB Navigaion data out SPi and UART
 * @brief Nav 1 load (SPI / UART) and send (UART) kalman filter and GPS data
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbNav0 (ExternPortTypeEnum port,
               UcbPacketStruct    *ptrUcbPacket)
{
	uint16_t packetIndex = 0;

	// set packet length
	ptrUcbPacket->payloadLength = UCB_NAV_0_LENGTH;

	// add roll angle, pitch angle, magnetometer yaw angle
	packetIndex = appendAttitudeTrue((uint8_t *)ptrUcbPacket->payload, packetIndex);

	// add X-angular rate, Y-angular rate, Z-angular rate
	packetIndex = appendCorrectedRates((uint8_t *)ptrUcbPacket->payload, packetIndex);

    // GPS NED Velocities
    packetIndex = appendGpsVel((uint8_t *)ptrUcbPacket->payload, packetIndex);

    // GPS position Lat, Lon, alt
    packetIndex = appendGpsPos((uint8_t *)ptrUcbPacket->payload, packetIndex);

    /// time ITOW Ttruncated
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// timer
                                 packetIndex,
                                 gAlgorithm.timer);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                 packetIndex,
                                 gAlgorithm.bitStatus.BITStatus.all );

    if( getUserCommunicationType() == UART_COMM ) {
        HandleUcbTx(port, ptrUcbPacket); /// send NAV 0 packet
    }
}

/** ****************************************************************************
 * @name _UcbNav1 load UCB Navigaion data out SPi and UART
 * @brief Nav 1 load (SPI / UART) and send (UART) kalman filter and GPS data
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbNav1 (ExternPortTypeEnum port,
               UcbPacketStruct    *ptrUcbPacket)
{
	uint16_t packetIndex = 0;

	// set packet length
	ptrUcbPacket->payloadLength = UCB_NAV_1_LENGTH;

	// add roll angle, pitch angle, magnetometer yaw angle
	packetIndex = appendAttitudeTrue((uint8_t *)ptrUcbPacket->payload, packetIndex);

	// Kalman X-angular rate, Y-angular rate, Z-angular rate
	packetIndex = appendCorrectedRates((uint8_t *)ptrUcbPacket->payload, packetIndex);

    // kalman accels [m/s^2]
    //packetIndex = appendKalmanAccels((uint8_t *)ptrUcbPacket->payload, packetIndex); // alt off 1
	packetIndex = appendAccels((uint8_t *)ptrUcbPacket->payload, packetIndex);  // todo - tm20160318 - do we need Kalman accls?

    // Kalman estimated NED Velocities [m/s]
    packetIndex = appendKalmanVel((uint8_t *)ptrUcbPacket->payload, packetIndex); // alt off 2

    // Kalman estimated position Lat, Lon, alt[m]
#if 1
        packetIndex = appendKalmanPos((uint8_t *)ptrUcbPacket->payload, packetIndex);
#else
        // GPS position Lat, Lon, alt[m]
        packetIndex = appendGpsPos((uint8_t *)ptrUcbPacket->payload, packetIndex);
#endif

    // Rate Temperature
    packetIndex = appendRateTemp((uint8_t *)ptrUcbPacket->payload, packetIndex);

    /// time ITOW
    packetIndex = uint32ToBuffer( ptrUcbPacket->payload,
                                  packetIndex,
                                  gAlgorithm.itow ); //gGpsDataPtr->itow );

#if 1
    // BIT status
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 gAlgorithm.bitStatus.BITStatus.all );
#else
    // replace bit status with counter
    packetIndex = uint16ToBuffer( ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  gAlgorithm.counter );
#endif

    if( getUserCommunicationType() == UART_COMM ) {
        HandleUcbTx(port, ptrUcbPacket); /// send NAV 0 packet
    }
}


/** ****************************************************************************
 * @name _UcbNav2 load UCB Navigaion data out SPi and UART
 * @brief Nav 1 load (SPI / UART) and send (UART) kalman filter and GPS data
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
void _UcbNav2 (ExternPortTypeEnum port,
               UcbPacketStruct    *ptrUcbPacket)
{
	uint16_t packetIndex = 0;

	// set packet length
	ptrUcbPacket->payloadLength = UCB_NAV_2_LENGTH;

	// add roll angle, pitch angle, magnetometer yaw angle
	packetIndex = appendAttitudeTrue((uint8_t *)ptrUcbPacket->payload, packetIndex);

    // add Raw X-angular rate, Y-angular rate, Z-angular rate
    packetIndex = appendRates((uint8_t *)ptrUcbPacket->payload, packetIndex);
    //packetIndex = appendCorrectedRates((uint8_t *)ptrUcbPacket->payload, packetIndex);

    // scaled Accels
    packetIndex = appendAccels((uint8_t *)ptrUcbPacket->payload, packetIndex);

    /// scaled magnetometer X, Y, Z
	packetIndex = appendMagReadings((uint8_t *)ptrUcbPacket->payload, packetIndex);

    // GPS estimated NED Velocities [m/s]
    packetIndex = appendGpsVel ((uint8_t *)ptrUcbPacket->payload, packetIndex);

    // GPS position Lat, Lon, alt[m]
    packetIndex = appendGpsPos((uint8_t *)ptrUcbPacket->payload, packetIndex);
    // Rate Temperature
//    packetIndex = appendRateTemp((uint8_t *)ptrUcbPacket->payload, packetIndex);

    /// time ITOW (gAlgorithm.itow updates at 10 msec, the GPS ITOW updates at 1 sec)
    packetIndex = uint32ToBuffer(ptrUcbPacket->payload,
                                 packetIndex,
                                 gAlgorithm.itow ); //gGpsDataPtr->itow);

    // Use counter in place of bit status
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                 packetIndex,
                                 gAlgorithm.counter );

    if( getUserCommunicationType() == UART_COMM ) {
        HandleUcbTx(port, ptrUcbPacket); /// send NAV 0 packet
    }
}

/** ****************************************************************************
 * @name SendUcbPacket API - taskUserCommunication.c
 * @brief top level send packet routine - calls other send routines based on
 *        packet type
 * Trace:
 *	[SDD_OUTPUT_PACKET <-- SRC_DATA_PACKET_TYPES]
 * @param [in] port - port type UCB or CRM
 * @param [out] packetPtr -- filled in packet from the mapped physical port
 ******************************************************************************/
void SendUcbPacket (ExternPortTypeEnum port,
                    UcbPacketStruct    *ptrUcbPacket)
{

    if ( ( (port == PRIMARY_UCB_PORT) &&  (ptrUcbPacket != 0)) )
    {
        // FIXME could be a lookup table using a search
		switch (ptrUcbPacket->packetType) {
            case UCB_IDENTIFICATION:   // ID 0x4944
                _UcbIdentification(port, ptrUcbPacket);
                break;
            case UCB_VERSION_DATA:     // VR 0x5652
                _UcbVersionData(port, ptrUcbPacket);
                break;
            case UCB_VERSION_ALL_DATA: // VA 0x5641
                _UcbVersionAllData(port, ptrUcbPacket);
                break;
            case UCB_ANGLE_1:          // A1 0x4131
                _UcbAngle1(port, ptrUcbPacket);
                break;
            case UCB_ANGLE_2:          // A2 0x4132
                _UcbAngle2(port, ptrUcbPacket);
                break;
            case UCB_ANGLE_5:          // A2 0x4135
// - todo tm20160603 - FIX!!!
                //_UcbAngle5(port, ptrUcbPacket);
                break;
            case UCB_ANGLE_U:          // AU 0x4155
                _UcbAngleU(port, ptrUcbPacket);
                break;
            case UCB_SCALED_0:         // S0 0x5330
                _UcbScaled0(port, ptrUcbPacket);
                break;
            case UCB_SCALED_1:         // S1 0x5331
                _UcbScaled1(port, ptrUcbPacket);
                break;
            case UCB_TEST_0:           // T0 0x5430
                _UcbTest0(port, ptrUcbPacket);
                break;
            case UCB_TEST_1:           // T1 0x5431
                _UcbTest1(port, ptrUcbPacket);
                break;
            case UCB_FACTORY_1:        // F1 0x4631
                _UcbFactory1(port, ptrUcbPacket);
                break;
            case UCB_FACTORY_2:        // F2 0x4632
                _UcbFactory2(port, ptrUcbPacket);
                break;
            case UCB_FACTORY_3:        // F3 0x4533
                _UcbFactory3(port, ptrUcbPacket);
                break;
            case UCB_MAG_CAL_1_COMPLETE: // CB 0x4342
                _UcbMagCal1Complete(port, ptrUcbPacket);
                break;
            case UCB_MAG_CAL_3_COMPLETE: // CD 0x4344
                _UcbMagCal3Complete(port, ptrUcbPacket);
                break;
            case UCB_NAV_0:              // N0 0x4e30
                _UcbNav0(port, ptrUcbPacket);
                break;
            case UCB_NAV_1:              // N1 0x4e31
                _UcbNav1(port, ptrUcbPacket);
                break;
            case UCB_NAV_2:              // N2 0x4e32
                _UcbNav2(port, ptrUcbPacket);
                break;
			default:
              break; /// default handler?
		}
	}
}

/** ****************************************************************************
 * @name LoadSPIBuffer API - taskUserCommunication.c load the selected
 *       burst message into the output (MISO) buffer.
 * @brief Non-JD SPI burst messages use UCB packets. This routine also limits
 *        the requested packets to the messages available for the model number.
 *        JD burst packet is not included here as all systems can request the
 *        scaled sensor data.
 * Trace:
 * @param [out] packetPtr -- filled in packet from the mapped physical port
 * @retval valid packet in packetPtr TRUE else FALSE
 ******************************************************************************/
void LoadUcbSPIBuffer ( UcbPacketStruct *ptrUcbPacket)
{
  ExternPortTypeEnum dummyPort = 0xf;

    if (ptrUcbPacket->systemType == IMU_6DOF_SYS ) {
        switch (ptrUcbPacket->spiAddress) {
          // this could be the same callback lookup as the UART UCB above
            case SPI_REG_A4_BURST_READ: // 0x43 - DEBUG modified A4 packet
// - todo tm20160603 - FIX!!!
                 //_UcbAngle4(dummyPort, ptrUcbPacket); // load the output buffer
                 break;
            case SPI_REG_S1_BURST_READ: // 0x42
                _UcbScaled1(dummyPort, ptrUcbPacket); // S1
                break;
            case SPI_REG_F1_BURST_READ: // 0x3F
                _UcbFactory1(dummyPort, ptrUcbPacket); // F1
                break;
            default:
                // ??
                break;
        }
    } else if ( ptrUcbPacket->systemType == IMU_9DOF_SYS ) {
        switch (ptrUcbPacket->spiAddress) {
            case SPI_REG_F1_BURST_READ: // 0x3F
                _UcbFactory1(dummyPort, ptrUcbPacket); // F1
                break;
            case SPI_REG_F2_BURST_READ: // 0x40:
                _UcbFactory2(dummyPort, ptrUcbPacket); // F2
                break;
            case SPI_REG_S0_BURST_READ: // 0x41:
                 _UcbScaled0(dummyPort, ptrUcbPacket); // S0
                break;
            case SPI_REG_S1_BURST_READ: // 0x42
                _UcbScaled1(dummyPort, ptrUcbPacket); // S1
                break;
            default:
                // ??
                break;
        }
    } else if ( ptrUcbPacket->systemType < INS_SYS ) {
        switch (ptrUcbPacket->spiAddress) {
            case SPI_REG_F1_BURST_READ: // 0x3F
                _UcbFactory1(dummyPort, ptrUcbPacket); // F1 (54 bytes = 27 words: reg 0x3F --> 0x74)
                break;
            case SPI_REG_F2_BURST_READ: // 0x40:
                _UcbFactory2(dummyPort, ptrUcbPacket); // F2  (66 bytes = 33 words: reg 0x40 --> 0x81)
                break;
            case SPI_REG_S0_BURST_READ: // 0x41:
                _UcbScaled0(dummyPort, ptrUcbPacket); // S0   (30 bytes = 15 words: reg 0x41 --> 0x5E)
                break;
            case SPI_REG_S1_BURST_READ: // 0x42:
                _UcbScaled1(dummyPort, ptrUcbPacket); // S1   (24 bytes = 12 words: reg 0x42 --> 0x59)
                break;
            case SPI_REG_A1_BURST_READ: // 0x43:
                 _UcbAngle1(dummyPort, ptrUcbPacket);// A1   (32 bytes = 16 words: reg 0x43 --> 0x62)
                break;
            default:
                // ??
                break;
        }
    } else if ( ptrUcbPacket->systemType == INS_SYS ) {
        switch (ptrUcbPacket->spiAddress) {
            case SPI_REG_F1_BURST_READ: // 0x3F
                _UcbFactory1(dummyPort, ptrUcbPacket); // F1
                break;
            case SPI_REG_F2_BURST_READ: // 0x40:
                _UcbFactory2(dummyPort, ptrUcbPacket); // F2
                break;
            case SPI_REG_S0_BURST_READ: // 0x41:
                _UcbScaled0(dummyPort, ptrUcbPacket); // S0
                break;
            case SPI_REG_S1_BURST_READ: // 0x42:
                _UcbScaled1(dummyPort, ptrUcbPacket); // S1
                break;
            case SPI_REG_A1_BURST_READ: // 0x43:
                 _UcbAngle1(dummyPort, ptrUcbPacket);// A1
                break;
            case SPI_REG_N0_BURST_READ: // 0x45:
                _UcbNav0(dummyPort, ptrUcbPacket);// N0
                break;
            default:
                // ??
                break;
        }
    }
}
