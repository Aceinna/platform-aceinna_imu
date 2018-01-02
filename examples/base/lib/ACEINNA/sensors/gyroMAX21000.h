/** ***************************************************************************
 * @file gyroMAX21000.c Gyroscope interface for the Maxim 21000 gyro Some
 *       featuresMAX21000 of interest for implementation: SPI, up to 10MHz
 * @author
 * @date   September, 2008
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 * Datasheet is here:
 * http://datasheets.maximintegrated.com/en/ds/MAX21000.pdf
 * Part of the register map taken from the eval kit thumbdrive's code
 * Note, the gyro should implement the interface described in
 * gyroscope.h. This file just provides the specifics, for use by the
 * associated C file only.
 *****************************************************************************/
#include <stdint.h>
#ifndef MAX21000_H
#define MAX21000_H

/**
* Constant: SINGLE_ADD_BURST
* This constant must be added to the address if a burst read on a single
* address must be performed
*/
#define SINGLE_ADD_BURST 0x40
#define READ_INDICATION  0x80


/// Bank selection value
#define BANK_00				0x00
#define BANK_01				0x01
#define BANK_CB				0xFF	// common bank or don't care

/**
* Constant: WHO_AM_I
* WHO_AM_I is the identifier code of MAX21000 and is equal to *0xB1*
*/
#define WHO_AM_I            0xB1
#define WHO_AM_I_BNK		BANK_CB
#define WHO_AM_I_REG		0x20
#define WHO_AM_I_MASK		0xFF
#define WHO_AM_I_POS		0x00
#define WHO_AM_I_LEN		0x08

#define BANK_SEL_BNK		BANK_CB
#define BANK_SEL_REG		0x21
#define BANK_SEL_MASK		0x07
#define BANK_SEL_POS		0x00
#define BANK_SEL_LEN		0x03

/**
* Constants: Data status flags
*
*	STATUS_NO_DATA		- 0x00
*	STATUS_NEW_DATA	- 0x01
*	STATUS_ERR_DATA	- 0x02
*/
#define STATUS_NO_DATA		0x00
#define STATUS_NEW_DATA		0x01
#define STATUS_ERR_DATA		0x02

#define STATUS_BNK			BANK_CB
#define STATUS_REG			0x22
#define STATUS_MASK			0x03
#define STATUS_POS			0x00
#define STATUS_LEN			0x02

#define GYRO_X_MSB_BNK		BANK_CB
#define GYRO_X_MSB_REG		0x23
#define GYRO_X_MSB_MASK		0xFF
#define GYRO_X_MSB_POS		0x00
#define GYRO_X_MSB_LEN		0x08

#define GYRO_X_LSB_BNK		BANK_CB
#define GYRO_X_LSB_REG		0x24
#define GYRO_X_LSB_MASK		0xFF
#define GYRO_X_LSB_POS		0x00
#define GYRO_X_LSB_LEN		0x08

#define GYRO_Y_MSB_BNK		BANK_CB
#define GYRO_Y_MSB_REG		0x25
#define GYRO_Y_MSB_MASK		0xFF
#define GYRO_Y_MSB_POS		0x00
#define GYRO_Y_MSB_LEN		0x08

#define GYRO_Y_LSB_BNK		BANK_CB
#define GYRO_Y_LSB_REG		0x26
#define GYRO_Y_LSB_MASK		0xFF
#define GYRO_Y_LSB_POS		0x00
#define GYRO_Y_LSB_LEN		0x08

#define GYRO_Z_MSB_BNK		BANK_CB
#define GYRO_Z_MSB_REG		0x27
#define GYRO_Z_MSB_MASK		0xFF
#define GYRO_Z_MSB_POS		0x00
#define GYRO_Z_MSB_LEN		0x08

#define GYRO_Z_LSB_BNK		BANK_CB
#define GYRO_Z_LSB_REG		0x28
#define GYRO_Z_LSB_MASK		0xFF
#define GYRO_Z_LSB_POS		0x00
#define GYRO_Z_LSB_LEN		0x08

#define TEMP_MSB_BNK		BANK_CB
#define TEMP_MSB_REG		0x29
#define TEMP_MSB_MASK		0xFF
#define TEMP_MSB_POS		0x00
#define TEMP_MSB_LEN		0x08

#define TEMP_LSB_BNK		BANK_CB
#define TEMP_LSB_REG		0x2A
#define TEMP_LSB_MASK		0xFF
#define TEMP_LSB_POS		0x00
#define TEMP_LSB_LEN		0x08

#define HP_RESET_BNK		BANK_CB
#define HP_RESET_REG		0x3B
#define HP_RESET_MASK		0xFF
#define HP_RESET_POS		0x00
#define HP_RESET_LEN		0x08

/**
* Constants: FIFO status flags
*
*       FIFO_EMPTY      - 0x01
*       FIFO_FULL       - 0x02
*       FIFO_OVER_THS   - 0x04
*       FIFO_RD_EMPTY   - 0x08
*       FIFO_DATA_LOST  - 0x10
*/
#define FIFO_EMPTY          0x01
#define FIFO_FULL           0x02
#define FIFO_OVER_THS		0x04
#define FIFO_RD_EMPTY		0x08
#define FIFO_DATA_LOST		0x10

#define FIFO_COUNT_BNK		BANK_CB
#define FIFO_COUNT_REG		0x3C
#define FIFO_COUNT_MASK		0xFF
#define FIFO_COUNT_POS		0x00
#define FIFO_COUNT_LEN		0x08

#define FIFO_STATUS_BNK		BANK_CB
#define FIFO_STATUS_REG		0x3D
#define FIFO_STATUS_MASK    0x1F
#define FIFO_STATUS_POS		0x00
#define FIFO_STATUS_LEN		0x05

#define FIFO_DATA_BNK		BANK_CB
#define FIFO_DATA_REG		0x3E
#define FIFO_DATA_MASK		0xFF
#define FIFO_DATA_POS		0x00
#define FIFO_DATA_LEN		0x08

#define PAR_RST_BNK		    BANK_CB
#define PAR_RST_REG		    0x3F
#define PAR_RST_MASK		0xFF
#define PAR_RST_POS		    0x00
#define PAR_RST_LEN		    0x08


/**
* Constant: Register 0 of BANK 0 content
*   SENSE_CFG0  -   0x00
*/
#define SENSE_CFG0		0x00

/**
* Constants: Rate Enable
*
*	EN_X_RATE	-	0x1
*	EN_Y_RATE	-	0x2
*	EN_Z_RATE	-	0x4
*	EN_ALL_RATE	-	0x7
*/
#define	EN_X_RATE		0x1
#define	EN_Y_RATE		0x2
#define	EN_Z_RATE		0x4
#define	EN_ALL_RATE		0x7

/// part of SENSE_CFG0
#define RATE_BNK			BANK_00
#define RATE_MASK			0x07
#define RATE_POS			0x00
#define RATE_LEN			0x03

/**
* Constants: Power Mode Constant
*
*	PW_MODE_DEEP_PD - 0x00 = 0x0 << 3
*	PW_MODE_NORMAL  - 0x08 = 0x1 << 3
*	PW_MODE_SLEEP   - 0x18 = 
*	PW_SL_TO_NM     - 0x30
*	PW_PD_TO_NM     - 0x38
*/
//#define	PW_MODE_DEEP_PD	0x00
//#define	PW_MODE_NORMAL	0x08
//#define	PW_MODE_SLEEP	0x10    //  0x18
//#define	PW_MODE_SPOT	0x18
//#define	PW_SL_TO_SP     0x20
//#define	PW_PD_TO_SP     0x28
//#define	PW_SL_TO_NM     0x30
//#define	PW_PD_TO_NM     0x38
#define	PW_MODE_PD      0x0
#define	PW_MODE_NORMAL  0x1
#define	PW_MODE_SLEEP   0x2
#define	PW_MODE_ECO     0x3
#define	PW_SL_TO_ECO    0x4
#define	PW_PD_TO_ECO    0x5
#define	PW_SL_TO_NM     0x6
#define	PW_PD_TO_NM     0x7

/// part of SENSE_CFG0
#define PW_BNK			BANK_00
#define PW_MASK			0x38
#define PW_POS			0x03
#define PW_LEN			0x03


/**
* Constants: Full Scale Constant
*
* If OIS full scale is enabled the FS value is
* downscaled of a factor 8.
*
*	FS_2000_DPS - 0x00
*	FS_1000_DPS - 0x40
*	FS_500_DPS  - 0x80
*	FS_250_DPS  - 0xC0
*/
//#define	FS_2000_DPS		0x00
//#define	FS_1000_DPS		0x40
//#define	FS_500_DPS		0x80
//#define	FS_250_DPS		0xC0
#define	FS_2000_DPS		0x0
#define	FS_1000_DPS		0x1
#define	FS_500_DPS		0x2
#define	FS_250_DPS		0x3

/// part of SENSE_CFG0
#define FS_BNK			BANK_00
#define FS_MASK			0xC0
#define FS_POS			0x06
#define FS_LEN			0x02

/**
* Constant: Register 1 of BANK 0 content
*
*   SENSE_CFG1  -   0x01
*/
#define SENSE_CFG1		0x01

/**
* Constants: OIS full scale Constant
*
* If OIS full scale is enabled the FS value is
* downscaled of a factor 8.
*
*	OIS_FS_ON 	-	0x01
*	OIS_FS_OFF	-	0x00
*/

#define OIS_FS_ON	0x01
#define OIS_FS_OFF	0x00

/// part of SENSE_CFG1
#define OIS_FS_BNK		BANK_00
#define OIS_FS_MASK		0x01
#define OIS_FS_POS		0x00
#define OIS_FS_LEN		0x01

/**
* Constants: Max frequency Constant
*
* This is the maximum frequency usable as ODR
*   MAX_ODR_10K	-	10000
*/
#define SET_MAX_ODR_10K 0x00    // must be removed!!!!
#define MAX_ODR_10K	    10000

/**
* Constants: Bandwidth Constant
* Bandwidth is espressed in Hz @ 10KHz.
*	SNS_BW_2	-	0x00
*	SNS_BW_4	-	0x04	000100b
*	SNS_BW_6	-	0x08	001000b
*	SNS_BW_8	-	0x0C	110000b
*	SNS_BW_10	-	0x10	10000b
*	SNS_BW_14	-	0x14	10100b
*	SNS_BW_22	-	0x18
*	SNS_BW_32	-	0x1C
*	SNS_BW_50	-	0x20
*	SNS_BW_75	-	0x24	100100b
*	SNS_BW_100	-	0x28
*	SNS_BW_150	-	0x2C
*	SNS_BW_200	-	0x30
*	SNS_BW_250	-	0x34
*	SNS_BW_300	-	0x38
*	SNS_BW_400	-	0x3C
*/
#define SNS_BW_2        0x0    // 0x0 << 2 = 0x00
#define SNS_BW_4        0x1    // 0x1 << 2 = 0x04
#define SNS_BW_6        0x2    // 0x2 << 2 = 0x08
#define SNS_BW_8        0x3    // 0x3 << 2 = 0x0C
#define SNS_BW_10       0x4    // 0x4 << 2 = 0x10
#define SNS_BW_14       0x5    // 0x5 << 2 = 0x14
#define SNS_BW_22       0x6    // 0x6 << 2 = 0x18
#define SNS_BW_32       0x7    // 0x7 << 2 = 0x1C
#define SNS_BW_50       0x8    // 0x8 << 2 = 0x20
#define SNS_BW_75       0x9    // 0x9 << 2 = 0x24
#define SNS_BW_100      0xA    // 0xA << 2 = 0x28
#define SNS_BW_150      0xB    // 0xB << 2 = 0x2C
#define SNS_BW_200      0xC    // 0xC << 2 = 0x30
#define SNS_BW_250      0xD    // 0xD << 2 = 0x34
#define SNS_BW_300      0xE    // 0xE << 2 = 0x38
#define SNS_BW_400      0xF    // 0xF << 2 = 0x3C

/// part of SENSE_CFG1
#define BW_BNK			BANK_00
#define BW_MASK			0x3C
#define BW_POS			0x02
#define BW_LEN			0x04

/// Full-scale
#define SNS_FS_NORM     0x0;
#define SNS_FS_OIS      0x1;

/**
* Constant: Register 2 of BANK 0 content
*   SENSE_CFG2  -   0x02
*/
#define SENSE_CFG2		0x02

/**
* Macro: numeric ODR
* ODR setting is computed as defined in the A.N. and depends
* on the max frequency setting.
* Parameters:
*		freqVal - Desired ODR frequency in Hz
*		maxOdr - Value of the max frequency used in Hz
*/
#define numericODR(freqVal, maxOdr) ((freqVal < maxOdr/500) ? ((maxOdr/(20*freqVal))+154) : 	\
                                     (freqVal < maxOdr/10)  ? ((maxOdr/(5*freqVal))+79) 	:	\
                                     ((maxOdr/freqVal)-1)		)

#define ODR_SPOT_200            0x00
#define ODR_SPOT_100            0x01
#define ODR_SPOT_50             0x02
#define ODR_SPOT_25             0x03

#define ODR_BNK			BANK_00
#define ODR_REG			SENSE_CFG2
#define ODR_MASK	    0xFF
#define ODR_POS			0x00
#define ODR_LEN			0x08

/**
* Constant: Register 3 of BANK 0 content
*   SENSE_CFG3  -   0x03
*/
#define SENSE_CFG3		0x03

/**
* Constants: High pass cut-off constant
* These are the settings for the possible cut off frequency
* for the high pass filter of the digital chain.
*
* Cut-off frequency is in the range 0.1Hz (HP_CUT_01) to
* 100Hz (HP_CUT_1000)
*
*    HP_CUT_01		    -    0x00
*    HP_CUT_02		    -    0x01
*    HP_CUT_03		    -    0x02
*    HP_CUT_05		    -    0x03
*    HP_CUT_07		    -    0x04
*    HP_CUT_10		    -    0x05
*    HP_CUT_17		    -    0x06
*    HP_CUT_30		    -    0x07
*    HP_CUT_45		    -    0x08
*    HP_CUT_70		    -    0x09
*    HP_CUT_110		    -    0x0A
*    HP_CUT_170		    -    0x0B
*    HP_CUT_260		    -    0x0C
*    HP_CUT_400		    -    0x0D
*    HP_CUT_640		    -    0x0E
*    HP_CUT_1000		-    0x0F
*/
#define HP_CUT_01		0x00
#define HP_CUT_02		0x01
#define HP_CUT_03		0x02
#define HP_CUT_05		0x03
#define HP_CUT_07		0x04
#define HP_CUT_10		0x05
#define HP_CUT_17		0x06
#define HP_CUT_30		0x07
#define HP_CUT_45		0x08
#define HP_CUT_70		0x09
#define HP_CUT_110		0x0A
#define HP_CUT_170		0x0B
#define HP_CUT_260		0x0C
#define HP_CUT_400		0x0D
#define HP_CUT_640		0x0E
#define HP_CUT_1000		0x0F

/// part of SENSE_CFG3
#define HP_CUT_BNK		BANK_00
#define HP_CUT_MASK		0x0F
#define HP_CUT_POS		0x00
#define HP_CUT_LEN		0x04

/**
* Constants: Data source constant
*
* These are the settings for the possible chain of the digital elaboration
*
* Data source can be: data, data with high pass filter, data at high
* frequency, data at high frequency with high pass filter
*
*	SNS_OUT 		- 0x00
*	SNS_OUT_HP		- 0x10
*	SNS_OUT_HF 		- 0x20
*	SNS_OUT_HF_HP	- 0x30
*/
#define SNS_OUT			0x00
#define SNS_OUT_HP		0x10
#define SNS_OUT_HF		0x20
#define SNS_OUT_HF_HP	0x30

#define SRC_CFG_BNK		BANK_00
#define SRC_CFG_MASK		0x30
#define SRC_CFG_POS		0x04
#define SRC_CFG_LEN		0x02

/**
* Constant: Register 19 of BANK 0 content
*
*   DR_CFG  -   0x13
*/
#define DR_CFG		0x13

/**
* Constants: Data ready reset mode constant
*
* These are the settings for data ready reset mode
*
*	DATA_RDY_ALL	- 0x00
*	DATA_RDY_ANY	- 0x01
*	DATA_RDY_STA	- 0x02
*/
#define DATA_RDY_ALL	0x0
#define DATA_RDY_ANY	0x1
#define DATA_RDY_STA	0x2

#define DATA_RDY_BNK	BANK_00
#define DATA_RDY_REG	DR_CFG
#define DATA_RDY_MASK	0x03
#define DATA_RDY_POS	0x00
#define DATA_RDY_LEN	0x02

/**
* Constant: Register 20 of BANK 0 content
*
*   IO_CFG  -   0x14
*/
#define IO_CFG		0x14

/**
* Constants: Pull Up/Down constant
*
* These are the settings for INT1, INT2, DSYNC pull-up and pull-down
*
*	DSYNC_PD_EN	-	0x80
*	DSYNC_PU_EN	- 	0x40
*	INT1_PD_EN	- 	0x20
*	INT1_PU_EN	- 	0x10
*	INT2_PD_EN	- 	0x08
*	INT2_PU_EN	- 	0x04
*/

#define DSYNC_PD_EN		0x80
#define DSYNC_PU_EN		0x40
#define INT1_PD_EN		0x20
#define INT1_PU_EN		0x10
#define INT2_PD_EN		0x08
#define INT2_PU_EN		0x04

/// part of IO_CFG
#define DSYNC_PUD_BNK   BANK_00
#define DSYNC_PUD_MASK   0xC0
#define DSYNC_PUD_POS   0x06
#define DSYNC_PUD_LEN   0x02

/// part of IO_CFG
#define INT1_PUD_BNK	BANK_00
#define INT1_PUD_MASK	0x30
#define INT1_PUD_POS	0x04
#define INT1_PUD_LEN	0x02

/// part of IO_CFG
#define INT2_PUD_BNK	BANK_00
#define INT2_PUD_MASK	0x0C
#define INT2_PUD_POS	0x02
#define INT2_PUD_LEN	0x02

/**
* Constant: Register 21 of BANK 0 content
*
*   I2C_CFG  -   0x15
*/
#define I2C_CFG		0x15

/**
* Constants: I2C mode constant
*
* These are the settings for I2C mode
*
*	I2C_FAST_NO_SPIKE	- 	0x00
*	I2C_FAST_STD		-	0x10
*	I2C_HIGH_STD		- 	0x20
*	I2C_HIGH_NO_SPIKE	- 	0x30
*	I2C_FAST_NO_DEL 	- 	0x40
*	I2C_ONLY_SPI		- 	0x50
*/

#define I2C_FAST_NO_SPIKE		0x0
#define I2C_FAST_STD			0x1
#define I2C_HIGH_STD			0x3
#define I2C_HIGH_NO_SPIKE		0x4
#define I2C_ONLY_SPI			0x5

/// part of I2C_CFG
#define I2C_MODE_BNK		BANK_00
#define I2C_MODE_MASK		0x70
#define I2C_MODE_POS		0x04
#define I2C_MODE_LEN		0x03

/**
* Constants: Pads current constant
*
* These are the settings for the pads current
*
*	PAD_CURR_3		- 	0x00	3mA driver out
*	PAD_CURR_6		-	0x04	6mA driver out
*	PAD_CURR_12	    - 	0x0C	12mA driver out
*/

#define PAD_CURR_3		0x0
#define PAD_CURR_6		0x1
#define PAD_CURR_12		0x3

/// part of I2C_CFG
#define PAD_BNK		BANK_00
#define PAD_MASK	0x0C
#define PAD_POS		0x02
#define PAD_LEN		0x02

/**
* Constant: I2C_OFF
*
* I2C_OFF set to 0x01 turns off the I2C state machine.
*/

/// part of I2C_CFG
#define	I2C_ON                  0x00
#define	I2C_OFF                 0x01
#define I2C_OFF_BNK             BANK_00
#define I2C_OFF_MASK            0x01
#define I2C_OFF_POS             0x00
#define I2C_OFF_LEN             0x01

/**
* Constant: Register 22 of BANK 0 content
*
*   ITF_OTP  -   0x16
*/
#define ITF_OTP		0x16

/**
* Constant: PAR_ERR
*
* PAR_ERR is set if an error occurs in register address
*/
/// part of ITF_OTP
#define PAR_ERR_BNK		BANK_00
#define PAR_ERR_MASK	0x40
#define PAR_ERR_POS		0x06
#define PAR_ERR_LEN		0x01

/**
* Constant: SPI_3_WIRE
*
* SPI_3_WIRE is set high for use SPI 3 wires mode
*/

/// part of ITF_OTP
#define	SPI_3_WIRE		0x20
#define SPI_3_WIRE_BNK  BANK_00
#define SPI_3_WIRE_MASK 0x20
#define SPI_3_WIRE_POS  0x05
#define SPI_3_WIRE_LEN  0x01

/**
* Constants: Autoincrement and parity
*
* Used for autoincrement enable/disable or even/odd
* parity check on register address
*
*   NO_PARITY_CHECK -   0x00
*   PARITY_EVEN     -   0x80
*   PARITY_ODD      -   0x10
*/

#define	NO_PARITY_CHECK     0x00
#define PARITY_EVEN         0x80
#define PARITY_ODD          0x10

/// part of ITF_OTP
#define IF_PARITY_BNK  BANK_00
#define IF_PARITY_MASK 0x18
#define IF_PARITY_POS  0x03
#define IF_PARITY_LEN  0x02

/**
* Constants: Endian constant
*
* These are the settings for the endian
*
*	BIG_ENDIAN 	- 0x00
*	LITTLE_ENDIAN	- 0x40
*/
#define BIG_ENDIAN	    0x00
#define LITTLE_ENDIAN	0x04

/// part of ITF_OTP
#define ENDIAN_BNK		BANK_00
#define ENDIAN_MASK		0x04
#define ENDIAN_POS		0x02
#define ENDIAN_LEN		0x01

/**
* Constant: MEM_RUN
*
* MEM_RUN is set if OTP is being downloaded
*/

/// part of ITF_OTP
#define MEM_RUN_BNK		BANK_00
#define MEM_RUN_MASK	0x02
#define MEM_RUN_POS		0x01
#define MEM_RUN_LEN		0x01

/**
* Constant: RESTART
*
* RESTART must be set for OTP trimming download
*/

/// part of ITF_OTP
#define	RESTART         0x01
#define RESTART_BNK     BANK_00
#define RESTART_MASK    0x01
#define RESTART_POS     0x00
#define RESTART_LEN     0x01

/**
*   Constant: Register 23 of BANK 0 content
*
*	Represent the threshold of the data for the FIFO ths flag
*	It counts the number of byte (2 byte is one data)
*/

#define	FIFO_THS        0x17
#define FIFO_THS_BNK	BANK_00
#define FIFO_THS_REG	0x17
#define FIFO_THS_MASK	0xFF
#define FIFO_THS_POS	0x00
#define FIFO_THS_LEN	0x08

/**
* Constant: Register 24 of BANK 0 content
*
*   FIFO_CFG  -   0x18
*/
#define FIFO_CFG    0x18

/**
* Constants: FIFO channels
*
*	FIFO_STORE_X	-	0x01
*	FIFO_STORE_Y	-	0x02
*	FIFO_STORE_Z	-	0x04
*/
#define FIFO_STORE_X	0x01
#define FIFO_STORE_Y	0x02
#define FIFO_STORE_Z	0x04

// part of FIFO_CFG
#define FIFO_STORE_BNK	BANK_00
#define FIFO_STORE_MASK	0x07
#define FIFO_STORE_POS	0x00
#define FIFO_STORE_LEN	0x03

/**
* Constants: FIFO overrun mode
*
*	FIFO_OVERRUN_ON	    -   0x10
*	FIFO_OVERRUN_OFF	-   0x00
*/
#define FIFO_OVERRUN_ON		0x10
#define FIFO_OVERRUN_OFF	0x00

/// part of FIFO_CFG
#define FIFO_OVERRUN_BNK	BANK_00
#define FIFO_OVERRUN_MASK	0x10
#define FIFO_OVERRUN_POS	0x04
#define FIFO_OVERRUN_LEN	0x01

/**
* Constants: FIFO interrupt selection
*
*	FIFO_INT_OR	    -   0x00
*	FIFO_INT_AND	-   0x20
*/
#define FIFO_INT_OR     0x00
#define FIFO_INT_AND    0x20

/// part of FIFO_CFG
#define FIFO_INT_BNK	BANK_00
#define FIFO_INT_MASK	0x20
#define FIFO_INT_POS	0x05
#define FIFO_INT_LEN	0x01

/**
* Constants: FIFO modes
*/
#define FIFO_MODE_OFF		0x00
#define FIFO_MODE_NORMAL	0x40
#define FIFO_MODE_INT		0x80
#define FIFO_MODE_TOGGLE	0xC0

/// part of FIFO_CFG
#define FIFO_MODE_BNK	BANK_00
#define FIFO_MODE_MASK	0xC0
#define FIFO_MODE_POS	0x06
#define FIFO_MODE_LEN	0x02

/**
* Constant: Register 26 of BANK 0 content
*
*   DSYNC_CFG  -   0x1A
*/
#define DSYNC_CFG    0x1A

/**
* Constants: DATA SYNC configurations
*
*	DSM_ENB 		-	0x08
*	DSW_LOW 		-	0x10
*	DSW_EDG 		-	0x20
*	DSQ_ENF 		-	0x40
*	DSQ_ENR 		-	0x80
*/
/// part of DSYNC_CFG
#define DSM_ENB         0x08
#define DSM_ENB_BNK	    BANK_00
#define DSM_ENB_MASK	0x08
#define DSM_ENB_POS	    0x03
#define DSM_ENB_LEN	    0x01

#define DSW_LOW         0x10
#define DSW_LOW_BNK	    BANK_00
#define DSW_LOW_MASK	0x10
#define DSW_LOW_POS	    0x04
#define DSW_LOW_LEN	    0x01

#define DSW_EDG         0x20
#define DSW_EDG_BNK	    BANK_00
#define DSW_EDG_MASK	0x20
#define DSW_EDG_POS	    0x05
#define DSW_EDG_LEN	    0x01

#define DSQ_ENF         0x40
#define DSQ_ENF_BNK	    BANK_00
#define DSQ_ENF_MASK    0x40
#define DSQ_ENF_POS	    0x06
#define DSQ_ENF_LEN	    0x01

#define DSQ_ENR         0x80
#define DSQ_ENR_BNK	    BANK_00
#define DSQ_ENR_MASK    0x80
#define DSQ_ENR_POS	    0x07
#define DSQ_ENR_LEN	    0x01

/**
* Constant: Register 27 of BANK 0 content
*
* Represents the number of sample to be
* stored in FIFO upon detecting a DSYNC active edge
*
*   DSYNC_CNT   -   0x1B
*/
#define DSYNC_CNT       0x1B
#define DSYNC_CNT_BNK	BANK_00
#define DSYNC_CNT_REG	DSYNC_CNT
#define DSYNC_CNT_MASK	0xFF
#define DSYNC_CNT_POS	0x00
#define DSYNC_CNT_LEN	0x08

/**
* Constant: Register 0 of BANK 1 content
*
* Represents the rate interrupt reference for X
*
*   INT_REF_X   -   0x00
*/
#define INT_REF_X       0x00
#define INT_REF_X_BNK	BANK_01
#define INT_REF_X_REG	INT_REF_X
#define INT_REF_X_MASK	0xFF
#define INT_REF_X_POS	0x00
#define INT_REF_X_LEN	0x08

/**
* Constant: Register 1 of BANK 1 content
*
* Represents the rate interrupt reference for Y
*
*   INT_REF_Y   -   0x01
*/
#define INT_REF_Y       0x01
#define INT_REF_Y_BNK	BANK_01
#define INT_REF_Y_REG	INT_REF_Y
#define INT_REF_Y_MASK	0xFF
#define INT_REF_Y_POS	0x00
#define INT_REF_Y_LEN	0x08

/**
* Constant: Register 2 of BANK 1 content
*
* Represents the rate interrupt reference for Z
*
*   INT_REF_Z   -   0x02
*/
#define INT_REF_Z       0x02
#define INT_REF_Z_BNK	BANK_01
#define INT_REF_Z_REG	INT_REF_Z
#define INT_REF_Z_MASK	0xFF
#define INT_REF_Z_POS	0x00
#define INT_REF_Z_LEN	0x08

/**
* Constant: Register 3 of BANK 1 content
*
* Represents the rate interrupt debounce for X
*
*   INT_DEB_X   -   0x03
*/
#define INT_DEB_X       0x03
#define INT_DEB_X_BNK	BANK_01
#define INT_DEB_X_REG	INT_DEB_X
#define INT_DEB_X_MASK	0xFF
#define INT_DEB_X_POS	0x00
#define INT_DEB_X_LEN	0x08

/**
* Constant: Register 4 of BANK 1 content
*
* Represents the rate interrupt debounce for Y
*
*   INT_DEB_Y   -   0x04
*/
#define INT_DEB_Y       0x04
#define INT_DEB_Y_BNK	BANK_01
#define INT_DEB_Y_REG	INT_DEB_Y
#define INT_DEB_Y_MASK	0xFF
#define INT_DEB_Y_POS	0x00
#define INT_DEB_Y_LEN	0x08

/**
* Constant: Register 5 of BANK 1 content
*
* Represents the rate interrupt debounce for Z
*
*   INT_DEB_Z   -   0x05
*/
#define INT_DEB_Z       0x05
#define INT_DEB_Z_BNK	BANK_01
#define INT_DEB_Z_REG	INT_DEB_Z
#define INT_DEB_Z_MASK	0xFF
#define INT_DEB_Z_POS	0x00
#define INT_DEB_Z_LEN	0x08

/**
* Constant: Register 6 of BANK 1 content
*
* Represents the rate interrupt configuration for X
*
*   INT_MASK_X   -   0x06
*/
#define INT_MASK_X      0x06
#define INT_MASK_X_BNK	BANK_01
#define INT_MASK_X_REG	INT_MASK_X
#define INT_MASK_X_MASK	0xFF
#define INT_MASK_X_POS	0x00
#define INT_MASK_X_LEN	0x08

/**
* Constant: Register 7 of BANK 1 content
*
* Represents the rate interrupt configuration for Y
*
*   INT_MASK_Y   -   0x07
*/
#define INT_MASK_Y      0x07
#define INT_MASK_Y_BNK	BANK_01
#define INT_MASK_Y_REG	INT_MASK_Y
#define INT_MASK_Y_MASK	0xFF
#define INT_MASK_Y_POS	0x00
#define INT_MASK_Y_LEN	0x08

/**
* Constant: Register 8 of BANK 1 content
*
* Represents the rate interrupt configuration for Z
*
*   INT_MASK_Z   -   0x08
*/
#define INT_MASK_Z      0x08
#define INT_MASK_Z_BNK	BANK_01
#define INT_MASK_Z_REG	INT_MASK_Z
#define INT_MASK_Z_MASK	0xFF
#define INT_MASK_Z_POS	0x00
#define INT_MASK_Z_LEN	0x08

/**
* Constant: Register 25 of BANK 1 content
*
* Revision ID
*
*   REVISION_ID   -   0x19
*/
#define REVISION_ID         0x19
#define REVISION_ID_BNK	    BANK_01
#define REVISION_ID_REG	    REVISION_ID
#define REVISION_ID_MASK	0xFF
#define REVISION_ID_POS	    0x00
#define REVISION_ID_LEN	    0x08

/**
* Constant: Register 26 of BANK 1 content
*
* Serial Number
*
*   SERIAL_0   -   0x1A
*/
#define SERIAL_0        0x1A
#define SERIAL_0_BNK	BANK_01
#define SERIAL_0_REG	SERIAL_0
#define SERIAL_0_MASK	0xFF
#define SERIAL_0_POS	0x00
#define SERIAL_0_LEN	0x08

/**
* Constant: Register 27 of BANK 1 content
*
* Serial Number
*
*   SERIAL_1   -   0x1B
*/
#define SERIAL_1        0x1B
#define SERIAL_1_BNK	BANK_01
#define SERIAL_1_REG	SERIAL_1
#define SERIAL_1_MASK	0xFF
#define SERIAL_1_POS	0x00
#define SERIAL_1_LEN	0x08

/**
* Constant: Register 28 of BANK 1 content
*
* Serial Number
*
*   SERIAL_2   -   0x1C
*/
#define SERIAL_2        0x1C
#define SERIAL_2_BNK	BANK_01
#define SERIAL_2_REG	SERIAL_2
#define SERIAL_2_MASK	0xFF
#define SERIAL_2_POS	0x00
#define SERIAL_2_LEN	0x08

/**
* Constant: Register 29 of BANK 1 content
*
* Serial Number
*
*   SERIAL_3   -   0x1D
*/
#define SERIAL_3        0x1D
#define SERIAL_3_BNK	BANK_01
#define SERIAL_3_REG	SERIAL_3
#define SERIAL_3_MASK	0xFF
#define SERIAL_3_POS	0x00
#define SERIAL_3_LEN	0x08

/**
* Constant: Register 30 of BANK 1 content
*
* Serial Number
*
*   SERIAL_4   -   0x1E
*/
#define SERIAL_4        0x1E
#define SERIAL_4_BNK	BANK_01
#define SERIAL_4_REG	SERIAL_4
#define SERIAL_4_MASK	0xFF
#define SERIAL_4_POS	0x00
#define SERIAL_4_LEN	0x08

/**
* Constant: Register 31 of BANK 1 content
*
* Serial Number
*
*   SERIAL_5   -   0x1F
*/
#define SERIAL_5        0x1F
#define SERIAL_5_BNK	BANK_01
#define SERIAL_5_REG	SERIAL_5
#define SERIAL_5_MASK	0xFF
#define SERIAL_5_POS	0x00
#define SERIAL_5_LEN	0x08

#define MAX21000_BUFFER_SIZE ((NUM_AXIS * sizeof(uint16_t) + 1) + 2)

extern uint8_t MAX21000WhoAmI(uint32_t *);
extern uint8_t MAX21000Config(uint32_t *, uint32_t *);
#endif /* MAX21000_H */