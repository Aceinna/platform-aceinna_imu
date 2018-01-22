/** ***************************************************************************
 * @file sae_j1939.h definitions of SAE J1939 standard
 * @Author Feng
 * @date   Feb. 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef SAE_J1939_H
#define SAE_J1939_H

typedef enum {
  GLOBAL,
  ON_HIGHWAY_EQUIPMENT,
  AGRICULTURE_FORESTRY_EQUIPMENT,
  CONSTRUCTION_EQUIPMENT,
  MARINE,
  INDUSTRY_PROCESS,
  RESERVED
} SAE_J1939_INDUSTRY_GROUP;

typedef enum {
  ARBITRARY_ADDRESS_DISBAlE,
  ARBITRARY_ADDRESS_ENABLE
} ARBITRARY_ADDRESS_CAPABLE;

#define MEMSIC_SAE_J1939_VEHICLE_SYSTEM               0
#define MEMSIC_SAE_J1939_VEHICLE_SYSTEM_INSTANCE      0
#define MEMSIC_SAE_J1939_FUNCTION                     131
#define MEMSIC_SAE_J1939_FUNCTION_INSTANCE            0
#define MEMSIC_SAE_J1939_ECU                          0
#define MEMSIC_SAE_J1939_MANUFACTURE_CODE             823


typedef union {
  struct {
    uint64_t arbitrary_address       : 1;
    uint64_t industry_group          : 3;
    uint64_t vehicle_system_instance : 4;
    uint64_t vehicle_system          : 7;
    uint64_t reserved                : 1;
    uint64_t function                : 8;
    uint64_t function_instance       : 5;
    uint64_t ecu                     : 3;
    uint64_t manufacture_code        : 11;
    uint64_t identity_number         : 21;
  } bits;
  
  uint64_t words;
} SAE_J1939_NAME_FIELD;

typedef struct {
  union {
    struct {
      uint8_t data_page  : 1;
      uint8_t reserved2  : 1;
      uint8_t priority   : 3;
      uint8_t reserved1  : 3;
    } control_bits;
    
    uint8_t r;
  };
  
  uint8_t pdu_format;
  uint8_t pdu_specific;
  uint8_t source;
} SAE_J1939_IDENTIFIER_FIELD;

#define SAE_J1939_PDU_FORMAT_ACK                     232
#define SAE_J1939_PDU_FORMAT_REQUEST                 234
#define SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM           238
#define SAE_J1939_PDU_FORMAT_DATA                    240
#define SAE_J1939_PDU_FORMAT_ECU                     253
#define SAE_J1939_PDU_FORMAT_SOFTWARE_VERSION        254
#define SAE_J1939_PDU_FORMAT_GLOBAL                  255

#define SAE_J1939_GROUP_EXTENSION_ECU                197
#define SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION   218
#define SAE_J1939_GROUP_EXTENSION_ALGORITHM_RESET    80
#define SAE_J1939_GROUP_EXTENSION_SAVE_CONFIGURATION 81
#define SAE_J1939_GROUP_EXTENSION_TEST_HARDWARE      82
#define SAE_J1939_GROUP_EXTENSION_TEST_SOFTWARE      83
#define SAE_J1939_GROUP_EXTENSION_TEST_STATUS        84
#define SAE_J1939_GROUP_EXTENSION_PACKET_RATE        85
#define SAE_J1939_GROUP_EXTENSION_PACKET_TYPE        86
#define SAE_J1939_GROUP_EXTENSION_DIGITAL_FILTER     87
#define SAE_J1939_GROUP_EXTENSION_ORIENTATION        88
#define SAE_J1939_GROUP_EXTENSION_USER_BEHAVIOR      89
#define SAE_J1939_GROUP_EXTENSION_BUILT_IN_TEST      90
#define SAE_J1939_GROUP_EXTENSION_ANGLE_ALARM        91
#define SAE_J1939_GROUP_EXTENSION_CONE_ALARM         92
#define SAE_J1939_GROUP_EXTENSION_ACCELERATION_PARAM 93
#define SAE_J1939_GROUP_EXTENSION_BANK0              240
#define SAE_J1939_GROUP_EXTENSION_BANK1              241
#define SAE_J1939_GROUP_EXTENSION_ADDR               254
#define SAE_J1939_GROUP_EXTENSION_ACK                255

#define SAE_J1939_GROUP_EXTENSION_SLOPE_SENSOR       41      
#define SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE       42
#define SAE_J1939_GROUP_EXTENSION_ACCELERATION       45

#define SAE_J1939_CONTROL_PRIORITY                   6
#define SAE_J1939_REQUEST_PRIORITY                   6
#define SAE_J1939_ACCELERATION_PRIORITY              2
#define SAE_J1939_SLOPE_PRIORITY                     3

#define SAE_J1939_DATA_PAGE                          0

typedef enum {
    MEMSIC_SAE_J1939_VERSION                    = 1,
    MEMSIC_SAE_J1939_ECU_ID                     = 2,
    MEMSIC_SAE_J1939_ALGORITHM_RESET            = 3,
    MEMSIC_SAE_J1939_CONFIG_SAVE                = 4,
    MEMSIC_SAE_J1939_ACK                        = 5,
    MEMSIC_SAE_J1939_HARDWARE_BITS              = 6,
    MEMSIC_SAE_J1939_SOFTWARE_BITS              = 7,
    MEMSIC_SAE_J1939_STATUS                     = 8,
    MEMSIC_SAE_J1939_RATE_DIVIDER               = 9,
    MEMSIC_SAE_J1939_PACKET_TYPE                = 10,
    MEMSIC_SAE_J1939_DIGITAL_FILTER             = 11,
    MEMSIC_SAE_J1939_ORIENTATION                = 12,
    MEMSIC_SAE_J1939_USER_BEHAVIOR              = 13,
    MEMSIC_SAE_J1939_BUILT_IN_TEST              = 14,
    MEMSIC_SAE_J1939_ANGLE_ALARM                = 15,
    MEMSIC_SAE_J1939_CONE_ALARM                 = 16,
    MEMSIC_SAE_J1939_ACCELERATION_PARAMETERS    = 17,
    MEMSIC_SAE_J1939_PS_BANK0                   = 18,
    MEMSIC_SAE_J1939_PS_BANK1                   = 19
} MEMSIC_SAE_J1939_CONTROL;

#define SAE_J1939_IDENTIFIER_LEN                       4
#define SAE_J1939_REQUEST_LEN                          3
#define SAE_J1939_PAYLOAD_MAX_LEN                      8
#define SAE_J1939_MAX_BUFFER_LEN                       20
#define SAE_J1939_REQUEST_LEN                          3

#define MEMSIC_SAE_J1939_VERSION_PACKET_LEN            6
#define MEMSIC_SAE_J1939_ECU_PACKET_LEN                8

#define MEMSIC_SAE_J1939_ALGO_RST_LEN                  3
#define MEMSIC_SAE_J1939_SAVE_CONFIG_LEN               3
#define MEMSIC_SAE_J1939_ACK_LEN                       8
#define MEMSIC_SAE_J1939_HARDWARE_BITS_LEN             8
#define MEMSIC_SAE_J1939_SOFTWARE_BITS_LEN             6
#define MEMSIC_SAE_J1939_STATUS_LEN                    6
#define MEMSIC_SAE_J1939_PACKET_RATE_LEN               2
#define MEMSIC_SAE_J1939_PACKET_TYPE_LEN               2
#define MEMSIC_SAE_J1939_DIGITAL_FILTER_LEN            3
#define MEMSIC_SAE_J1939_ORIENTATION_LEN               3
#define MEMSIC_SAE_J1939_USER_BEHAVIOR_LEN             3
#define MEMSIC_SAE_J1939_BUILT_IN_TEST                 8
#define MEMSIC_SAE_J1939_ANGLE_ALARM_LEN               7
#define MEMSIC_SAE_J1939_CONE_ALARM_LEN                7
#define MEMSIC_SAE_J1939_ACCELERATION_PARAM_LEN        7
#define MEMSIC_SAE_J1939_BANK0_LEN                     8
#define MEMSIC_SAE_J1939_BANK1_LEN                     8

#define MEMSIC_SAE_J1939_SLOPE_SENSOR2_LEN             8
#define MEMSIC_SAE_J1939_ANGULAR_RATE_LEN              8
#define MEMSIC_SAE_J1939_ACCELERATION_LEN              8

#define MEMSIC_SAE_J1939_REQUEST                       0
#define MEMSIC_SAE_J1939_RESPONSE                      1

#define MEMSIC_SAE_J1939_SUCCESS                       0
#define MEMSIC_SAE_J1939_FAILURE                       1

#define MEMSIC_SAE_J1939_ADDRESS_MIN                 128
#define MEMSIC_SAE_J1939_ADDRESS_MAX                 247

#define MEMSIC_SAE_J1939_SLOPE_SENSOR_TYPE             1
#define MEMSIC_SAE_J1939_ANGULAR_RATE_TYPE             2
#define MEMSIC_SAE_J1939_ACCELERATOR_TYPE              4
#define MEMSIC_SAE_J1939_TYPE_MASK                     7

#define MEMSIC_SAE_J1939_SLOPE_OFFSET                  250.00
#define MEMSIC_SAE_J1939_XL_OFFSET                     320.00
#define MEMSIC_SAE_J1939_RATE_OFFSET                   250.00

enum {
  MEMSIC_SAE_J1939_PACKET_RATE_0           =           0,   //quiet
  MEMSIC_SAE_J1939_PACKET_RATE_2           =           2,   // 2Hz
  MEMSIC_SAE_J1939_PACKET_RATE_5           =           5,   // 5Hz
  MEMSIC_SAE_J1939_PACKET_RATE_10          =           10,  // 10Hz
  MEMSIC_SAE_J1939_PACKET_RATE_20          =           20,  // 20Hz
  MEMSIC_SAE_J1939_PACKET_RATE_25          =           25,  // 25Hz
  MEMSIC_SAE_J1939_PACKET_RATE_50          =           50,  // 50Hz
  MEMSIC_SAE_J1939_PACKET_RATE_100         =           100  // 100Hz
};

enum {
  MEMSIC_SAE_J1939_PACKET_SLOPE_SENSOR     =           1,   // slope sensor
  MEMSIC_SAE_J1939_PACKET_ANGULAR_RATE     =           2,   // angular rate
  MEMSIC_SAE_J1939_PACKET_ACCELERATOR      =           4    // accelerator
};
 

typedef enum {
  DESC_IDLE                  =   0,
  DESC_OCCUPIED              =   1,
  DESC_PENDING               =   2
} DESC_STATE;

typedef enum {
  _ECU_IDLE                  =   0,
  _ECU_LOST_CONNECTION       =   1,
  _ECU_NORMAL                =   2,
  _ECU_EMPTY_ADDRESS         =   3,
  _ECU_EXPIRED               =   4
} _ECU_STATUS;

typedef enum {
  _ECU_INVALID_NAME          =   -1,
  _ECU_TX_OVERFLOW           =   -2,
  _ECU_RX_OVERFLOW           =   -3,
  _ECU_BAUDRATE_DETECT       =   1,
  _ECU_WAIT_ADDRESS          =   2,
  _ECU_CHECK_ADDRESS         =   3,
  _ECU_ALGORITHM_RESET       =   4,
  _ECU_READY                 =   64,
  _ECU_WAIT_ID               =   65,
  _ECU_WAIT_SOFTWARE_VER     =   66,
  _ECU_WAIT_ALG_RESET        =   68,
  _ECU_WAIT_CONFIG_SAVE      =   72,
  _ECU_WAIT_BUILTIN_TEST     =   80
} _ECU_STATE;


typedef enum {
  _ECU_MASTER                =   0,
  _ECU_SLAVE                 =   1
} _ECU_CATEGORY;

typedef enum {
  SAE_J1939_REQUEST_PACKET            =    1,
  SAE_J1939_ACK_PACKET                =    2,
  SAE_J1939_RESPONSE_PACKET           =    3,
  SAE_J1939_SET_PACKET                =    4,
  SAE_J1939_DATA_PACKET               =    5
} SAE_J1939_PACKET_TYPE;

struct sae_j1939_tx_desc {
  SAE_J1939_PACKET_TYPE       tx_pkt_type;
  uint8_t                     tx_payload_len;
  DESC_STATE                  tx_pkt_ready;
  SAE_J1939_IDENTIFIER_FIELD  tx_identifier;
  CanTxMsg                    tx_buffer;

  struct sae_j1939_tx_desc *next;  
};

#define SAE_J1939_MAX_TX_DESC             32
#define SAE_J1939_MAX_RX_DESC             32

struct sae_j1939_rx_desc {
  uint8_t                     rx_pkt_len;
  DESC_STATE                  rx_pkt_ready;
  SAE_J1939_IDENTIFIER_FIELD  rx_identifier;
  CanRxMsg                    rx_buffer;
  
  struct sae_j1939_rx_desc * next;
};

typedef struct {
  SAE_J1939_IDENTIFIER_FIELD  ver_pgn;
} VERSION_PACKET;

typedef struct {
  SAE_J1939_IDENTIFIER_FIELD ecu_id_pgn;
  uint8_t *ecu_id;
} ECU_ID_PACKET;

typedef struct {
  SAE_J1939_IDENTIFIER_FIELD addr_claim_pg_pgn;
} ADDR_CLAIM_PG_PACKET;

typedef struct {
    uint8_t request;
    uint8_t dest_address;
    uint8_t success;
} COMMAND_SET_PAYLOAD;

#define MEMSIC_SAE_J1939_BUILTIN_HARDWARE               1
#define MEMSIC_SAE_J1939_BUILTIN_SOFTWARE               2
#define MEMSIC_SAE_J1939_BUILTIN_STATUS                 4

typedef struct {
    uint16_t request             :    1;
    uint16_t master_bit          :    15;
    uint16_t hardware_bit;
    uint16_t power_bit;
    uint16_t env_bit;
} HARDWARE_TEST_PAYLOAD;

typedef struct {
    uint16_t request             :     1;
    uint16_t software_bit        :     15;
    uint16_t algorithm_bit;
    uint16_t data_bit;
} SOFTWARE_TEST_PAYLOAD;

typedef struct {
    uint16_t request             :     1;
    uint16_t hardware_status     :     15;
    uint16_t software_status;
    uint16_t sensor_status;
} STATUS_TEST_PAYLOAD;

typedef struct {
    uint8_t dest_address;
    uint8_t odr;
} RATE_CONFIG_PAYLOAD;

typedef  struct {
    uint8_t dest_address;
    union {
        struct {
            uint8_t slope_sensor       :   1;
            uint8_t angular_rate       :   1;
            uint8_t accelerator        :   1;
            uint8_t reserve            :   5;
        } b;
        
        uint8_t r;
    } type_bits;
} PACKET_TYPE_PAYLOAD;

typedef struct {
    uint8_t dest_address;
    uint8_t rate_cutoff;
    uint8_t accel_cutoff;
} DIGITAL_FILTER_PAYLOAD;

typedef struct {
    uint8_t dest_address;
    uint8_t orien_bits[2];
} ORIENTATION_SETTING;

typedef struct {
    uint8_t dest_address;
    uint8_t  restart_on_overrange;
    uint8_t  dynamic_motion;
} USER_BEHAVIOR_PAYLOAD;

typedef struct {
    uint8_t dest_address;
    uint8_t roll_upper;
    uint8_t roll_lower;
    uint8_t pitch_upper;
    uint8_t pitch_lower;
    uint8_t roll_hysteresis;
    uint8_t pitch_hyseresis;
} ANGLE_ALARM_PAYLOAD;

typedef struct {
    uint8_t dest_address;
    uint16_t alarm_selector;
    uint16_t angle_limit;
    uint16_t angle_hysteresis;
} CONE_ALARM_PAYLOAD;

typedef struct {
    uint8_t dest_address;
    uint16_t x_acceleration;
    uint16_t y_acceleration;
    uint16_t z_acceleration;
} ACCELERATION_PARAM_PAYLOAD;

typedef struct {
        uint8_t alg_reset_ps;
        uint8_t save_cfg_ps;
        uint8_t hardware_bit_ps;
        uint8_t software_bit_ps;
        uint8_t status_ps;
        SAE_J1939_IDENTIFIER_FIELD  bank0_pgn;
} BANK0_PS_PAYLOAD;

typedef struct {
        uint8_t packet_rate_ps;
        uint8_t packet_type_ps;
        uint8_t digital_filter_ps;
        uint8_t orientation_ps;
        uint8_t user_behavior_ps;
        uint8_t angle_alarm_ps;
        uint8_t cone_alarm_ps;
        uint8_t acceleration_param_ps;
        SAE_J1939_IDENTIFIER_FIELD  bank1_pgn;
} BANK1_PS_PAYLOAD;

typedef struct {
    uint64_t pitch                :       24;
    uint64_t roll                 :       24;
    uint64_t pitch_compensation   :       2;
    uint64_t pitch_merit          :       2;
    uint64_t roll_compensation    :       2;
    uint64_t roll_merit           :       2;
    uint64_t measure_latency      :       8;
} SLOPE_SENSOR_2;

typedef struct {
    uint16_t pitch_rate;
    uint16_t roll_rate;
    uint16_t yaw_rate;
    uint16_t time_stamp;
} AUGULAR_RATE;

typedef struct {
    uint16_t   acceleration_x;
    uint16_t   acceleration_y;
    uint16_t   acceleration_z;
    uint16_t   time_stamp;
} ACCELERATION_SENSOR;

typedef enum {
  _ECU_500K      =    0,
  _ECU_250K      =    1,
  _ECU_125K      =    2,
  _ECU_1000K     =    3
} _ECU_BAUD_RATE;
  
#define SAE_J1939_MAX_TABLE_ENTRY         128

// address table
typedef struct {
  SAE_J1939_NAME_FIELD ecu_name;
  uint8_t  address;
  _ECU_STATUS status;
  _ECU_CATEGORY  category;
  tTime last_scan_time; // ms
  tTime idle_time;    // ms
  tTime alive_time;   // second
} ECU_ADDRESS_ENTRY;

typedef struct {
  SAE_J1939_NAME_FIELD ecu_name;
  uint8_t  address;
  uint8_t  baud_rate_detect_enable;
  _ECU_BAUD_RATE  baudRate;
  uint8_t  version[5];
  uint16_t packet_rate;
  uint16_t packet_type;
  uint8_t  accel_cut_off;
  uint8_t  rate_cut_off;
  uint16_t  orien_bits;
  uint8_t  restart_on_overrange;
  uint8_t  dynamic_motion;
  uint8_t roll_upper;
  uint8_t roll_lower;
  uint8_t pitch_upper;
  uint8_t pitch_lower;
  uint8_t roll_hysteresis;
  uint8_t pitch_hyseresis;
  uint16_t alarm_selector;
  uint16_t angle_limit;
  uint16_t angle_hysteresis;
  uint16_t acceleration_x;
  uint16_t acceleration_y;
  uint16_t acceleration_z;
  uint8_t  config_changed;
  uint8_t alg_reset_ps;
  uint8_t save_cfg_ps;
  uint8_t hardware_bit_ps;
  uint8_t software_bit_ps;
  uint8_t status_ps;
  uint8_t packet_rate_ps;
  uint8_t packet_type_ps;
  uint8_t digital_filter_ps;
  uint8_t orientation_ps;
  uint8_t user_behavior_ps;
  uint8_t angle_alarm_ps;
  uint8_t cone_alarm_ps;
  uint8_t acceleration_param_ps;
} EcuConfigurationStruct;

typedef struct {
  SAE_J1939_NAME_FIELD        *name;
  uint8_t                     *addr;
  _ECU_CATEGORY               category;
  _ECU_STATE                  state;
  
  ECU_ADDRESS_ENTRY           * addrTbl;
  
  struct sae_j1939_tx_desc    * curr_tx_desc;
  struct sae_j1939_rx_desc    * curr_process_desc;
  struct sae_j1939_rx_desc    * curr_rx_desc;
  
  void                        (* init_table)(void);
  void                        (* update_table)(ECU_ADDRESS_ENTRY *entry);
  uint8_t                     (* add_entry)(uint8_t, SAE_J1939_NAME_FIELD);
  uint8_t                     (* del_entry)(ECU_ADDRESS_ENTRY *);
  
  uint8_t                     (* xmit)(struct sae_j1939_tx_desc *);
 
} ECU_INSTANCE;

enum {
  _ECU_CONFIG_PACKET_RATE                   =      1,
  _ECU_CONFIG_PACKET_TYPE                   =      2,
  _ECU_CONFIG_DIGITAL_FILTER                =      4,
  _ECU_CONFIG_ORIENTATION                   =      8,
  _ECU_CONFIG_USER_BEHAVIOR                 =     16,
  _ECU_CONFIG_ANGLE_ALARM                   =     32,
  _ECU_CONFIG_CONE_ALARM                    =     64,
  _ECU_CONFIG_ACCELERATION_PARAM            =    128,
  _ECU_CONFIG_GROUP_EXTENSION_BANK          =    256,
  _ECU_CONFIG_MASK                          =    511
};
  
typedef enum {
    MEMSIC_J1939_INVALID_IDENTIFIER         =     -1,
    MEMSIC_J1939_IGNORE                     =      0,
    MEMSIC_J1939_SOFTWARE_VERSION           =      2,
    MEMSIC_J1939_ECU_ID                     =      3,
    MEMSIC_J1939_ALG_RST                    =      4,
    MEMSIC_J1939_CFG_SAVE                   =      5,
    MEMSIC_J1939_HARDWARE_TEST              =      6,
    MEMSIC_J1939_SOFTWARE_TEST              =      7,
    MEMSIC_J1939_STATUS_TEST                =      8,
    MEMSIC_J1939_BUILTIN_TEST               =      9,
    MEMSIC_J1939_DATA                       =      10,
    MEMSIC_J1939_ADDRESS_CLAIM              =      11,
    MEMSIC_J1939_REQUEST_PACKET             =      12,
    MEMSIC_J1939_CONFIG                     =      13
} MEMSIC_J1939_PACKET_TYPE;

enum {
  _ECU_ADDR_AVAILABLE                       =      0,
  _ECU_ADDR_OCCUPIED                        =      1
};

typedef struct {
  uint8_t      status;
  uint8_t      addr;
} MEMSIC_ECU_ADDR;

#define MEMSIC_ECU_ADDRESS_MAX              120

extern ECU_INSTANCE gEcuInst;
extern EcuConfigurationStruct gEcuConfig;
extern EcuConfigurationStruct *gEcuConfigPtr;

extern void sae_j1939_initialize();
extern void initialize_mapping_table();
extern void update_mapping_table(ECU_ADDRESS_ENTRY *);
extern uint8_t del_ecu_mapping_table(ECU_ADDRESS_ENTRY *);

extern void sae_j1939_get_identifier(SAE_J1939_IDENTIFIER_FIELD *);
extern void sae_j1939_set_identifier(SAE_J1939_IDENTIFIER_FIELD *);
extern MEMSIC_J1939_PACKET_TYPE is_valid_j1939_master_recv(struct sae_j1939_rx_desc *);
extern uint8_t is_valid_sae_j1939_identifier(SAE_J1939_IDENTIFIER_FIELD *);

extern void ecu_process(void);

extern uint8_t find_tx_desc(struct sae_j1939_tx_desc **);
extern MEMSIC_J1939_PACKET_TYPE is_valid_data_packet(SAE_J1939_IDENTIFIER_FIELD *);
extern MEMSIC_J1939_PACKET_TYPE is_valid_config_command(SAE_J1939_IDENTIFIER_FIELD *);
extern MEMSIC_J1939_PACKET_TYPE is_data_packet(SAE_J1939_IDENTIFIER_FIELD *);\
extern MEMSIC_J1939_PACKET_TYPE is_valid_address_claim(SAE_J1939_IDENTIFIER_FIELD *);

extern uint8_t send_j1939_packet(struct sae_j1939_tx_desc *);
extern void send_address_claim(ECU_INSTANCE *);
extern void build_set_pkt(struct sae_j1939_tx_desc *, MEMSIC_SAE_J1939_CONTROL, _ECU_CATEGORY);
extern void build_command_set(COMMAND_SET_PAYLOAD *, ECU_ADDRESS_ENTRY *, _ECU_CATEGORY);
extern void process_request_pg(struct sae_j1939_rx_desc *);
extern void build_request_pkt(struct sae_j1939_tx_desc *);

extern void memsic_j1939_transmit_isr(void);
extern void memsic_j1939_receive_isr(void);
#endif // SAE_J1939_H