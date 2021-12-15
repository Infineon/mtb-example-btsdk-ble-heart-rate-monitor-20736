
/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
*
* LE Heart Rate profile, service, application
*
* Refer to Bluetooth SIG Heart Rate Profile 1.0 and Heart Rate Service
* 1.0 specifications for details.
*
* This is ROM implementation for LE Heart Rate Monitor device.
* An application can use any portion of this code to simplify development
* and also to reduce download & startup times.
*
* During initialization the app registers with LE stack to receive various
* notifications including bonding complete, connection status change, and
* peer write.  When a device is successfully bonded, the application saves
* the peer's Bluetooth Device address to the NVRAM. The Bonded device can also
* write client configuration descriptor to receive HR measurement; that is
* also saved to the NVRAM.  When a new measurement is received (for example over the
* fake UART), it is sent to the bonded and registered host.
*
* Features demonstrated
*  - GATT database and Device configuration initialization
*  - Registration with LE stack for various events
*  - NVRAM read/write operations
*  - Processing write requests from the client
*  - Sending notifications to the client
*
* To demonstrate the app, work through the following steps:
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Use the CySmart application or a similar one with heart rate monitor
*    functionality to connect & bond
* 4. Explore, receive real time heart rate measurements, and write control
*    point value using the CySmart app
*
*/
#include "spar_utils.h"
#include "bleprofile.h"
#include "bleapp.h"
#include "blehrm.h"
#include "gpiodriver.h"
#include "string.h"
#include "stdio.h"
#include "platform.h"
#include "sparcommon.h"


#define BLE_P2

//////////////////////////////////////////////////////////////////////////////
//                      local interface declaration
//////////////////////////////////////////////////////////////////////////////
static void   heart_rate_monitor_create(void);
static int    heart_rate_monitor_write_handler(LEGATTDB_ENTRY_HDR *p);
static void   heart_rate_monitor_fine_timeout(UINT32 count);
static void   heart_rate_monitor_handle_UART(char *hrm_char);
static void   heart_rate_monitor_fake_UART(char *hrm_char, UINT32 count);
static UINT8  heart_rate_monitor_date_parse(char *data, char *parse_data, UINT8 num);
static int    heart_rate_monitor_writeCb(LEGATTDB_ENTRY_HDR *p);
static void   heart_rate_monitor_send_notification(BLEHRM_HRM_DATA *p_blehrm_hrm_data);

static void   heart_rate_monitor_db_init(void);
static void   heart_rate_monitor_connUp(void);
static void   heart_rate_monitor_connDown(void);
static void   heart_rate_monitor_advStop(void);
static void   heart_rate_monitor_appTimerCb(UINT32 arg);
static void   heart_rate_monitor_appFineTimerCb(UINT32 arg);
static void   heart_rate_monitor_smpBondResult(LESMP_PARING_RESULT  result);
static void   heart_rate_monitor_encryptionChanged(HCI_EVT_HDR *evt);
static UINT32 heart_rate_monitor_hrmButton(UINT32 function);
static void   heart_rate_monitor_timeOut(UINT32 count);

//////////////////////////////////////////////////////////////////////////////
//                      global variables
//////////////////////////////////////////////////////////////////////////////

PLACE_IN_DROM const UINT8 heart_rate_monitor_db_data[]=
{
    // GATT service
    PRIMARY_SERVICE_UUID16 (0x0001, UUID_SERVICE_GATT),

    CHARACTERISTIC_UUID16  (0x0002, 0x0003, UUID_CHARACTERISTIC_SERVICE_CHANGED, LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_NONE, 4),
        0x00, 0x00, 0x00, 0x00,

    // GAP service
    PRIMARY_SERVICE_UUID16 (0x0014, UUID_SERVICE_GAP),

    CHARACTERISTIC_UUID16 (0x0015, 0x0016, UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 15),
        'L','E',' ','H','R',' ','M','o','n','i','t','o','r',0x00,0x00,

    CHARACTERISTIC_UUID16 (0x0017, 0x0018, UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 2),
        BIT16_TO_8(APPEARANCE_GENERIC_HEART_RATE_SENSOR),

    // Heart Rate service
    PRIMARY_SERVICE_UUID16 (0x0028, UUID_SERVICE_HEART_RATE),

    CHARACTERISTIC_UUID16 (0x0029, 0x002a, UUID_CHARACTERISTIC_HEART_RATE_MEASUREMENT,
                           LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_NONE, 4),
        0x08,0x00,0x00,0x00,

    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x002b, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                      LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 2),
        0x00,0x00,

    CHARACTERISTIC_UUID16 (0x002e, 0x002f, UUID_CHARACTERISTIC_HEART_RATE_SENSOR_LOCATION,
                           LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 1),
        0x01,                       // Body Sensor Location Chest

    CHARACTERISTIC_UUID16_WRITABLE (0x0030, 0x0031, UUID_CHARACTERISTIC_HEART_RATE_CONTROL_POINT,
                                    LEGATTDB_CHAR_PROP_WRITE,  LEGATTDB_PERM_WRITE_REQ,  1),
        0x00,

    // Device Info service
    PRIMARY_SERVICE_UUID16 (0x003d, UUID_SERVICE_DEVICE_INFORMATION),

    CHARACTERISTIC_UUID16 (0x003e, 0x003f, UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 8),
        'I','n','f','i','n','e','o','n',

    CHARACTERISTIC_UUID16 (0x0040, 0x0041, UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 8),
        '1','2','3','4',0x00,0x00,0x00,0x00,

    CHARACTERISTIC_UUID16 (0x0042, 0x0043, UUID_CHARACTERISTIC_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 8),
        0x00,0x01,0x02,0x03,0x4,0x5,0x6,0x7,
};

const UINT16 heart_rate_monitor_db_size = sizeof(heart_rate_monitor_db_data);

PLACE_IN_DROM const BLE_PROFILE_CFG heart_rate_monitor_cfg =
{
    /*.fine_timer_interval            =*/ 5000,        //ms
    /*.default_adv                    =*/ 4,           // HIGH_UNDIRECTED_DISCOVERABLE
    /*.button_adv_toggle              =*/ 0,           // pairing button make adv toggle (if 1) or always on (if 0)
    /*.high_undirect_adv_interval     =*/ 32,          // slots
    /*.low_undirect_adv_interval      =*/ 2048,        // slots
    /*.high_undirect_adv_duration     =*/ 300,          // seconds
    /*.low_undirect_adv_duration      =*/ 300,         // seconds
    /*.high_direct_adv_interval       =*/ 0,           // seconds
    /*.low_direct_adv_interval        =*/ 0,           // seconds
    /*.high_direct_adv_duration       =*/ 0,           // seconds
    /*.low_direct_adv_duration        =*/ 0,           // seconds
    /*.local_name                     =*/ "LE HR Monitor",        // [LOCAL_NAME_LEN_MAX]
    /*.cod                            =*/ {BIT16_TO_8(APPEARANCE_GENERIC_HEART_RATE_SENSOR), 0}, // [COD_LEN];
    /*.ver                            =*/ "1.00",      //char ver[VERSION_LEN];
    /*.encr_required                  =*/ 0,           // if 1, encryption is needed before sending indication/notification
    /*.disc_required                  =*/ 0,           // if 1, disconnection after confirmation
    /*.test_enable                    =*/ 1,           // TEST MODE is enabled when 1
    /*.tx_power_level                 =*/ 0x04,        // dbm
    /*.con_idle_timeout               =*/ 30,          // second   0-> no timeout
    /*.powersave_timeout              =*/ 5,           // second  0-> no timeout
    /*.hdl                            =*/ {0x002a, 0x002f, 0x0031, 0x00, 0x00},                        // [HANDLE_NUM_MAX] GATT HANDLE number
    /*.serv                           =*/ {UUID_SERVICE_HEART_RATE, UUID_SERVICE_HEART_RATE, UUID_SERVICE_HEART_RATE, 0x00, 0x00}, //GATT service UUID
    /*.cha                            =*/ {UUID_CHARACTERISTIC_HEART_RATE_MEASUREMENT, UUID_CHARACTERISTIC_HEART_RATE_SENSOR_LOCATION,
                                            UUID_CHARACTERISTIC_HEART_RATE_CONTROL_POINT, 0x00, 0x00},  // GATT characteristic UUID
    /*.findme_locator_enable          =*/ 0,           // if 1 Find me locator is enable
    /*.findme_alert_level             =*/ 0,           // alert level of find me
    /*.client_grouptype_enable        =*/ 0,           // if 1 grouptype read can be used
    /*.linkloss_button_enable         =*/ 0,           // if 1 linkloss button is enable
    /*.pathloss_check_interval        =*/ 0,           // second
    /*.alert_interval                 =*/ 0,           // interval of alert
    /*.high_alert_num                 =*/ 0,           // number of alert for each interval
    /*.mild_alert_num                 =*/ 0,           // number of alert for each interval
    /*.status_led_enable              =*/ 0,           // if 1 status LED is enable
    /*.status_led_interval            =*/ 0,           // second
    /*.status_led_con_blink           =*/ 0,           // blink num of connection
    /*.status_led_dir_adv_blink       =*/ 0,           // blink num of dir adv
    /*.status_led_un_adv_blink        =*/ 0,           // blink num of undir adv
    /*.led_on_ms                      =*/ 0,           // led blink on duration in ms
    /*.led_off_ms                     =*/ 0,           // led blink off duration in ms
    /*.buz_on_ms                      =*/ 0,           // buzzer on duration in ms
    /*.button_power_timeout           =*/ 0,           // seconds
    /*.button_client_timeout          =*/ 0,           // seconds
    /*.button_discover_timeout        =*/ 1,           // seconds
    /*.button_filter_timeout          =*/ 10,          // seconds
#ifdef BLE_UART_LOOPBACK_TRACE
    /*.button_uart_timeout            =*/ 15,          // seconds
#endif
};

typedef struct
{
    //NVRAM save area
    BLEPROFILE_HOSTINFO blehrm_hostinfo;

    BLEHRM_HRM_DATA     blehrm_hrm_data;

    UINT32              blehrm_apptimer_count;
    UINT32              blehrm_appfinetimer_count;
    UINT16              blehrm_con_handle;
    BD_ADDR             blehrm_remote_addr;

    INT32               blehrm_ee_offset;
    UINT16              blehrm_hrm_hdl;
    UINT16              blehrm_hrm_client_hdl;
    UINT16              blehrm_hrm_cp_hdl;
    UINT16              blehrm_hrm_bsl_hdl;
    UINT8               blehrm_bat_enable;
    UINT8               blehrm_notification_enable;
    UINT8               blehrm_measurement_done;
} tHrmAppState;

tHrmAppState *hrmAppState; // was defined as external in Smart version.

///////////////////////////////////////////////////////////////////////////////////////////////////
// Function definitions
///////////////////////////////////////////////////////////////////////////////////////////////////

// Following structure defines UART configuration
const BLE_PROFILE_PUART_CFG blehrm_puart_cfg =
{
    /*.baudrate   =*/ 115200,
    /*.txpin      =*/ PUARTENABLE | GPIO_PIN_UART_TX,
    /*.rxpin      =*/ PUARTENABLE | GPIO_PIN_UART_RX,
};

// Following structure defines GPIO configuration used by the application
const BLE_PROFILE_GPIO_CFG blehrm_gpio_cfg =
{
    /*.gpio_pin =*/
    {
    GPIO_PIN_WP,      // This need to be used to enable/disable NVRAM write protect
    GPIO_PIN_BUTTON,  // Button GPIO is configured to trigger either direction of interrupt
    GPIO_PIN_LED,     // LED GPIO, optional to provide visual effects
    GPIO_PIN_BATTERY, // Battery monitoring GPIO. When it is lower than particular level, it will give notification to the application
    GPIO_PIN_BUZZER,  // Buzzer GPIO, optional to provide audio effects
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 // other GPIOs are not used
    },
    /*.gpio_flag =*/
    {
    GPIO_SETTINGS_WP,
    GPIO_SETTINGS_BUTTON,
    GPIO_SETTINGS_LED,
    GPIO_SETTINGS_BATTERY,
    GPIO_SETTINGS_BUZZER,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    }
};


APPLICATION_INIT()
{
    bleapp_set_cfg((UINT8 *)heart_rate_monitor_db_data,
                   sizeof(heart_rate_monitor_db_data),
                   (void *)&heart_rate_monitor_cfg,
                   (void *)&blehrm_puart_cfg,
                   (void *)&blehrm_gpio_cfg,
                   heart_rate_monitor_create);

    // BLE_APP_DISABLE_TRACING();     ////// Uncomment to disable all tracing
    BLE_APP_ENABLE_TRACING_ON_PUART();
}

// RAM Create function.  We do not call ROM blehrm_Create function because
// it will register its own write handler, which we need to avoid.
void heart_rate_monitor_create(void)
{
    ble_trace0("heart_rate_monitor_create\n");

    hrmAppState = (tHrmAppState *)cfa_mm_Sbrk(sizeof(tHrmAppState));
    memset(hrmAppState, 0x00, sizeof(tHrmAppState));

    bleprofile_Init(bleprofile_p_cfg);
    bleprofile_GPIOInit(bleprofile_gpio_p_cfg);

    heart_rate_monitor_db_init(); //load handle number

    // register connection up and connection down handler.
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_UP, heart_rate_monitor_connUp);
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_DOWN, heart_rate_monitor_connDown);
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_ADV_TIMEOUT, heart_rate_monitor_advStop);

    // handler for Encryption changed.
    blecm_regEncryptionChangedHandler(heart_rate_monitor_encryptionChanged);

    // handler for Bond result
    lesmp_regSMPResultCb((LESMP_SINGLE_PARAM_CB) heart_rate_monitor_smpBondResult);

    // write handler
    legattdb_regWriteHandleCb((LEGATTDB_WRITE_CB)heart_rate_monitor_write_handler);

    bleprofile_regButtonFunctionCb(heart_rate_monitor_hrmButton);

    //data init
    memset(&(hrmAppState->blehrm_hrm_data), 0x00, sizeof(BLEHRM_HRM_DATA));
    hrmAppState->blehrm_hrm_data.flag = HRM_ENERGY_EXPENDED_STATUS;

    bleprofile_regTimerCb(heart_rate_monitor_appFineTimerCb, heart_rate_monitor_appTimerCb);
    bleprofile_StartTimer();

    heart_rate_monitor_connDown();
}


// this callback is issued after peer device wrote something into the GATT database
int heart_rate_monitor_write_handler(LEGATTDB_ENTRY_HDR *p)
{
    UINT16  handle      = legattdb_getHandle(p);
    int     len         = legattdb_getAttrValueLen(p);
    UINT8   *attrPtr    = legattdb_getAttrValue(p);

    ble_trace3("WriteCb: handle %04x client_hdl:%04x cp_hdl:%04x\n", handle, hrmAppState->blehrm_hrm_client_hdl, hrmAppState->blehrm_hrm_cp_hdl);

    if (hrmAppState->blehrm_hrm_client_hdl && handle == hrmAppState->blehrm_hrm_client_hdl)
    {
        BLEPROFILE_DB_PDU db_cl_pdu;

        bleprofile_ReadHandle(hrmAppState->blehrm_hrm_client_hdl, &db_cl_pdu);
        ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);
        ble_trace0("\n");

        bleprofile_ReadNVRAM(VS_BLE_HOST_LIST, sizeof(BLEPROFILE_HOSTINFO), (UINT8 *)&(hrmAppState->blehrm_hostinfo));

        // Save client characteristic descriptor to NVRAM
        if (memcmp(hrmAppState->blehrm_remote_addr, hrmAppState->blehrm_hostinfo.bdAddr, 6) == 0)
        {
            UINT8 writtenbyte;

            hrmAppState->blehrm_hostinfo.serv         = UUID_SERVICE_HEART_RATE;
            hrmAppState->blehrm_hostinfo.cha          = UUID_CHARACTERISTIC_HEART_RATE_MEASUREMENT;
            hrmAppState->blehrm_hostinfo.cli_cha_desc = db_cl_pdu.pdu[0] + (db_cl_pdu.pdu[1]<<8);

            writtenbyte = bleprofile_WriteNVRAM(VS_BLE_HOST_LIST, sizeof(BLEPROFILE_HOSTINFO), (UINT8 *)&(hrmAppState->blehrm_hostinfo));

            ble_trace1("NVRAMWr:%04x\n", writtenbyte);
        }
    }
    else if (hrmAppState->blehrm_hrm_cp_hdl && handle == hrmAppState->blehrm_hrm_cp_hdl)
    {
        BLEHRM_CP_HDR       *cpHdr = (BLEHRM_CP_HDR *) attrPtr ;

        if (cpHdr->opcode == HRM_CP_RESET_ENERGY_EXPENDED)
        {
            // reset variable
            // just keeping the offset value here
            // It should send any message to real sensor and reset it
            hrmAppState->blehrm_ee_offset = hrmAppState->blehrm_hrm_data.ee;
            ble_trace1("blehrm_ee_offset=%d\n", hrmAppState->blehrm_ee_offset);
        }
        else
        {
            return 0x80;
        }
    }

    return 0;
}

void blehrm_timeOut(UINT32 count)
{
    //ble_trace1("Normaltimer:%d", count);

    if (hrmAppState->blehrm_bat_enable)
    {
        blebat_pollMonitor();
    }

    bleprofile_pollPowersave();
}



void heart_rate_monitor_fine_timeout(UINT32 count)
{
    char hrm_char[READ_UART_LEN + 1];
    memset(hrm_char, 0x0, READ_UART_LEN+1);
    //ble_trace1("Finetimer:%d", finecount);

    //Reading
    bleprofile_ReadUART(hrm_char);

    //ble_trace6("UART RX: %02x %02x %02x %02x %02x %02x ",
    //      hrm_char[0], hrm_char[1], hrm_char[2], hrm_char[3], hrm_char[4], hrm_char[5]);

#if 1 // UART RX does not work on the 20736 boards (KirProg3 limitation)
      // so, fake a UART read
    if (bleprofile_p_cfg->test_enable)
    {
        //This is making faking data
        //For test only
        heart_rate_monitor_fake_UART(hrm_char, count);
    }
#endif

    if (hrm_char[0] == 'D' && hrm_char[1] == 'D') //download start
    {
        blecm_setFilterEnable(0);
    }
    else if (hrm_char[0] == 'A' && hrm_char[1] == 'A') //download start
    {
        blecm_setFilterEnable(1);
        heart_rate_monitor_connDown();
    }
    else  //hrm main reading
    {
        heart_rate_monitor_handle_UART(hrm_char);
    }

    // button control
    bleprofile_ReadButton();

}

void heart_rate_monitor_fake_UART(char *hrm_char, UINT32 count)
{
    UINT8 hrm;

    //test only
    if (hrmAppState->blehrm_notification_enable == 1)
    {
        //faking data measurement
        hrm = count&0xFF;
        ble_trace1("hrm=%d", hrm);

        if (hrm >= 100)
        {
            hrm_char[0] = '0' + (hrm / 100);
            hrm_char[1] = '0' + ((hrm % 100) / 10);
            hrm_char[2] = '0' + (hrm % 10);
            hrm_char[3] = ',';
            hrm_char[4] = '0';
            hrm_char[5] = ',';
            hrm_char[6] = '0';
            hrm_char[7] = 0x0a;
        }
        else if (hrm >= 10)
        {
            hrm_char[0] = '0' + ((hrm % 100) / 10);
            hrm_char[1] = '0' + (hrm % 10);
            hrm_char[2] = ',';
            hrm_char[3] = '0';
            hrm_char[4] = ',';
            hrm_char[5] = '0';
            hrm_char[6] = 0x0a;
        }
        else
        {
            hrm_char[0] = '0' + (hrm % 10);
            hrm_char[1] = ',';
            hrm_char[2] = '0';
            hrm_char[3] = ',';
            hrm_char[4] = '0';
            hrm_char[5] = 0x0a;
        }
    }
}


void heart_rate_monitor_handle_UART(char *hrm_char)
{
    int     i;
    UINT8   hrm_len;
    char    parse_char[4];
    UINT32  temp;
    //ble_trace0("ble_handleUART()\n");

    //this part can be replaced by callback function
    if (bleprofile_handleUARTCb)
    {
        hrmAppState->blehrm_measurement_done = bleprofile_handleUARTCb((UINT8 *)hrm_char, (UINT8 *)&(hrmAppState->blehrm_hrm_data));
    }
    else
    {
        //Parse HRM data
        hrm_len = heart_rate_monitor_date_parse(hrm_char, parse_char, 0);
        //ble_trace1("hrm char len:%d\n", hrm_len);

        if (hrm_len)
        {
            hrmAppState->blehrm_hrm_data.hrm = 0;
            //Writing DB with new value
            for (i = 0; i < hrm_len; i++)
            {
                hrmAppState->blehrm_hrm_data.hrm += (parse_char[i] - '0');

                if (i < hrm_len - 1)
                {
                    hrmAppState->blehrm_hrm_data.hrm *= 10;
                }
            }
            hrmAppState->blehrm_hrm_data.flag = HRM_ENERGY_EXPENDED_STATUS;

            //Parse calorie data
            hrm_len = heart_rate_monitor_date_parse(hrm_char, parse_char, 2);

            if (hrm_len)
            {
                hrmAppState->blehrm_hrm_data.ee = 0;
                //Writing DB with new value
                for (i = 0; i < hrm_len; i++)
                {
                    hrmAppState->blehrm_hrm_data.ee += (parse_char[i] - '0');

                    if (i < hrm_len - 1)
                    {
                        hrmAppState->blehrm_hrm_data.ee *= 10;
                    }
                }
                temp = hrmAppState->blehrm_hrm_data.ee;
                temp *= 41868;
                temp /= 10000;
                hrmAppState->blehrm_hrm_data.ee = (UINT16)temp;
            }

            hrmAppState->blehrm_measurement_done = 1; //New measurement is done
        }
    }

    // send Notification
    if (hrmAppState->blehrm_measurement_done) //if connected and encrpted, old data is sent
    {
        heart_rate_monitor_send_notification(&hrmAppState->blehrm_hrm_data);
    }
}

UINT8 heart_rate_monitor_date_parse(char *data, char *parse_data, UINT8 num)
{
    char    *hrm_char = parse_data;
    int     i;
    INT8    len;

    len = -1 * num; // 0 : before finding first ',' 1-3: number written in hrm_char
    for (i = 0; i < READ_UART_LEN; i++)
    {
        if (len < 0)
        {
            //skip until first ','
            if (data[i] == ',')
            {
                len++;
            }
        }
        else if (len >= 0 && len <= 3)
        {
            if (data[i] == '-') // '-' number means wrong entry
            {
                return 0; //error
            }
            else if (data[i] == ',') // ',' means end of number
            {
                hrm_char[len] = 0; // null character

                return len; //return number bytes written
            }
            else if (data[i] == 0x0a) // '\n' means end of number
            {
                hrm_char[len] = 0; // null character

                return len; //return number bytes written
            }
            else if (data[i] == 0x0d) // '\r' means end of number
            {
                hrm_char[len] = 0; // null character

                return len; //return number bytes written
            }
            else if (data[i] >= '0' && data[i] <='9') //number character
            {
                hrm_char[len] = data[i];
                len++;
            }
            else // error character case
            {
                return 0; //error
            }
        }
        else //length is longer than 3 means error happens.
        {
            return 0; //error
        }
    }

    return 0; //No ',' found case, error
}

// Application can call this function to send notification with HRM data to the connected client
void heart_rate_monitor_send_notification(BLEHRM_HRM_DATA *p_blehrm_hrm_data)
{
    BLEPROFILE_DB_PDU   db_pdu, db_cl_pdu;
    //ble_trace0("heart_rate_monitor_send_notification()\n");

    if (hrmAppState->blehrm_notification_enable)
    {
        int i;

        //write partial based on flag
        // FLAG(1) - 1 bytes are mandatory
        db_pdu.pdu[0] = p_blehrm_hrm_data->flag;
        i = 1;

        // Variable item
        if (p_blehrm_hrm_data->flag & HRM_HEARTRATE_VALUE) // 16 byte
        {
            //ble_trace0("HRM_HEARTRATE_VALUE\n");
            db_pdu.pdu[i++] = p_blehrm_hrm_data->hrm & 0xff;
            db_pdu.pdu[i++] = (p_blehrm_hrm_data->hrm >> 8) & 0xff;
        }
        else
        {
            db_pdu.pdu[i++] = p_blehrm_hrm_data->hrm & 0xff;
        }

        //Optional item
        if (p_blehrm_hrm_data->flag & HRM_ENERGY_EXPENDED_STATUS)
        {
            //ble_trace0("HRM_ENERGY_EXPENDED_STATUS\n");
            UINT16 ee = p_blehrm_hrm_data->ee - hrmAppState->blehrm_ee_offset;
            db_pdu.pdu[i++] = ee & 0xff;
            db_pdu.pdu[i++] = (ee >> 8) & 0xff;
        }

        if (p_blehrm_hrm_data->flag & HRM_RR_INTERVAL_SUPPORT)
        {
            //ble_trace0("HRM_RR_INTERVAL_SUPPORT\n");
            memcpy(&(db_pdu.pdu[i]), (UINT8 *)&(p_blehrm_hrm_data->rr[0]), 2 * p_blehrm_hrm_data->rr_len);
            i += (2 * p_blehrm_hrm_data->rr_len);
        }

        // Do not need to save in the DB because attrib is not readable
        db_pdu.len = i;
       // bleprofile_WriteHandle(hrmAppState->blehrm_hrm_hdl, &db_pdu);

        //check client char cfg
        bleprofile_ReadHandle(hrmAppState->blehrm_hrm_client_hdl, &db_cl_pdu);
        ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);
        //ble_trace1("db_cl_pdu.len %d\n", db_cl_pdu.len);
        //ble_trace1("(db_cl_pdu.pdu[0] & CCC_NOTIFICATION); %d\n", (db_cl_pdu.pdu[0] & CCC_NOTIFICATION) );

        if (db_cl_pdu.len == 2 && (db_cl_pdu.pdu[0] & CCC_NOTIFICATION))
        {
            bleprofile_sendNotification(hrmAppState->blehrm_hrm_hdl, (UINT8 *)db_pdu.pdu, db_pdu.len);
            ble_trace0("hrm notification sent to connected client\n");
        }

        hrmAppState->blehrm_measurement_done = 0; //enable new measurement
    }
}

void heart_rate_monitor_db_init(void)
{
    BLEPROFILE_DB_PDU db_pdu;
    int i;
    //load handle number

    for (i = 0; i < HANDLE_NUM_MAX; i++)
    {
        if (bleprofile_p_cfg->serv[i] == UUID_SERVICE_HEART_RATE &&
            bleprofile_p_cfg->cha[i]  == UUID_CHARACTERISTIC_HEART_RATE_MEASUREMENT)
        {
            hrmAppState->blehrm_hrm_hdl = bleprofile_p_cfg->hdl[i];
            ble_trace1("blehrm_hrm_hdl:%04x\n", hrmAppState->blehrm_hrm_hdl);
            bleprofile_ReadHandle(hrmAppState->blehrm_hrm_hdl, &db_pdu);
            //ble_trace5("%02x %02x %02x %02x(%02x)",
            //    db_pdu.pdu[0], db_pdu.pdu[1], db_pdu.pdu[2], db_pdu.pdu[3], db_pdu.len);
            ble_tracen((char *)db_pdu.pdu, db_pdu.len);

            hrmAppState->blehrm_hrm_client_hdl = legattdb_findCharacteristicDescriptor(
                hrmAppState->blehrm_hrm_hdl, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION);

            ble_trace1("blehrm_hrm_client_hdl:%04x", hrmAppState->blehrm_hrm_client_hdl);
            bleprofile_ReadHandle(hrmAppState->blehrm_hrm_client_hdl, &db_pdu);
            //ble_trace5("%02x %02x %02x %02x(%02x)",
            //    db_pdu.pdu[0], db_pdu.pdu[1], db_pdu.pdu[2], db_pdu.pdu[3], db_pdu.len);
            ble_tracen((char *)db_pdu.pdu, db_pdu.len);
        }
        else if (bleprofile_p_cfg->serv[i] == UUID_SERVICE_HEART_RATE &&
                 bleprofile_p_cfg->cha[i]  == UUID_CHARACTERISTIC_HEART_RATE_CONTROL_POINT)
        {
            hrmAppState->blehrm_hrm_cp_hdl = bleprofile_p_cfg->hdl[i];
            ble_trace1("blehrm_hrm_cp_hdl:%04x\n", hrmAppState->blehrm_hrm_cp_hdl);
            bleprofile_ReadHandle(hrmAppState->blehrm_hrm_cp_hdl, &db_pdu);
            //ble_trace5("%02x %02x %02x %02x(%02x)",
            //    db_pdu.pdu[0], db_pdu.pdu[1], db_pdu.pdu[2], db_pdu.pdu[3], db_pdu.len);
            ble_tracen((char *)db_pdu.pdu, db_pdu.len);
        }
        else if (bleprofile_p_cfg->serv[i] == UUID_SERVICE_HEART_RATE &&
                 bleprofile_p_cfg->cha[i]  == UUID_CHARACTERISTIC_HEART_RATE_SENSOR_LOCATION)
        {
            hrmAppState->blehrm_hrm_bsl_hdl = bleprofile_p_cfg->hdl[i];
            ble_trace1("blehrm_hrm_bsl_hdl:%04x\n", hrmAppState->blehrm_hrm_bsl_hdl);
            bleprofile_ReadHandle(hrmAppState->blehrm_hrm_bsl_hdl, &db_pdu);
            //ble_trace5("%02x %02x %02x %02x(%02x)",
            //    db_pdu.pdu[0], db_pdu.pdu[1], db_pdu.pdu[2], db_pdu.pdu[3], db_pdu.len);
            ble_tracen((char *)db_pdu.pdu, db_pdu.len);
        }
        else if (bleprofile_p_cfg->serv[i] == UUID_SERVICE_BATTERY &&
                 bleprofile_p_cfg->cha[i]  == UUID_CHARACTERISTIC_BATTERY_LEVEL)
        {
            hrmAppState->blehrm_bat_enable = 1;
            blebat_Init();
        }
    }

    //init data with HRM
    bleprofile_ReadHandle(hrmAppState->blehrm_hrm_hdl, &db_pdu);
    memcpy((char *)(&(hrmAppState->blehrm_hrm_data)), db_pdu.pdu, sizeof(db_pdu.pdu));
    ble_tracen((char *)(&(hrmAppState->blehrm_hrm_data)), sizeof(db_pdu.pdu));

}

void heart_rate_monitor_connUp(void)
{
    BLEPROFILE_DB_PDU db_cl_pdu;

    hrmAppState->blehrm_con_handle = (UINT16)emconinfo_getConnHandle();
    ble_trace0("blrhrm_connUp()\n");
    // print the bd address.
    memcpy(hrmAppState->blehrm_remote_addr, (UINT8 *)emconninfo_getPeerPubAddr(), sizeof(hrmAppState->blehrm_remote_addr));
    bleprofile_ReadNVRAM(VS_BLE_HOST_LIST, sizeof(BLEPROFILE_HOSTINFO), (UINT8 *)&(hrmAppState->blehrm_hostinfo));

    // Client can set values for Client Configuration descriptor once during bonding.  On
    // every successful connection we need to read value from the NVRAM and set descriptors
    // approprietely
    if (hrmAppState->blehrm_hrm_client_hdl != 0)
    {
        // if we are connected to not bonded device descriptor is 0
        db_cl_pdu.len    = 2;
        db_cl_pdu.pdu[0] = 0x00;
        db_cl_pdu.pdu[1] = 0x00;

        if ((memcmp(hrmAppState->blehrm_remote_addr, hrmAppState->blehrm_hostinfo.bdAddr, 6) == 0) &&
            (hrmAppState->blehrm_hostinfo.serv == UUID_SERVICE_HEART_RATE) &&
            (hrmAppState->blehrm_hostinfo.cha  == UUID_CHARACTERISTIC_HEART_RATE_MEASUREMENT))
        {
            db_cl_pdu.pdu[0] = hrmAppState->blehrm_hostinfo.cli_cha_desc & 0xFF;
            db_cl_pdu.pdu[1] = hrmAppState->blehrm_hostinfo.cli_cha_desc >>8;
        }
        bleprofile_WriteHandle(hrmAppState->blehrm_hrm_client_hdl, &db_cl_pdu);
        ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);
    }

    // Start Connection idle timer to disconnect if there is no activity
    bleprofile_StartConnIdleTimer(bleprofile_p_cfg->con_idle_timeout, bleprofile_appTimerCb);

    // If encryption is not required, we are ready to send notifications
    if (bleprofile_p_cfg->encr_required == 0)
    {
        hrmAppState->blehrm_notification_enable = 1; //notification enable
        ble_trace0("Send notifications enabled.\n");
    }

    bleprofile_Discoverable(NO_DISCOVERABLE, NULL);
}

void heart_rate_monitor_connDown(void)
{
    ble_trace1("blehrm connection down, disc reason: %02x\n", emconinfo_getDiscReason());

    bleprofile_ReadNVRAM(VS_BLE_HOST_LIST, sizeof(BLEPROFILE_HOSTINFO), (UINT8 *)&(hrmAppState->blehrm_hostinfo));

    // Save client characteristic descriptor to NVRAM
    if (memcmp(hrmAppState->blehrm_remote_addr, hrmAppState->blehrm_hostinfo.bdAddr, 6) == 0)
    {
        BLEPROFILE_DB_PDU db_cl_pdu;
        UINT8             writtenbyte;

        if (hrmAppState->blehrm_hrm_client_hdl)
        {
            bleprofile_ReadHandle(hrmAppState->blehrm_hrm_client_hdl, &db_cl_pdu);
            ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);

            hrmAppState->blehrm_hostinfo.serv         = UUID_SERVICE_HEART_RATE;
            hrmAppState->blehrm_hostinfo.cha          = UUID_CHARACTERISTIC_HEART_RATE_MEASUREMENT;
            hrmAppState->blehrm_hostinfo.cli_cha_desc = db_cl_pdu.pdu[0] + (db_cl_pdu.pdu[1]<<8);

            writtenbyte = bleprofile_WriteNVRAM(VS_BLE_HOST_LIST, sizeof(BLEPROFILE_HOSTINFO), (UINT8 *)&(hrmAppState->blehrm_hostinfo));

            ble_trace1("NVRAM write:%04x\n", writtenbyte);
        }
    }

    // Mandatory discovery mode
    if (bleprofile_p_cfg->default_adv == MANDATORY_DISCOVERABLE)
    {
        bleprofile_Discoverable(HIGH_UNDIRECTED_DISCOVERABLE, NULL);
    }
    else
    {
        bleprofile_Discoverable(bleprofile_p_cfg->default_adv, hrmAppState->blehrm_hostinfo.bdAddr);
    }

    hrmAppState->blehrm_con_handle          = 0; //reset connection handle
    hrmAppState->blehrm_notification_enable = 0; //notification enable
    hrmAppState->blehrm_measurement_done    = 0;
    hrmAppState->blehrm_ee_offset           = 0;
}

void heart_rate_monitor_advStop(void)
{
    ble_trace0("ADV Stop\n");
}

void heart_rate_monitor_appTimerCb(UINT32 arg)
{
    switch(arg)
    {
        case BLEPROFILE_GENERIC_APP_TIMER:
            {
                (hrmAppState->blehrm_apptimer_count)++;

                //heart_rate_monitor_timeOut(hrmAppState->blehrm_apptimer_count);
            }
            break;

    }
}

void heart_rate_monitor_appFineTimerCb(UINT32 arg)
{
    (hrmAppState->blehrm_appfinetimer_count)++;

    heart_rate_monitor_fine_timeout(hrmAppState->blehrm_appfinetimer_count);
}


void heart_rate_monitor_smpBondResult(LESMP_PARING_RESULT  result)
{
    ble_trace1("blehrm, bond result %02x\n", result);

    if (result == LESMP_PAIRING_RESULT_BONDED)
    {
        // saving bd_addr in nvram
        UINT8 writtenbyte;

        memcpy(hrmAppState->blehrm_remote_addr, (UINT8 *)emconninfo_getPeerPubAddr(), sizeof(hrmAppState->blehrm_remote_addr));
        memcpy(hrmAppState->blehrm_hostinfo.bdAddr, hrmAppState->blehrm_remote_addr, sizeof(BD_ADDR));

        if (hrmAppState->blehrm_hrm_client_hdl)
        {
            BLEPROFILE_DB_PDU db_cl_pdu;

            bleprofile_ReadHandle(hrmAppState->blehrm_hrm_client_hdl, &db_cl_pdu);
            ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);

            hrmAppState->blehrm_hostinfo.serv         = UUID_SERVICE_HEART_RATE;
            hrmAppState->blehrm_hostinfo.cha          = UUID_CHARACTERISTIC_HEART_RATE_MEASUREMENT;
            hrmAppState->blehrm_hostinfo.cli_cha_desc = db_cl_pdu.pdu[0] + (db_cl_pdu.pdu[1]<<8);
        }

        writtenbyte = bleprofile_WriteNVRAM(VS_BLE_HOST_LIST, sizeof(BLEPROFILE_HOSTINFO), (UINT8 *)&(hrmAppState->blehrm_hostinfo));

        ble_trace1("NVRAM write:%04x\n", writtenbyte);
    }
}


void heart_rate_monitor_encryptionChanged(HCI_EVT_HDR *evt)
{
    ble_trace0("Enc Change\n");

    if (bleprofile_ReadNVRAM(VS_BLE_HOST_LIST, sizeof(BLEPROFILE_HOSTINFO), (UINT8 *)&(hrmAppState->blehrm_hostinfo)))
    {
        if (memcmp(hrmAppState->blehrm_hostinfo.bdAddr, emconninfo_getPeerPubAddr(), 6) == 0)
        {
            ble_trace0("bd address match\n");
        }
    }

    if (bleprofile_p_cfg->encr_required != 0)
    {
        hrmAppState->blehrm_notification_enable = 1; //notification enable

        ble_trace0("Send notifications to client flag enabled.\n");
    }
}

UINT32 heart_rate_monitor_hrmButton(UINT32 function)
{
    if (function == BUTTON_DISCOVER)
    {
        hrmAppState->blehrm_con_handle          = 0; //reset connection handle
        hrmAppState->blehrm_notification_enable = 0; //notification enable
        hrmAppState->blehrm_measurement_done    = 0;
        hrmAppState->blehrm_ee_offset           = 0;
        ble_trace0("Connection handle cleared & notifications disabled. \n");
    }

    return 0;
}


int heart_rate_monitor_writeCb(LEGATTDB_ENTRY_HDR *p)
{
    UINT16  handle      = legattdb_getHandle(p);
    int     len         = legattdb_getAttrValueLen(p);
    UINT8   *attrPtr    = legattdb_getAttrValue(p);

    if (hrmAppState->blehrm_hrm_client_hdl && handle == hrmAppState->blehrm_hrm_client_hdl)
    {
        BLEPROFILE_DB_PDU db_cl_pdu;

        bleprofile_ReadHandle(hrmAppState->blehrm_hrm_client_hdl, &db_cl_pdu);
        ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);

        bleprofile_ReadNVRAM(VS_BLE_HOST_LIST, sizeof(BLEPROFILE_HOSTINFO), (UINT8 *)&(hrmAppState->blehrm_hostinfo));

        // Save client characteristic descriptor to NVRAM
        if (memcmp(hrmAppState->blehrm_remote_addr, hrmAppState->blehrm_hostinfo.bdAddr, 6) == 0)
        {
            UINT8 writtenbyte;

            hrmAppState->blehrm_hostinfo.serv         = UUID_SERVICE_HEART_RATE;
            hrmAppState->blehrm_hostinfo.cha          = UUID_CHARACTERISTIC_HEART_RATE_MEASUREMENT;
            hrmAppState->blehrm_hostinfo.cli_cha_desc = db_cl_pdu.pdu[0] + (db_cl_pdu.pdu[1]<<8);

            writtenbyte = bleprofile_WriteNVRAM(VS_BLE_HOST_LIST, sizeof(BLEPROFILE_HOSTINFO), (UINT8 *)&(hrmAppState->blehrm_hostinfo));

            ble_trace1("NVRAMWr:%04x\n", writtenbyte);
        }
    }
    else if (hrmAppState->blehrm_hrm_cp_hdl && handle == hrmAppState->blehrm_hrm_cp_hdl)
    {
        BLEHRM_CP_HDR       *cpHdr = (BLEHRM_CP_HDR *) attrPtr ;

        if (cpHdr->opcode == HRM_CP_RESET_ENERGY_EXPENDED)
        {
            // reset variable
            // just keeping the offset value here
            // It should send any message to real sensor and reset it
            hrmAppState->blehrm_ee_offset = hrmAppState->blehrm_hrm_data.ee;
            ble_trace1("blehrm_ee_offset=%d\n", hrmAppState->blehrm_ee_offset);
        }
    }

    return 0;
}
