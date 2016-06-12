/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_tps.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "ble_dis.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"

#include "device_manager.h"
#include "pstorage.h"

#include "app_scheduler.h"
#include "app_timer_appsh.h"

#include "motot_driver.h"
#include "PN532.h"	
#include "key_driver.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Smart_Bike"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_BLE                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_FAST_INTERVAL           160                                      /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL           320                                      /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */

#define APP_ADV_FAST_TIMEOUT            10                                          /**< The duration of the fast advertising period (in seconds). */
#define APP_ADV_SLOW_TIMEOUT            5                                         /**< The duration of the slow advertising period (in seconds). */


#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         10                                          /**< Size of timer operation queues. */

#define APP_TIMER_INTERVAL     					APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)	
#define APP_TIMER_KEY     							APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)	
#define APP_TIMER_NFC										APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)

#define MIN_CONN_INTERVAL              MSEC_TO_UNITS(16, UNIT_1_25_MS)       /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL              MSEC_TO_UNITS(60, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                        /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                            /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                            /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                            /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                            /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                           /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define TX_POWER_LEVEL                      (-8)                                         /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */


#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

#define MANUFACTURER_NAME               "NOA LABS"                      /**< Manufacturer. Will be passed to Device Information Service. */

#define PNP_ID_VENDOR_ID_SOURCE         0x02                                        /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                0x1915                                      /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID               0xEEEE                                      /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION          0x0001                                      /**< Product Version. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define column1	17
#define column2	12
#define column3	24


#define new_bike    '0'
#define on_login		'1'
#define on_station  '2'
#define on_loan		  '3'
#define out_staion  '4'
#define on_back		  '5'

		uint8_t pstorage_wait_flag = 0;
		pstorage_block_t pstorage_wait_handle = 0;
		pstorage_handle_t       base_handle;
		pstorage_handle_t				block_0_handle;
		pstorage_handle_t				block_1_handle;
		pstorage_handle_t				block_2_handle;
		pstorage_handle_t				block_3_handle;
		pstorage_handle_t				block_4_handle;
		pstorage_handle_t				block_5_handle;
		pstorage_handle_t				block_6_handle;
		pstorage_handle_t				block_7_handle;
		pstorage_handle_t				block_8_handle;		
		pstorage_module_param_t param;
		uint8_t                 default_id_value[16];
		uint8_t                 exist_id_value[16];
		uint8_t                 name_data[16];
		uint8_t 								uid_data[16];
		uint8_t 								exist_uid_data[16];
		uint8_t                 exist_name_data[16];
		uint8_t                 dest_data_2[16];
		uint8_t                 dest_data_3[16];
		uint8_t                 dest_data_4[16];
		uint8_t                 dest_data_5[16];
		uint8_t                 dest_data_6[16];
		uint8_t                 dest_data_7[16];
		uint8_t                 dest_data_8[16];
		
uint8_t key_word[10];
uint8_t key_index=0;
uint8_t key_time=0;
uint8_t check_keyword_flag=0;
uint8_t check_back_flag=0;
uint8_t test_flag=0;
uint8_t key_open_flag=0;
bool sleep_flag=0;
bool reset_flag=0;

uint8_t state=new_bike;

extern uint8_t RxData;
uint8_t before_rxdata=0xff;
static ble_tps_t                        m_tps;                                   /**< Structure used to identify the TX Power service. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};
static void buttons_leds_init(bool * p_erase_bonds);
void set_column_input(void);
void set_column_output(void);
void set_row_input(void);
void set_row_output(void);
void app_updata_value(void);
void open_key(void);
uint8_t check_uid(void);
uint8_t check_key(void);
uint8_t check_if_nfc(void);
APP_TIMER_DEF(m_key_read_id);
static dm_application_instance_t        m_app_handle;                               /**< Application identifier allocated by device manager. */
static dm_handle_t                      m_bonded_peer_handle;                       /**< Device reference handle to the current connected peer. */

extern	u8 card_uid[6];
extern	u8 uid_len;
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}
/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
		uint8_t device_name[17]={'S','m','a','r','t','_','B','i','k','e'};
		exist_name_data[0]=exist_id_value[0];
		for(char i=0;i<7;i++)
		{
			device_name[10+i]=exist_name_data[i];
		}
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          device_name,
                                          17);	
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
extern u8 card_uid[6];
extern u8 uid_len;
u8 get_uid_flag=0;
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	if(length==3)
	{
					Get_uid();
	}		
	switch(state)
	{
		case on_login:
			break;
		case on_loan:
			if(length==1)
			{
				check_keyword_flag=1;					
					servo_right();
					servo_left();
			}	
			break;
		case on_back:
			if(length==2)
			{
				check_back_flag=1;	
			}	
			break;			
		default:
			break;
	}
}

/**@snippet [Handling the data received over BLE] */

static void nus_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t m_nus_init;
    
    memset(&m_nus_init, 0, sizeof(m_nus_init));

    m_nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &m_nus_init);
    APP_ERROR_CHECK(err_code);	
}
/**@brief Function for initializing the TX Power Service.
 */
static void tps_init(void)
{
    uint32_t       err_code;
    ble_tps_init_t tps_init_obj;

    memset(&tps_init_obj, 0, sizeof(tps_init_obj));
    tps_init_obj.initial_tx_power_level = TX_POWER_LEVEL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&tps_init_obj.tps_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&tps_init_obj.tps_attr_md.write_perm);

    err_code = ble_tps_init(&m_tps, &tps_init_obj);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    uint32_t         err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
    pnp_id.product_id       = PNP_ID_PRODUCT_ID;
    pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init_obj.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	dis_init();
	nus_init();
	tps_init();
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_SLOW:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_FAST_WHITELIST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_SLOW_WHITELIST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            err_code = ble_advertising_restart_without_whitelist();
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_whitelist_t whitelist;
            ble_gap_addr_t    * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t     * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];

            whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            whitelist.irk_count  = BLE_GAP_WHITELIST_IRK_MAX_COUNT;
            whitelist.pp_addrs   = p_whitelist_addr;
            whitelist.pp_irks    = p_whitelist_irk;

            err_code = dm_whitelist_create(&m_app_handle, &whitelist);
            APP_ERROR_CHECK(err_code);

            err_code = ble_advertising_whitelist_reply(&whitelist);
            APP_ERROR_CHECK(err_code);
            break;
        }
        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            ble_gap_addr_t peer_address;

            // Only Give peer address if we have a handle to the bonded peer.
            if(m_bonded_peer_handle.appl_id != DM_INVALID_ID)
            {
         
                err_code = dm_peer_addr_get(&m_bonded_peer_handle, &peer_address);
                if (err_code != (NRF_ERROR_NOT_FOUND | DEVICE_MANAGER_ERR_BASE))
                {
                    APP_ERROR_CHECK(err_code);

                    err_code = ble_advertising_peer_addr_reply(&peer_address);
                    APP_ERROR_CHECK(err_code);
                }

            }
            break;
        }
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                              err_code;
    ble_gatts_rw_authorize_reply_params_t auth_reply;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
				switch(state)
				{
					case on_login:
							default_id_value[0]=on_station;
							test_flag=1;
							reset_flag=1;
					break;
					
					case on_station:
							default_id_value[0]=on_station;
							test_flag=1;
							reset_flag=1;	
					break;
					
					case on_loan:
						if(check_keyword_flag)
						{
							default_id_value[0]=out_staion;
							test_flag=1;
							reset_flag=1;								
						}
						else
						{
							default_id_value[0]=on_station;
							test_flag=1;
							reset_flag=1;							
						}
						break;
						
					case on_back:
						if(check_back_flag)
						{
							default_id_value[0]=on_station;
							test_flag=1;
							reset_flag=1;								
						}
						else
						{
							default_id_value[0]=out_staion;
							test_flag=1;
							reset_flag=1;							
						}						
						break;					
				}					
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;
				case BLE_GAP_EVT_TIMEOUT:
				switch(state)
				{
					
					case new_bike:
						   sleep_flag=1;
					break;
					
					case on_login:
						if(default_id_value[0]!=on_station)
						{
							default_id_value[0]=new_bike;
							test_flag=1;
							reset_flag=1;
						}
					break;
						
					case on_station:
							default_id_value[0]=on_station;										
					break;	
					
					case on_loan:
						if(check_keyword_flag)
						{
							default_id_value[0]=out_staion;
							test_flag=1;
							reset_flag=1;								
						}
						else
						{
							default_id_value[0]=on_station;
							test_flag=1;
							reset_flag=1;							
						}
						break;
						
					case on_back:	
						if(check_back_flag)
						{
							default_id_value[0]=on_station;
							test_flag=1;
							reset_flag=1;								
						}
						else
						{
							default_id_value[0]=out_staion;
							test_flag=1;
							reset_flag=1;							
						}						
						break;
										
				}
           				
					break;
        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            if(p_ble_evt->evt.gatts_evt.params.authorize_request.type
               != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_PREP_WRITE_REQ)
                    || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
                    || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (p_ble_evt->evt.gatts_evt.params.authorize_request.type
                        == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(m_conn_handle,&auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server and Client timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice 
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
		ble_tps_on_ble_evt(&m_tps, p_ble_evt);
	
	 dm_ble_evt_handler(p_ble_evt);
    
}
static void sys_evt_dispatch(uint32_t sys_evt)
{
    ble_advertising_on_sys_evt(sys_evt);
		pstorage_sys_event_handler(sys_evt);
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
        
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
		
				    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
uint8_t check_row_num()
{
	uint8_t row_num=0;
	if(!nrf_gpio_pin_read(column1))
	{
		row_num=1;
	}
	else if(!nrf_gpio_pin_read(column2))
	{
		row_num=2;
	}
	else if(!nrf_gpio_pin_read(column3))
	{
		row_num=3;
	}
	return row_num;
}
void bsp_event_handler(bsp_event_t event)
{
	bsp_buttons_disable();
	set_column_input();
	set_row_output();
	if(state==on_station)
	{
		app_timer_start(m_key_read_id, APP_TIMER_KEY, NULL);
    switch (event)
    {
        case BSP_EVENT_KEY_0:
						switch(check_row_num())
						{
							case 1:
								key_word[key_index]='1';
								key_index++;
								ble_nus_string_send(&m_nus, "button 1", 8);
							printf("1");
								break;
							case 2:
								key_word[key_index]='2';
								key_index++;								
								ble_nus_string_send(&m_nus, "button 2", 8);
							printf("2");
								break;
							case 3:
								key_word[key_index]='3';
								key_index++;								
								ble_nus_string_send(&m_nus, "button 3", 8);
							printf("3");
								break;
							default:
								ble_nus_string_send(&m_nus, "button error", 12);
								break;
						}
            break;

        case BSP_EVENT_KEY_1:
						switch(check_row_num())
						{
							case 1:
								key_word[key_index]='4';
								key_index++;
								ble_nus_string_send(&m_nus, "button 4", 8);
							printf("4");
								break;
							case 2:
								key_word[key_index]='5';
								key_index++;
								ble_nus_string_send(&m_nus, "button 5", 8);
							printf("5");
								break;
							case 3:
								key_word[key_index]='6';
								key_index++;
								ble_nus_string_send(&m_nus, "button 6", 8);
							printf("6");
								break;
							default:
								ble_nus_string_send(&m_nus, "button error", 12);
								break;
						}
            break;
				
        case BSP_EVENT_KEY_2:
						switch(check_row_num())
						{
							case 1:
								key_word[key_index]='7';
								key_index++;
								ble_nus_string_send(&m_nus, "button 7", 8);
							printf("7");
								break;
							case 2:
								key_word[key_index]='8';
								key_index++;
								ble_nus_string_send(&m_nus, "button 8", 8);
							printf("8");
								break;
							case 3:
								key_word[key_index]='9';
								key_index++;
								ble_nus_string_send(&m_nus, "button 9", 8);
							printf("9");
								break;
							default:
								ble_nus_string_send(&m_nus, "button error", 12);
								break;
						}	
            break;	
				
        case BSP_EVENT_KEY_3:
						switch(check_row_num())
						{
							case 1:
								ble_nus_string_send(&m_nus, "button del", 9);
							printf("del");
								key_index--;
								break;
							case 2:
								key_word[key_index]='0';
								key_index++;
								ble_nus_string_send(&m_nus, "button 0", 9);
							printf("0");
								break;
							case 3:
								key_word[key_index]='F';
								//key_index=0;
								ble_nus_string_send(&m_nus, "button ok", 9);
							printf("ok");
								break;
							default:
								ble_nus_string_send(&m_nus, "button error", 12);
								break;
						}	
            break;	
        default:
            return;						
    }	
						set_column_output();
						set_row_input();	
		bsp_buttons_enable();	
	}
	if(state==out_staion)
	{
		app_timer_start(m_key_read_id, APP_TIMER_KEY, NULL);
    switch (event)
    {
        case BSP_EVENT_KEY_0:
						switch(check_row_num())
						{
							case 1:
								key_word[key_index]='1';
								key_index++;
							printf("1");
								break;
							case 2:
								key_word[key_index]='2';
								key_index++;								
							printf("2");
								break;
							case 3:
								key_word[key_index]='3';
								key_index++;								
							printf("3");
								break;
							default:
								break;
						}
            break;

        case BSP_EVENT_KEY_1:
						switch(check_row_num())
						{
							case 1:
								key_word[key_index]='4';
								key_index++;
							printf("4");
								break;
							case 2:
								key_word[key_index]='5';
								key_index++;
							printf("5");
								break;
							case 3:
								key_word[key_index]='6';
								key_index++;
							printf("6");
								break;
							default:
								break;
						}
            break;
				
        case BSP_EVENT_KEY_2:
						switch(check_row_num())
						{
							case 1:
								key_word[key_index]='7';
								key_index++;
							printf("7");
								break;
							case 2:
								key_word[key_index]='8';
								key_index++;
							printf("8");
								break;
							case 3:
								key_word[key_index]='9';
								key_index++;
							printf("9");
								break;
							default:
								break;
						}	
            break;	
				
        case BSP_EVENT_KEY_3:
						switch(check_row_num())
						{
							case 1:
							printf("del");
								key_index--;
								break;
							case 2:
								key_word[key_index]='0';
								key_index++;
							printf("0");
								break;
							case 3:
								key_word[key_index]='F';
							printf("ok");
								break;
							default:
								break;
						}	
            break;	
        default:
            return;						
    }	
						set_column_output();
						set_row_input();	
		bsp_buttons_enable();				
	}
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        20,//RX_PIN_NUMBER,
        20,//TX_PIN_NUMBER,
        20,//RTS_PIN_NUMBER,
        20,//CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;//BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;//BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED 

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;
    ble_adv_modes_config_t options_station =
    {
        BLE_ADV_WHITELIST_ENABLED,
        BLE_ADV_DIRECTED_ENABLED, 
        BLE_ADV_DIRECTED_SLOW_DISABLED,0,0,
        BLE_ADV_FAST_DISABLED, 32,5,
        BLE_ADV_SLOW_ENABLED, 8000, 0
    };
		ble_adv_modes_config_t options_loan =
    {
        BLE_ADV_WHITELIST_ENABLED,
        BLE_ADV_DIRECTED_ENABLED, 
        BLE_ADV_DIRECTED_SLOW_DISABLED,0,0,
        BLE_ADV_FAST_ENABLED, 160,30,
        BLE_ADV_SLOW_DISABLED, 0, 0
    };
		ble_adv_modes_config_t option_default =
    {
        BLE_ADV_WHITELIST_ENABLED,
        BLE_ADV_DIRECTED_ENABLED, 
        BLE_ADV_DIRECTED_SLOW_DISABLED,0,0,
        BLE_ADV_FAST_ENABLED, 160, 1,
        BLE_ADV_SLOW_DISABLED, APP_ADV_SLOW_INTERVAL, APP_ADV_SLOW_TIMEOUT
    };
	switch(state)
	{
		case on_station:
		    err_code = ble_advertising_init(&advdata, NULL, &options_station, on_adv_evt, ble_advertising_error_handler);
    APP_ERROR_CHECK(err_code);		
		break;	
		
		case on_loan:
		case on_login:
		case on_back:
		    err_code = ble_advertising_init(&advdata, NULL, &options_loan, on_adv_evt, ble_advertising_error_handler);
    APP_ERROR_CHECK(err_code);				
			break;
		default:
		    err_code = ble_advertising_init(&advdata, NULL, &option_default, on_adv_evt, ble_advertising_error_handler);
    APP_ERROR_CHECK(err_code);				
			break;
	}
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init( BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}
/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void key_read_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	uint8_t back_key[4]="888F";
	key_time++;
	if(key_time>100)
	{
		key_index=0;
		memset(key_word,0,10);
		key_time=0;
		app_timer_stop(m_key_read_id);
	}
	else if(state==on_station&&key_word[key_index]=='F')
	{
		if(key_index==0)
		{
				default_id_value[0]=on_loan;
				uid_data[6]=1;
				test_flag=1;
				reset_flag=1;
		}
		else
		{
					default_id_value[0]=on_loan;
					default_id_value[1]=key_index+1;
			for(char i=0;i<(key_index+1);i++)
			{
				default_id_value[2+i]=key_word[i];
			}
					  test_flag=1;
						reset_flag=1;
		}
		key_index=0;
		memset(key_word,0,10);
		key_time=0;
		app_timer_stop(m_key_read_id);
	}
	else if(state==out_staion&&key_word[key_index]=='F')
	{
		if(key_index==0)
		{
			if(check_uid())
			{
				  memset(key_word,0,10);
					servo_right();
					servo_left();				
			}				
		}
		else if(!memcmp(back_key,key_word,4))
		{
			default_id_value[0]=on_back;
			default_id_value[1]=4;
      memcpy(&default_id_value[2],back_key,4);		
			test_flag=1;
			reset_flag=1;			
		}
		else if(check_key())
		{
			memset(key_word,0,10);
			servo_right();
			servo_left();					
		}
		key_index=0;
		memset(key_word,0,10);
		key_time=0;
		app_timer_stop(m_key_read_id);
	}
}
uint8_t check_key(void)
{
	if(exist_id_value[1]==key_index+1)
	{
		for(char i=0;i<exist_id_value[1];i++)
		{
			if(exist_id_value[2+i]!=key_word[i])
				return 0;
		}
		return 1;
	}
	return 0;
}
static void timers_init(void)
{

    // Initialize timer module.
APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

    // Create timers.
    uint32_t err_code;
	
    err_code = app_timer_create(&m_key_read_id, APP_TIMER_MODE_REPEATED, key_read_handler);
    APP_ERROR_CHECK(err_code); 	
}
void set_column_input()
{
	nrf_gpio_cfg_input(column1,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(column2,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(column3,NRF_GPIO_PIN_PULLUP);
}
void set_column_output()
{
	nrf_gpio_cfg_output(column1);
	nrf_gpio_cfg_output(column2);
	nrf_gpio_cfg_output(column3);
	
	nrf_gpio_pin_clear(column1);
	nrf_gpio_pin_clear(column2);
	nrf_gpio_pin_clear(column3);	
}
void set_row_output()
{
	nrf_gpio_cfg_output(13);
	nrf_gpio_cfg_output(14);
	nrf_gpio_cfg_output(15);
	nrf_gpio_cfg_output(16);
	
	nrf_gpio_pin_clear(13);
	nrf_gpio_pin_clear(14);
	nrf_gpio_pin_clear(15);
	nrf_gpio_pin_clear(16);		
}
void set_row_input()
{
	nrf_gpio_cfg_input(13,NRF_GPIO_PIN_PULLUP);//ROW4
	nrf_gpio_cfg_input(14,NRF_GPIO_PIN_PULLUP);//ROW3
	nrf_gpio_cfg_input(15,NRF_GPIO_PIN_PULLUP);//ROW2
	nrf_gpio_cfg_input(16,NRF_GPIO_PIN_PULLUP);//ROW1
}
void io_init()
{
  nrf_gpio_cfg_output(0);//LED_G
  nrf_gpio_cfg_output(1);//LED_B
	nrf_gpio_cfg_output(30);//LED_R
	
	nrf_gpio_pin_clear(0);
	nrf_gpio_pin_clear(1);
	nrf_gpio_pin_clear(30);
	
	nrf_gpio_cfg_output(5);//CS
	nrf_gpio_cfg_output(2);//SCK
	nrf_gpio_cfg_output(4);//MOSI	
	nrf_gpio_cfg_watcher(3);//,NRF_GPIO_PIN_PULLDOWN);//MISO	
	
	nrf_gpio_cfg_output(9);
	nrf_gpio_pin_clear(9);
	
	nrf_gpio_cfg_output(8);//5V_EN
	nrf_gpio_pin_set(8);
	
	nrf_gpio_cfg_output(10);//NFC_EN
	nrf_gpio_pin_set(10);	
	
	set_column_output();
	set_row_input();
}
static void example_cb_handler(pstorage_handle_t  * handle,
															 uint8_t              op_code,
                               uint32_t             result,
                               uint8_t            * p_data,
                               uint32_t             data_len)
{
		if(handle->block_id == pstorage_wait_handle) { pstorage_wait_flag = 0; }  //If we are waiting for this callback, clear the wait flag.
	
		switch(op_code)
		{
			case PSTORAGE_LOAD_OP_CODE:
				 if (result == NRF_SUCCESS)
				 {
			 
				 }
				 else
				 {

				 }
				 break;
			case PSTORAGE_STORE_OP_CODE:
				 if (result == NRF_SUCCESS)
				 {

				 }
				 else
				 {

				 }
				 break;				 
			case PSTORAGE_UPDATE_OP_CODE:
				 if (result == NRF_SUCCESS)
				 {

				 }
				 else
				 {

				 }
				 break;
			case PSTORAGE_CLEAR_OP_CODE:
				 if (result == NRF_SUCCESS)
				 {

				 }
				 else
				 {

				 }
				 break;
			 
		}			
}
void app_pstorage_init()
{
		pstorage_init();
	
		param.block_size  = 16;                   //Select block size of 16 bytes
		param.block_count = 9;                   //Select 10 blocks, total of 160 bytes
		param.cb          = example_cb_handler;   //Set the pstorage callback handler	
		pstorage_register(&param, &base_handle);
	
		pstorage_block_identifier_get(&base_handle, 0, &block_0_handle);
		pstorage_block_identifier_get(&base_handle, 1, &block_1_handle);
		pstorage_block_identifier_get(&base_handle, 2, &block_2_handle);
		pstorage_block_identifier_get(&base_handle, 3, &block_3_handle);
		pstorage_block_identifier_get(&base_handle, 4, &block_4_handle);
		pstorage_block_identifier_get(&base_handle, 5, &block_5_handle);
		pstorage_block_identifier_get(&base_handle, 6, &block_6_handle);
		pstorage_block_identifier_get(&base_handle, 7, &block_7_handle);
		pstorage_block_identifier_get(&base_handle, 8, &block_8_handle);
	
		pstorage_load(dest_data_8 , &block_0_handle, 16, 0);				 //Read from flash, only one block is allowed for each pstorage_load command
		pstorage_load(dest_data_7, &block_1_handle, 16, 0);				 //Read from flash
		pstorage_load(dest_data_2, &block_2_handle, 16, 0);			   //Read from flash
		pstorage_load(dest_data_3, &block_3_handle, 16, 0);			   //Read from flash
		pstorage_load(dest_data_4, &block_4_handle, 16, 0);			   //Read from flash
		pstorage_load(dest_data_5, &block_5_handle, 16, 0);			   //Read from flash
		
		pstorage_load(uid_data, &block_6_handle, 16, 0);			   //Read from flash
		pstorage_load(name_data , &block_7_handle, 16, 0);			   //Read from flash
		pstorage_load(default_id_value, &block_8_handle, 16, 0);			   //Read from flash
	
	memcpy(exist_id_value,default_id_value,16);
	memcpy(exist_name_data,name_data,16);
	memcpy(exist_uid_data,uid_data,16);
	for(char i=0;i<7;i++)
	{
		name_data[i]='9';
	}
}
void app_clear_all()
{
		pstorage_clear(&block_0_handle, 16); 
		pstorage_clear(&block_1_handle, 16); 
		pstorage_load(default_id_value, &block_0_handle, 16, 0);				 //Read from flash, only one block is allowed for each pstorage_load command
		pstorage_load(name_data, &block_1_handle, 16, 0);				 //Read from flash, only one block is allowed for each pstorage_load command

}
void app_updata_value()
{
    if(test_flag)    
		{			
			test_flag=0;
			pstorage_update(&block_6_handle, uid_data, 16, 0);
			pstorage_update(&block_7_handle, name_data, 16, 0);
			pstorage_update(&block_8_handle, default_id_value, 16, 0);
		}	
if(sleep_flag&&state!=out_staion)	
{
	sleep_flag=0;
	printf("sleep");
	nrf_delay_ms(2000);
	sd_power_system_off();
}	
if(reset_flag)
{
	reset_flag=0;
	printf("reset");
	nrf_delay_ms(2000);
	sd_nvic_SystemReset();
}

}

static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */


/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

    switch (p_event->event_id)
    {
        case DM_EVT_DEVICE_CONTEXT_LOADED: // Fall through.
        case DM_EVT_SECURITY_SETUP_COMPLETE:
            m_bonded_peer_handle = (*p_handle);
            break;
    }

    return NRF_SUCCESS;
}
/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize peer device handle.
    err_code = dm_handle_initialize(&m_bonded_peer_handle);
    APP_ERROR_CHECK(err_code);

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}
uint8_t check_uid(void)
{
						if(exist_uid_data[6]==1)
						{
							nrf_gpio_pin_clear(10);
							Get_uid();
							if(!memcmp(uid_data,exist_uid_data,6))
							return 1;
						}
					return 0;
}
uint8_t check_if_nfc()
{
					if(uid_data[6]==1)
					{
						Get_uid();				
						test_flag=1;
						return 1;
					}
					else
						return 0;
}

/**@brief Application main function.
 */
int main(void)
{
	bool erase_bonds;
  timers_init();
	app_pstorage_init();
  uart_init();
	pwm_driver_init();
  io_init();
  buttons_leds_init(&erase_bonds);	
	state=default_id_value[0];
	servo_left();	
	switch(state)
	{
		case new_bike:
			
			default_id_value[0]=on_login;
			test_flag=1;
			sleep_flag=1;		
			ble_stack_init();
			scheduler_init();	
			device_manager_init(true);	
			gap_params_init();
			advertising_init(); 
			services_init();  
			conn_params_init();	
			printf("\r\n new_bike!\r\n");
			ble_advertising_start(BLE_ADV_MODE_IDLE);//BLE_ADV_MODE_IDLE BLE_ADV_MODE_FAST BLE_ADV_MODE_DIRECTED_SLOW
			break;
		
		case on_login:
			ble_stack_init();
			scheduler_init();	
			device_manager_init(true);
			gap_params_init();
			advertising_init(); 
			services_init();  
			conn_params_init();
			printf("\r\n on_login!\r\n");
			ble_advertising_start(BLE_ADV_MODE_FAST);//BLE_ADV_MODE_IDLE BLE_ADV_MODE_FAST BLE_ADV_MODE_DIRECTED_SLOW
			break;
		
		case on_station:
			ble_stack_init();
			scheduler_init();	
			device_manager_init(false);
			gap_params_init();
			advertising_init(); 
			services_init();  
			conn_params_init();
			printf("\r\n on_station!\r\n");
			ble_advertising_start(BLE_ADV_MODE_SLOW);//BLE_ADV_MODE_IDLE BLE_ADV_MODE_FAST BLE_ADV_MODE_DIRECTED_SLOW
			break;
		
		case on_loan:
			check_if_nfc();
			ble_stack_init();
			scheduler_init();	
			device_manager_init(false);
			gap_params_init();
			advertising_init(); 
			services_init();  
			conn_params_init();
			printf("\r\n on_loan!\r\n");
			check_if_nfc();	
			ble_advertising_start(BLE_ADV_MODE_FAST);		
			break;
		
		case out_staion:
			ble_stack_init();
			scheduler_init();	
			device_manager_init(false);	
			gap_params_init();
			advertising_init(); 
			services_init();  
			conn_params_init();	
			printf("\r\n out_staion!\r\n");
			ble_advertising_start(BLE_ADV_MODE_IDLE);		
		break;
		
		case on_back:
			ble_stack_init();
			scheduler_init();	
			device_manager_init(true);
			gap_params_init();
			advertising_init(); 
			services_init();  
			conn_params_init();
			printf("\r\n on_back!\r\n");
			ble_advertising_start(BLE_ADV_MODE_FAST);	
		break;	
		
		default:
			default_id_value[0]=on_login;
			test_flag=1;
			sleep_flag=1;	
			ble_stack_init();
			scheduler_init();	
			device_manager_init(true);	
			gap_params_init();
			advertising_init(); 
			services_init();  
			conn_params_init();	
			printf("\r\n default!\r\n");
			ble_advertising_start(BLE_ADV_MODE_IDLE);//BLE_ADV_MODE_IDLE BLE_ADV_MODE_FAST BLE_ADV_MODE_DIRECTED_SLOW		
			break;			
	}
	
	    for (;;)
    {
			app_sched_execute();
        power_manage();
			app_updata_value();
    }
}

/** 
 * @}
 */
