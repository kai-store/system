/******************************************************************************
 *
 *  Copyright (C) 2014 The Android Open Source Project
 *  Copyright (C) 2009-2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/************************************************************************************
 *
 *  Filename:      btif_core.c
 *
 *  Description:   Contains core functionality related to interfacing between
 *                 Bluetooth HAL and BTE core stack.
 *
 ***********************************************************************************/

#include <ctype.h>
#include <cutils/properties.h>
#include <dirent.h>
#include <fcntl.h>
#include <hardware/bluetooth.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#define LOG_TAG "bt_btif_core"
#include "btcore/include/bdaddr.h"

#include "bdaddr.h"
#include "bt_utils.h"
#include "bta_api.h"
#include "bte.h"
#include "btif_api.h"
#include "btif_av.h"
#include "btif_config.h"
#include "btif_pan.h"
#include "btif_profile_queue.h"
#include "btif_config.h"
#include "btif_sock.h"
#include "btif_storage.h"
#include "btif_util.h"
#include "btu.h"
#include "device/include/controller.h"
#include "osi/include/fixed_queue.h"
#include "osi/include/future.h"
#include "gki.h"
#include "osi/include/osi.h"
#include "osi/include/log.h"
#include "stack_manager.h"
#include "osi/include/thread.h"

/************************************************************************************
**  Constants & Macros
************************************************************************************/

#ifndef BTE_DID_CONF_FILE
#define BTE_DID_CONF_FILE "/etc/bluetooth/bt_did.conf"
#endif

/************************************************************************************
**  Local type definitions
************************************************************************************/

/* These type definitions are used when passing data from the HAL to BTIF context
*  in the downstream path for the adapter and remote_device property APIs */

typedef struct {
  bt_bdaddr_t bd_addr;
  bt_property_type_t type;
} btif_storage_read_t;

typedef struct {
  bt_bdaddr_t bd_addr;
  bt_property_t prop;
} btif_storage_write_t;

typedef union {
  btif_storage_read_t read_req;
  btif_storage_write_t write_req;
} btif_storage_req_t;

typedef enum {
    BTIF_CORE_STATE_DISABLED = 0,
    BTIF_CORE_STATE_ENABLING,
    BTIF_CORE_STATE_ENABLED,
#if defined (BOARD_HAVE_FM_BCM)
    BTIF_CORE_STATE_RADIO_ENABLED,
#endif
    BTIF_CORE_STATE_DISABLING
} btif_core_state_t;

/************************************************************************************
**  Static variables
************************************************************************************/

bt_bdaddr_t btif_local_bd_addr;

#if defined (BOARD_HAVE_FM_BCM)
static btif_core_state_t btif_core_state = BTIF_CORE_STATE_DISABLED;
/* holds whther it is radio request (enable_radio/disable_radio) */
static BOOLEAN btif_core_is_radio_req = FALSE;

/* holds the count of radios that are enabled */
static int btif_core_radio_ref_count = 0;
#endif
static tBTA_SERVICE_MASK btif_enabled_services = 0;
#if defined (BOARD_HAVE_FM_BCM)
static int btif_shutdown_pending = 0;
#endif

/*
* This variable should be set to 1, if the Bluedroid+BTIF libraries are to
* function in DUT mode.
*
* To set this, the btif_init_bluetooth needs to be called with argument as 1
*/
static UINT8 btif_dut_mode = 0;

static thread_t *bt_jni_workqueue_thread;
static const char *BT_JNI_WORKQUEUE_NAME = "bt_jni_workqueue";

/************************************************************************************
**  Static functions
************************************************************************************/
static void btif_jni_associate(UNUSED_ATTR uint16_t event, UNUSED_ATTR char *p_param);
static void btif_jni_disassociate(UNUSED_ATTR uint16_t event, UNUSED_ATTR char *p_param);

/* sends message to btif task */
static void btif_sendmsg(void *p_msg);

/************************************************************************************
**  Externs
************************************************************************************/
extern fixed_queue_t *btu_hci_msg_queue;

extern void bte_load_did_conf(const char *p_path);

/** TODO: Move these to _common.h */
void bte_main_boot_entry(void);
void bte_main_disable(void);
void bte_main_shutdown(void);
#if (defined(HCILP_INCLUDED) && HCILP_INCLUDED == TRUE)
void bte_main_enable_lpm(BOOLEAN enable);
#endif
void bte_main_postload_cfg(void);
void btif_dm_execute_service_request(UINT16 event, char *p_param);
#ifdef BTIF_DM_OOB_TEST
void btif_dm_load_local_oob(void);
#endif
void bte_main_config_hci_logging(BOOLEAN enable, BOOLEAN bt_disabled);
#if defined (BOARD_HAVE_FM_BCM)
void btif_handle_bluetooth_enable_evt(tBTA_STATUS status);

extern void stack_callback(bt_state_t state);
#endif

/*******************************************************************************
**
** Function         btif_context_switched
**
** Description      Callback used to execute transferred context callback
**
**                  p_msg : message to be executed in btif context
**
** Returns          void
**
*******************************************************************************/

static void btif_context_switched(void *p_msg)
{

    BTIF_TRACE_VERBOSE("btif_context_switched");

    tBTIF_CONTEXT_SWITCH_CBACK *p = (tBTIF_CONTEXT_SWITCH_CBACK *) p_msg;

    /* each callback knows how to parse the data */
    if (p->p_cb)
        p->p_cb(p->event, p->p_param);
}


/*******************************************************************************
**
** Function         btif_transfer_context
**
** Description      This function switches context to btif task
**
**                  p_cback   : callback used to process message in btif context
**                  event     : event id of message
**                  p_params  : parameter area passed to callback (copied)
**                  param_len : length of parameter area
**                  p_copy_cback : If set this function will be invoked for deep copy
**
** Returns          void
**
*******************************************************************************/

bt_status_t btif_transfer_context (tBTIF_CBACK *p_cback, UINT16 event, char* p_params, int param_len, tBTIF_COPY_CBACK *p_copy_cback)
{
    tBTIF_CONTEXT_SWITCH_CBACK *p_msg;

    BTIF_TRACE_VERBOSE("btif_transfer_context event %d, len %d", event, param_len);

    /* allocate and send message that will be executed in btif context */
    if ((p_msg = (tBTIF_CONTEXT_SWITCH_CBACK *) GKI_getbuf(sizeof(tBTIF_CONTEXT_SWITCH_CBACK) + param_len)) != NULL)
    {
        p_msg->hdr.event = BT_EVT_CONTEXT_SWITCH_EVT; /* internal event */
        p_msg->p_cb = p_cback;

        p_msg->event = event;                         /* callback event */

        /* check if caller has provided a copy callback to do the deep copy */
        if (p_copy_cback)
        {
            p_copy_cback(event, p_msg->p_param, p_params);
        }
        else if (p_params)
        {
            memcpy(p_msg->p_param, p_params, param_len);  /* callback parameter data */
        }

        btif_sendmsg(p_msg);
        return BT_STATUS_SUCCESS;
    }
    else
    {
        /* let caller deal with a failed allocation */
        return BT_STATUS_NOMEM;
    }
}

/*******************************************************************************
**
** Function         btif_is_dut_mode
**
** Description      checks if BTIF is currently in DUT mode
**
** Returns          1 if test mode, otherwize 0
**
*******************************************************************************/

UINT8 btif_is_dut_mode(void)
{
    return (btif_dut_mode == 1);
}

/*******************************************************************************
**
** Function         btif_is_enabled
**
** Description      checks if main adapter is fully enabled
**
** Returns          1 if fully enabled, otherwize 0
**
*******************************************************************************/

int btif_is_enabled(void)
{
    return ((!btif_is_dut_mode()) && (stack_manager_get_interface()->get_stack_is_running()));
}

#if defined (BOARD_HAVE_FM_BCM)
/*******************************************************************************
**
** Function         btif_is_radio_enabled
**
** Description      checks if main adapter is fully enabled
**
** Returns          1 if fully enabled, otherwize 0
**
*******************************************************************************/

int btif_is_radio_enabled(void)
{
    return ((!btif_is_dut_mode()) && ((btif_core_state == BTIF_CORE_STATE_ENABLED)
    ||(btif_core_state == BTIF_CORE_STATE_RADIO_ENABLED)));
}
#endif

void btif_init_ok(UNUSED_ATTR uint16_t event, UNUSED_ATTR char *p_param) {
  BTIF_TRACE_DEBUG("btif_task: received trigger stack init event");
#if (BLE_INCLUDED == TRUE)
  btif_dm_load_ble_local_keys();
#endif
  BTA_EnableBluetooth(bte_dm_evt);
}

void btif_init_fail(UNUSED_ATTR uint16_t event, UNUSED_ATTR char *p_param) {
  BTIF_TRACE_DEBUG("btif_task: hardware init failed");
  bte_main_disable();
  btif_queue_release();
  bte_main_shutdown();
  btif_dut_mode = 0;

  future_ready(stack_manager_get_hack_future(), FUTURE_FAIL);
}

/*******************************************************************************
**
** Function         btif_task
**
** Description      BTIF task handler managing all messages being passed
**                  Bluetooth HAL and BTA.
**
** Returns          void
**
*******************************************************************************/
static void bt_jni_msg_ready(void *context) {
  BT_HDR *p_msg = (BT_HDR *)context;

  BTIF_TRACE_VERBOSE("btif task fetched event %x", p_msg->event);

  switch (p_msg->event) {
    case BT_EVT_CONTEXT_SWITCH_EVT:
      btif_context_switched(p_msg);
      break;
    default:
      BTIF_TRACE_ERROR("unhandled btif event (%d)", p_msg->event & BT_EVT_MASK);
      break;
  }
  GKI_freebuf(p_msg);
}

/*******************************************************************************
**
** Function         btif_sendmsg
**
** Description      Sends msg to BTIF task
**
** Returns          void
**
*******************************************************************************/

void btif_sendmsg(void *p_msg)
{
    thread_post(bt_jni_workqueue_thread, bt_jni_msg_ready, p_msg);
}

void btif_thread_post(thread_fn func, void *context) {
    thread_post(bt_jni_workqueue_thread, func, context);
}

static void btif_fetch_local_bdaddr(bt_bdaddr_t *local_addr)
{
    char val[256];
    uint8_t valid_bda = FALSE;
    int val_size = 0;
    const uint8_t null_bdaddr[BD_ADDR_LEN] = {0,0,0,0,0,0};

    /* Get local bdaddr storage path from property */
    if (property_get(PROPERTY_BT_BDADDR_PATH, val, NULL))
    {
        int addr_fd;

        BTIF_TRACE_DEBUG("local bdaddr is stored in %s", val);

        if ((addr_fd = open(val, O_RDONLY)) != -1)
        {
            memset(val, 0, sizeof(val));
            read(addr_fd, val, FACTORY_BT_BDADDR_STORAGE_LEN);
            string_to_bdaddr(val, local_addr);
            /* If this is not a reserved/special bda, then use it */
            if (memcmp(local_addr->address, null_bdaddr, BD_ADDR_LEN) != 0)
            {
                valid_bda = TRUE;
                BTIF_TRACE_DEBUG("Got Factory BDA %02X:%02X:%02X:%02X:%02X:%02X",
                    local_addr->address[0], local_addr->address[1], local_addr->address[2],
                    local_addr->address[3], local_addr->address[4], local_addr->address[5]);
            }

            close(addr_fd);
        }
    }

    if(!valid_bda)
    {
        val_size = sizeof(val);
        if(btif_config_get_str("Adapter", "Address", val, &val_size))
        {
            string_to_bdaddr(val, local_addr);
            BTIF_TRACE_DEBUG("local bdaddr from bt_config.xml is  %s", val);
            return;
        }
     }

    /* No factory BDADDR found. Look for previously generated random BDA */
    if ((!valid_bda) && \
        (property_get(PERSIST_BDADDR_PROPERTY, val, NULL)))
    {
        string_to_bdaddr(val, local_addr);
        valid_bda = TRUE;
        BTIF_TRACE_DEBUG("Got prior random BDA %02X:%02X:%02X:%02X:%02X:%02X",
            local_addr->address[0], local_addr->address[1], local_addr->address[2],
            local_addr->address[3], local_addr->address[4], local_addr->address[5]);
    }

    /* Generate new BDA if necessary */
    if (!valid_bda)
    {
        bdstr_t bdstr;
        /* Seed the random number generator */
        srand((unsigned int) (time(0)));

        /* No autogen BDA. Generate one now. */
        local_addr->address[0] = 0x22;
        local_addr->address[1] = 0x22;
        local_addr->address[2] = (uint8_t) ((rand() >> 8) & 0xFF);
        local_addr->address[3] = (uint8_t) ((rand() >> 8) & 0xFF);
        local_addr->address[4] = (uint8_t) ((rand() >> 8) & 0xFF);
        local_addr->address[5] = (uint8_t) ((rand() >> 8) & 0xFF);

        /* Convert to ascii, and store as a persistent property */
        bdaddr_to_string(local_addr, bdstr, sizeof(bdstr));

        BTIF_TRACE_DEBUG("No preset BDA. Generating BDA: %s for prop %s",
             (char*)bdstr, PERSIST_BDADDR_PROPERTY);

        if (property_set(PERSIST_BDADDR_PROPERTY, (char*)bdstr) < 0)
            BTIF_TRACE_ERROR("Failed to set random BDA in prop %s",PERSIST_BDADDR_PROPERTY);
    }

    //save the bd address to config file
    bdstr_t bdstr;
    bdaddr_to_string(local_addr, bdstr, sizeof(bdstr));
    val_size = sizeof(val);
    if (btif_config_get_str("Adapter", "Address", val, &val_size))
    {
        if (strcmp(bdstr, val) ==0)
        {
            // BDA is already present in the config file.
            return;
        }
    }
    btif_config_set_str("Adapter", "Address", bdstr);
}

/*******************************************************************************
**
** Function         btif_init_bluetooth
**
** Description      Creates BTIF task and prepares BT scheduler for startup
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_init_bluetooth() {
  bte_main_boot_entry();

  /* As part of the init, fetch the local BD ADDR */
  memset(&btif_local_bd_addr, 0, sizeof(bt_bdaddr_t));
  btif_fetch_local_bdaddr(&btif_local_bd_addr);

  bt_jni_workqueue_thread = thread_new(BT_JNI_WORKQUEUE_NAME);
  if (bt_jni_workqueue_thread == NULL) {
    LOG_ERROR("%s Unable to create thread %s", __func__, BT_JNI_WORKQUEUE_NAME);
    goto error_exit;
  }

  // Associate this workqueue thread with jni.
  btif_transfer_context(btif_jni_associate, 0, NULL, 0, NULL);

  return BT_STATUS_SUCCESS;

error_exit:;
     thread_free(bt_jni_workqueue_thread);

     bt_jni_workqueue_thread = NULL;

     return BT_STATUS_FAIL;
}


#if defined (BOARD_HAVE_FM_BCM)
static void btif_in_generic_evt(UINT16 event, char * params)
{
     switch(event) {
        case BTIF_CORE_BT_RADIO_ON:			
            btif_handle_bluetooth_enable_evt(BTA_SUCCESS);
            break;
        case BTIF_CORE_BT_STATE_ON:
            /* enable bt services  */			
            btif_dm_enable_bt_services();
            btif_handle_bluetooth_enable_evt(BTA_SUCCESS);
            break;
        case BTIF_CORE_BT_RADIO_OFF:
			btif_disable_bluetooth_evt();
            break;
        case BTIF_CORE_BT_STATE_OFF:
            btif_check_send_bt_off();
			btif_disable_bluetooth_evt();
            break;
     }
}


bt_status_t btif_enable_bluetooth(void)
{
    if( 0 == btif_core_radio_ref_count)
    {
        if(btif_core_state != BTIF_CORE_STATE_DISABLED)
        {
		    BTIF_TRACE_DEBUG("not disabled\n");
			return BT_STATUS_DONE;
        }
		// Include this for now to put btif config into a shutdown-able state
        module_start_up(get_module(BTIF_CONFIG_MODULE));
		btif_core_state = BTIF_CORE_STATE_ENABLING;
		bte_main_enable();
		btif_core_radio_ref_count++;
    }
	else{
		btif_core_radio_ref_count++;
		/*btif core/chip is already enabled so just do other initialisation according to event*/
        btif_transfer_context(btif_in_generic_evt, BTIF_CORE_BT_STATE_ON, NULL, 0, NULL);
	}
	
	
	return BT_STATUS_SUCCESS;
}

bt_status_t btif_enable_radio(void)
{
    btif_core_is_radio_req = TRUE;
    if( 0 == btif_core_radio_ref_count)
    {
        if(btif_core_state != BTIF_CORE_STATE_DISABLED)
        {
		    BTIF_TRACE_DEBUG("not disabled\n");
			return BT_STATUS_DONE;
        }
		// Include this for now to put btif config into a shutdown-able state
        module_start_up(get_module(BTIF_CONFIG_MODULE));
		btif_core_state = BTIF_CORE_STATE_ENABLING;
		bte_main_enable();
		btif_core_radio_ref_count++;
    }
	else{
		btif_core_radio_ref_count++;
		/*btif core/chip is already enabled so just do other initialisation according to event*/
        btif_transfer_context(btif_in_generic_evt, BTIF_CORE_BT_RADIO_ON, NULL, 0, NULL);
	}

	return BT_STATUS_SUCCESS;
}

/*******************************************************************************
**
** Function         btif_handle_bluetooth_enable_evt
**
** Description      Handles the module initialization for Bt/radio enabling
**
** Returns          void
**
*******************************************************************************/

void btif_handle_bluetooth_enable_evt(tBTA_STATUS status)
{
    /* callback to HAL */
    if (status == BTA_SUCCESS)
    {
        if (!btif_core_is_radio_req)
        {
            /* init rfcomm & l2cap api */
            btif_sock_init();

            /* init pan */
            btif_pan_init();

            /* load did configuration */
            bte_load_did_conf(BTE_DID_CONF_FILE);

#ifdef BTIF_DM_OOB_TEST
            btif_dm_load_local_oob();
#endif
            /* now fully enabled, update state */
            btif_core_state = BTIF_CORE_STATE_ENABLED;
            future_ready(stack_manager_get_hack_future(), FUTURE_SUCCESS);
			//stack_callback(BT_STATE_ON);
            //HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_STATE_ON);
        }
        else
        {
            /* If the chip is turned on go to BTIF_CORE_STATE_RADIO_ENABLED else
               stay in prev state  BTIF_CORE_STATE_ENABLED(bt/radio both enabled)*/
            if (btif_core_state == BTIF_CORE_STATE_ENABLING)
                btif_core_state = BTIF_CORE_STATE_RADIO_ENABLED;
            //stack_callback(BT_RADIO_ON);
            future_ready(stack_manager_get_hack_future(), FUTURE_SUCCESS);
            //HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_RADIO_ON);
            btif_core_is_radio_req =FALSE;
        }
    }
    else
    {
        if (!btif_core_is_radio_req)
        {
            /* cleanup rfcomm & l2cap api */
            btif_sock_cleanup();

            btif_pan_cleanup();

            /* we failed to enable, reset state */
            btif_core_state = BTIF_CORE_STATE_DISABLED;
			//stack_callback(BT_STATE_OFF);
			future_ready(stack_manager_get_hack_future(), FUTURE_FAIL);
            //HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_STATE_OFF);
        }
        else
        {
            /* we failed to enable, reset state */
            btif_core_state = BTIF_CORE_STATE_DISABLED;
			//stack_callback(BT_RADIO_OFF);
            //HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_RADIO_OFF);
            btif_core_is_radio_req =FALSE;
        }
    }
}

/*******************************************************************************
**
** Function         btif_check_send_bt_off
**
** Description     If bt is disabled when radio is still on , the bt off event to be sent to app.
**
** Returns          void
**
*******************************************************************************/
void btif_check_send_bt_off()
{
    BTIF_TRACE_DEBUG("btif_check_send_bt_off btif_core_state= (%d),ACL links = (%d)",
        btif_core_state, BTM_GetNumAclLinks());

    // BT OFF event is sent only when all the ACL links are down and
    // BT is OFF, but Radio is enabled (FM/GPS/NFC).

    if (btif_core_state == BTIF_CORE_STATE_RADIO_ENABLED
        && (BTM_GetNumAclLinks() == 0) )
    {
        //stack_callback(BT_STATE_OFF);
        //HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_STATE_OFF);
    }
}

bt_status_t btif_disable_radio(int * ref_count)
{
    tBTA_STATUS status;
	
    btif_core_is_radio_req = TRUE;
    btif_core_radio_ref_count--;
    
	*ref_count = btif_core_radio_ref_count;

    if (0 == btif_core_radio_ref_count){

        if (!btif_is_radio_enabled())
        {
            BTIF_TRACE_ERROR("btif_disable_radio : not yet enabled");
            return BT_STATUS_NOT_READY;
        }

        BTIF_TRACE_DEBUG("BTIF DISABLE RADIO");

        btif_core_state = BTIF_CORE_STATE_DISABLING;
        status = BTA_DisableBluetooth();

        if (status != BTA_SUCCESS)
        {
            BTIF_TRACE_ERROR("disable radio failed (%d)", status);

            /* reset the original state to allow attempting disable again */
            btif_core_state = BTIF_CORE_STATE_RADIO_ENABLED;

            return BT_STATUS_FAIL;
        }
    }
	else{
	   /* btif core/chip should not be turned off as ref is
           not 0 so just handle closure for radio modules if any*/
        btif_transfer_context(btif_in_generic_evt,
            BTIF_CORE_BT_RADIO_OFF, NULL, 0, NULL);
	}

    return BT_STATUS_SUCCESS;
}
#endif

/*******************************************************************************
**
** Function         btif_enable_bluetooth_evt
**
** Description      Event indicating bluetooth enable is completed
**                  Notifies HAL user with updated adapter state
**
** Returns          void
**
*******************************************************************************/

void btif_enable_bluetooth_evt(tBTA_STATUS status)
{
    const controller_t *controller = controller_get_interface();
    bdstr_t bdstr;
    bdaddr_to_string(controller->get_address(), bdstr, sizeof(bdstr));

    BTIF_TRACE_DEBUG("%s: status %d, local bd [%s]", __FUNCTION__, status, bdstr);

#if defined (BOARD_HAVE_FM_BCM)
	/* enable bt services if it is not radio req */
    if (!btif_core_is_radio_req)
        btif_dm_enable_bt_services();
#endif

    if (bdcmp(btif_local_bd_addr.address, controller->get_address()->address))
    {
        // TODO(zachoverflow): this whole code path seems like a bad time waiting to happen
        // We open the vendor library using the old address.
        bdstr_t old_address;
        bt_property_t prop;

        bdaddr_to_string(&btif_local_bd_addr, old_address, sizeof(old_address));

        /**
         * The Controller's BDADDR does not match to the BTIF's initial BDADDR!
         * This could be because the factory BDADDR was stored separately in
         * the Controller's non-volatile memory rather than in device's file
         * system.
         **/
        BTIF_TRACE_WARNING("***********************************************");
        BTIF_TRACE_WARNING("BTIF init BDA was %s", old_address);
        BTIF_TRACE_WARNING("Controller BDA is %s", bdstr);
        BTIF_TRACE_WARNING("***********************************************");

        btif_local_bd_addr = *controller->get_address();

        //save the bd address to config file
        btif_config_set_str("Adapter", "Address", bdstr);
        btif_config_save();

        //fire HAL callback for property change
        prop.type = BT_PROPERTY_BDADDR;
        prop.val = (void*)&btif_local_bd_addr;
        prop.len = sizeof(bt_bdaddr_t);
        HAL_CBACK(bt_hal_cbacks, adapter_properties_cb, BT_STATUS_SUCCESS, 1, &prop);
    }

    bte_main_postload_cfg();
#if (defined(HCILP_INCLUDED) && HCILP_INCLUDED == TRUE)
    bte_main_enable_lpm(TRUE);
#endif
    /* add passing up bd address as well ? */

#if defined (BOARD_HAVE_FM_BCM)
	btif_handle_bluetooth_enable_evt(status);
#else
    /* callback to HAL */
    if (status == BTA_SUCCESS)
    {
        /* init rfcomm & l2cap api */
        btif_sock_init();

        /* init pan */
        btif_pan_init();

        /* load did configuration */
        bte_load_did_conf(BTE_DID_CONF_FILE);

#ifdef BTIF_DM_OOB_TEST
        btif_dm_load_local_oob();
#endif

        future_ready(stack_manager_get_hack_future(), FUTURE_SUCCESS);
    }
    else
    {
        /* cleanup rfcomm & l2cap api */
        btif_sock_cleanup();

        btif_pan_cleanup();

        future_ready(stack_manager_get_hack_future(), FUTURE_FAIL);
    }
#endif
}

#if defined (BOARD_HAVE_FM_BCM)
/*******************************************************************************
**
** Function         btif_disable_bluetooth
**
** Description      Inititates shutdown of Bluetooth system.
**                  Any active links will be dropped and device entering
**                  non connectable/discoverable mode
**
** Returns          void
**
*******************************************************************************/
bt_status_t btif_disable_bluetooth(int* ref_count)
{
    tBTA_STATUS status;

    BTIF_TRACE_DEBUG("BTIF DISABLE BLUETOOTH");
	btif_core_radio_ref_count--;
	*ref_count = btif_core_radio_ref_count;
	if (0 == btif_core_radio_ref_count)
    {
		if (!btif_is_enabled())
    	{
        	BTIF_TRACE_ERROR("btif_disable_bluetooth : not yet enabled");
        	return BT_STATUS_NOT_READY;
    	}
		
       btif_core_state = BTIF_CORE_STATE_DISABLING;
       btif_dm_on_disable();
       /* cleanup rfcomm & l2cap api */
       btif_sock_cleanup();
       btif_pan_cleanup();
       status = BTA_DisableBluetooth();
	   if (status != BTA_SUCCESS)
       {
          BTIF_TRACE_ERROR("disable bt failed (%d)", status);

          /* reset the original state to allow attempting disable again */
           btif_core_state = BTIF_CORE_STATE_ENABLED;

           return BT_STATUS_FAIL;
        }	   
	}
	else
    {
        /*btif core/chip should not be turned off as ref is not 0
        so just handle closure for bt modules*/
        BTIF_TRACE_DEBUG("BTIF DISABLE BLUETOOTH MODULES");

        btif_dm_on_disable();
        btif_core_state = BTIF_CORE_STATE_RADIO_ENABLED;


        /* cleanup rfcomm & l2cap api */
        btif_sock_cleanup();
        btif_pan_cleanup();

        btif_transfer_context(btif_in_generic_evt, BTIF_CORE_BT_STATE_OFF, NULL, 0, NULL);
    }

    return BT_STATUS_SUCCESS;
}

/*******************************************************************************
**
** Function         btif_disable_bluetooth_evt
**
** Description      Event notifying BT disable is now complete.
**                  Terminates main stack tasks and notifies HAL
**                  user with updated BT state.
**
** Returns          void
**
*******************************************************************************/

void btif_disable_bluetooth_evt(void)
{
    BTIF_TRACE_DEBUG("%s", __FUNCTION__);

    if (!btif_core_is_radio_req)
	   btif_dm_disable_bt_services();

#if (defined(HCILP_INCLUDED) && HCILP_INCLUDED == TRUE)
    bte_main_enable_lpm(FALSE);
#endif

#if (BLE_INCLUDED == TRUE)
     BTA_VendorCleanup();
#endif
	  if (!btif_core_radio_ref_count)
		  bte_main_disable();
	  if (!btif_core_is_radio_req)
	  {
		 /* update local state */
		 btif_core_state = BTIF_CORE_STATE_DISABLED;	 
	 }
	 else
	 {
		 /* now fully disabled, update state */
		 btif_core_state = BTIF_CORE_STATE_DISABLED;
		 btif_core_is_radio_req = FALSE;
	 }
	 
	 if (btif_shutdown_pending)
	 {
		 BTIF_TRACE_DEBUG("%s: calling btif_shutdown_bluetooth", __FUNCTION__);
		 btif_shutdown_pending = 0;
		 //btif_shutdown_bluetooth();
	 }

    /* callback to HAL */
    future_ready(stack_manager_get_hack_future(), FUTURE_SUCCESS);
}

/*******************************************************************************
**
** Function         btif_shutdown_bluetooth
**
** Description      Finalizes BT scheduler shutdown and terminates BTIF
**                  task.
**
** Returns          void
**
*******************************************************************************/

bt_status_t btif_shutdown_bluetooth(void)
{
    int ref_count = 0;
    BTIF_TRACE_DEBUG("%s", __FUNCTION__);
	
	if (btif_core_state == BTIF_CORE_STATE_DISABLING)
	{
		 BTIF_TRACE_WARNING("shutdown during disabling");
		 /* shutdown called before disabling is done */
		 btif_shutdown_pending = 1;
		 return BT_STATUS_NOT_READY;
	}

	if (btif_is_radio_enabled())
    {
        BTIF_TRACE_WARNING("shutdown while still enabled, initiate disable");

        /* shutdown called prior to disabling, initiate disable */
        btif_disable_bluetooth(&ref_count);
        btif_shutdown_pending = 1;
        return BT_STATUS_NOT_READY;
    }
    
    btif_shutdown_pending = 0;
	if (btif_core_state == BTIF_CORE_STATE_ENABLING)
    {
        // Java layer abort BT ENABLING, could be due to ENABLE TIMEOUT
        // Direct call from cleanup()@bluetooth.c
        // bring down HCI/Vendor lib
        bte_main_disable();
        btif_core_radio_ref_count--;
        btif_core_is_radio_req = FALSE;
        btif_core_state = BTIF_CORE_STATE_DISABLED;
    }

    btif_transfer_context(btif_jni_disassociate, 0, NULL, 0, NULL);

    btif_queue_release();

    thread_free(bt_jni_workqueue_thread);
    bt_jni_workqueue_thread = NULL;

    bte_main_shutdown();

    btif_dut_mode = 0;

    BTIF_TRACE_DEBUG("%s done", __FUNCTION__);

    return BT_STATUS_SUCCESS;
}

#else
/*******************************************************************************
**
** Function         btif_disable_bluetooth
**
** Description      Inititates shutdown of Bluetooth system.
**                  Any active links will be dropped and device entering
**                  non connectable/discoverable mode
**
** Returns          void
**
*******************************************************************************/
bt_status_t btif_disable_bluetooth(void)
{
    BTIF_TRACE_DEBUG("BTIF DISABLE BLUETOOTH");

    btif_dm_on_disable();
    /* cleanup rfcomm & l2cap api */
    btif_sock_cleanup();
    btif_pan_cleanup();
    BTA_DisableBluetooth();

    return BT_STATUS_SUCCESS;
}

/*******************************************************************************
**
** Function         btif_disable_bluetooth_evt
**
** Description      Event notifying BT disable is now complete.
**                  Terminates main stack tasks and notifies HAL
**                  user with updated BT state.
**
** Returns          void
**
*******************************************************************************/

void btif_disable_bluetooth_evt(void)
{
    BTIF_TRACE_DEBUG("%s", __FUNCTION__);

#if (defined(HCILP_INCLUDED) && HCILP_INCLUDED == TRUE)
    bte_main_enable_lpm(FALSE);
#endif

#if (BLE_INCLUDED == TRUE)
     BTA_VendorCleanup();
#endif

     bte_main_disable();

    /* callback to HAL */
    future_ready(stack_manager_get_hack_future(), FUTURE_SUCCESS);
}

/*******************************************************************************
**
** Function         btif_shutdown_bluetooth
**
** Description      Finalizes BT scheduler shutdown and terminates BTIF
**                  task.
**
** Returns          void
**
*******************************************************************************/

bt_status_t btif_shutdown_bluetooth(void)
{
    BTIF_TRACE_DEBUG("%s", __FUNCTION__);

    btif_transfer_context(btif_jni_disassociate, 0, NULL, 0, NULL);

    btif_queue_release();

    thread_free(bt_jni_workqueue_thread);
    bt_jni_workqueue_thread = NULL;

    bte_main_shutdown();

    btif_dut_mode = 0;

    BTIF_TRACE_DEBUG("%s done", __FUNCTION__);

    return BT_STATUS_SUCCESS;
}
#endif

/*******************************************************************************
**
** Function         btif_dut_mode_cback
**
** Description     Callback invoked on completion of vendor specific test mode command
**
** Returns          None
**
*******************************************************************************/
static void btif_dut_mode_cback( tBTM_VSC_CMPL *p )
{
    UNUSED(p);
    /* For now nothing to be done. */
}

/*******************************************************************************
**
** Function         btif_dut_mode_configure
**
** Description      Configure Test Mode - 'enable' to 1 puts the device in test mode and 0 exits
**                       test mode
**
** Returns          BT_STATUS_SUCCESS on success
**
*******************************************************************************/
bt_status_t btif_dut_mode_configure(uint8_t enable)
{
    BTIF_TRACE_DEBUG("%s", __FUNCTION__);

    if (!stack_manager_get_interface()->get_stack_is_running()) {
        BTIF_TRACE_ERROR("btif_dut_mode_configure : Bluetooth not enabled");
        return BT_STATUS_NOT_READY;
    }

    btif_dut_mode = enable;
    if (enable == 1) {
        BTA_EnableTestMode();
    } else {
        BTA_DisableTestMode();
    }
    return BT_STATUS_SUCCESS;
}

/*******************************************************************************
**
** Function         btif_dut_mode_send
**
** Description     Sends a HCI Vendor specific command to the controller
**
** Returns          BT_STATUS_SUCCESS on success
**
*******************************************************************************/
bt_status_t btif_dut_mode_send(uint16_t opcode, uint8_t *buf, uint8_t len)
{
    /* TODO: Check that opcode is a vendor command group */
    BTIF_TRACE_DEBUG("%s", __FUNCTION__);
    if (!btif_is_dut_mode()) {
         BTIF_TRACE_ERROR("Bluedroid HAL needs to be init with test_mode set to 1.");
         return BT_STATUS_FAIL;
    }
    BTM_VendorSpecificCommand(opcode, len, buf, btif_dut_mode_cback);
    return BT_STATUS_SUCCESS;
}

/*****************************************************************************
**
**   btif api adapter property functions
**
*****************************************************************************/

static bt_status_t btif_in_get_adapter_properties(void)
{
    bt_property_t properties[6];
    uint32_t num_props;

    bt_bdaddr_t addr;
    bt_bdname_t name;
    bt_scan_mode_t mode;
    uint32_t disc_timeout;
    bt_bdaddr_t bonded_devices[BTM_SEC_MAX_DEVICE_RECORDS];
    bt_uuid_t local_uuids[BT_MAX_NUM_UUIDS];
    num_props = 0;

    /* BD_ADDR */
    BTIF_STORAGE_FILL_PROPERTY(&properties[num_props], BT_PROPERTY_BDADDR,
                               sizeof(addr), &addr);
    btif_storage_get_adapter_property(&properties[num_props]);
    num_props++;

    /* BD_NAME */
    BTIF_STORAGE_FILL_PROPERTY(&properties[num_props], BT_PROPERTY_BDNAME,
                               sizeof(name), &name);
    btif_storage_get_adapter_property(&properties[num_props]);
    num_props++;

    /* SCAN_MODE */
    BTIF_STORAGE_FILL_PROPERTY(&properties[num_props], BT_PROPERTY_ADAPTER_SCAN_MODE,
                               sizeof(mode), &mode);
    btif_storage_get_adapter_property(&properties[num_props]);
    num_props++;

    /* DISC_TIMEOUT */
    BTIF_STORAGE_FILL_PROPERTY(&properties[num_props], BT_PROPERTY_ADAPTER_DISCOVERY_TIMEOUT,
                               sizeof(disc_timeout), &disc_timeout);
    btif_storage_get_adapter_property(&properties[num_props]);
    num_props++;

    /* BONDED_DEVICES */
    BTIF_STORAGE_FILL_PROPERTY(&properties[num_props], BT_PROPERTY_ADAPTER_BONDED_DEVICES,
                               sizeof(bonded_devices), bonded_devices);
    btif_storage_get_adapter_property(&properties[num_props]);
    num_props++;

    /* LOCAL UUIDs */
    BTIF_STORAGE_FILL_PROPERTY(&properties[num_props], BT_PROPERTY_UUIDS,
                               sizeof(local_uuids), local_uuids);
    btif_storage_get_adapter_property(&properties[num_props]);
    num_props++;

    HAL_CBACK(bt_hal_cbacks, adapter_properties_cb,
                     BT_STATUS_SUCCESS, num_props, properties);

    return BT_STATUS_SUCCESS;
}

static bt_status_t btif_in_get_remote_device_properties(bt_bdaddr_t *bd_addr)
{
    bt_property_t remote_properties[8];
    uint32_t num_props = 0;

    bt_bdname_t name, alias;
    uint32_t cod, devtype;
    bt_uuid_t remote_uuids[BT_MAX_NUM_UUIDS];

    memset(remote_properties, 0, sizeof(remote_properties));
    BTIF_STORAGE_FILL_PROPERTY(&remote_properties[num_props], BT_PROPERTY_BDNAME,
                               sizeof(name), &name);
    btif_storage_get_remote_device_property(bd_addr,
                                            &remote_properties[num_props]);
    num_props++;

    BTIF_STORAGE_FILL_PROPERTY(&remote_properties[num_props], BT_PROPERTY_REMOTE_FRIENDLY_NAME,
                               sizeof(alias), &alias);
    btif_storage_get_remote_device_property(bd_addr,
                                            &remote_properties[num_props]);
    num_props++;

    BTIF_STORAGE_FILL_PROPERTY(&remote_properties[num_props], BT_PROPERTY_CLASS_OF_DEVICE,
                               sizeof(cod), &cod);
    btif_storage_get_remote_device_property(bd_addr,
                                            &remote_properties[num_props]);
    num_props++;

    BTIF_STORAGE_FILL_PROPERTY(&remote_properties[num_props], BT_PROPERTY_TYPE_OF_DEVICE,
                               sizeof(devtype), &devtype);
    btif_storage_get_remote_device_property(bd_addr,
                                            &remote_properties[num_props]);
    num_props++;

    BTIF_STORAGE_FILL_PROPERTY(&remote_properties[num_props], BT_PROPERTY_UUIDS,
                               sizeof(remote_uuids), remote_uuids);
    btif_storage_get_remote_device_property(bd_addr,
                                            &remote_properties[num_props]);
    num_props++;

    HAL_CBACK(bt_hal_cbacks, remote_device_properties_cb,
                     BT_STATUS_SUCCESS, bd_addr, num_props, remote_properties);

    return BT_STATUS_SUCCESS;
}


/*******************************************************************************
**
** Function         execute_storage_request
**
** Description      Executes adapter storage request in BTIF context
**
** Returns          bt_status_t
**
*******************************************************************************/

static void execute_storage_request(UINT16 event, char *p_param)
{
    bt_status_t status = BT_STATUS_SUCCESS;

    BTIF_TRACE_EVENT("execute storage request event : %d", event);

    switch(event)
    {
        case BTIF_CORE_STORAGE_ADAPTER_WRITE:
        {
            btif_storage_req_t *p_req = (btif_storage_req_t*)p_param;
            bt_property_t *p_prop = &(p_req->write_req.prop);
            BTIF_TRACE_EVENT("type: %d, len %d, 0x%x", p_prop->type,
                               p_prop->len, p_prop->val);

            status = btif_storage_set_adapter_property(p_prop);
            HAL_CBACK(bt_hal_cbacks, adapter_properties_cb, status, 1, p_prop);
        } break;

        case BTIF_CORE_STORAGE_ADAPTER_READ:
        {
            btif_storage_req_t *p_req = (btif_storage_req_t*)p_param;
            char buf[512];
            bt_property_t prop;
            prop.type = p_req->read_req.type;
            prop.val = (void*)buf;
            prop.len = sizeof(buf);
            if (prop.type == BT_PROPERTY_LOCAL_LE_FEATURES)
            {
                #if (BLE_INCLUDED == TRUE)
                tBTM_BLE_VSC_CB cmn_vsc_cb;
                bt_local_le_features_t local_le_features;

                /* LE features are not stored in storage. Should be retrived from stack */
                BTM_BleGetVendorCapabilities(&cmn_vsc_cb);
                local_le_features.local_privacy_enabled = BTM_BleLocalPrivacyEnabled();

                prop.len = sizeof (bt_local_le_features_t);
                if (cmn_vsc_cb.filter_support == 1)
                    local_le_features.max_adv_filter_supported = cmn_vsc_cb.max_filter;
                else
                    local_le_features.max_adv_filter_supported = 0;
                local_le_features.max_adv_instance = cmn_vsc_cb.adv_inst_max;
                local_le_features.max_irk_list_size = cmn_vsc_cb.max_irk_list_sz;
                local_le_features.rpa_offload_supported = cmn_vsc_cb.rpa_offloading;
                local_le_features.scan_result_storage_size = cmn_vsc_cb.tot_scan_results_strg;
                local_le_features.activity_energy_info_supported = cmn_vsc_cb.energy_support;
                local_le_features.version_supported = cmn_vsc_cb.version_supported;
                local_le_features.total_trackable_advertisers =
                    cmn_vsc_cb.total_trackable_advertisers;

                local_le_features.extended_scan_support = cmn_vsc_cb.extended_scan_support > 0;
                local_le_features.debug_logging_supported = cmn_vsc_cb.debug_logging_supported > 0;
                memcpy(prop.val, &local_le_features, prop.len);
                #endif
            }
            else
            {
                status = btif_storage_get_adapter_property(&prop);
            }
            HAL_CBACK(bt_hal_cbacks, adapter_properties_cb, status, 1, &prop);
        } break;

        case BTIF_CORE_STORAGE_ADAPTER_READ_ALL:
        {
            status = btif_in_get_adapter_properties();
        } break;

        case BTIF_CORE_STORAGE_NOTIFY_STATUS:
        {
            HAL_CBACK(bt_hal_cbacks, adapter_properties_cb, status, 0, NULL);
        } break;

        default:
            BTIF_TRACE_ERROR("%s invalid event id (%d)", __FUNCTION__, event);
            break;
    }
}

static void execute_storage_remote_request(UINT16 event, char *p_param)
{
    bt_status_t status = BT_STATUS_FAIL;
    bt_property_t prop;

    BTIF_TRACE_EVENT("execute storage remote request event : %d", event);

    switch (event)
    {
        case BTIF_CORE_STORAGE_REMOTE_READ:
        {
            char buf[1024];
            btif_storage_req_t *p_req = (btif_storage_req_t*)p_param;
            prop.type = p_req->read_req.type;
            prop.val = (void*) buf;
            prop.len = sizeof(buf);

            status = btif_storage_get_remote_device_property(&(p_req->read_req.bd_addr),
                                                             &prop);
            HAL_CBACK(bt_hal_cbacks, remote_device_properties_cb,
                            status, &(p_req->read_req.bd_addr), 1, &prop);
        }break;
        case BTIF_CORE_STORAGE_REMOTE_WRITE:
        {
           btif_storage_req_t *p_req = (btif_storage_req_t*)p_param;
           status = btif_storage_set_remote_device_property(&(p_req->write_req.bd_addr),
                                                            &(p_req->write_req.prop));
        }break;
        case BTIF_CORE_STORAGE_REMOTE_READ_ALL:
        {
           btif_storage_req_t *p_req = (btif_storage_req_t*)p_param;
           btif_in_get_remote_device_properties(&p_req->read_req.bd_addr);
        }break;
    }
}

void btif_adapter_properties_evt(bt_status_t status, uint32_t num_props,
                                    bt_property_t *p_props)
{
    HAL_CBACK(bt_hal_cbacks, adapter_properties_cb,
                     status, num_props, p_props);

}
void btif_remote_properties_evt(bt_status_t status, bt_bdaddr_t *remote_addr,
                                   uint32_t num_props, bt_property_t *p_props)
{
    HAL_CBACK(bt_hal_cbacks, remote_device_properties_cb,
                     status, remote_addr, num_props, p_props);
}

/*******************************************************************************
**
** Function         btif_in_storage_request_copy_cb
**
** Description     Switch context callback function to perform the deep copy for
**                 both the adapter and remote_device property API
**
** Returns          None
**
*******************************************************************************/
static void btif_in_storage_request_copy_cb(UINT16 event,
                                                 char *p_new_buf, char *p_old_buf)
{
     btif_storage_req_t *new_req = (btif_storage_req_t*)p_new_buf;
     btif_storage_req_t *old_req = (btif_storage_req_t*)p_old_buf;

     BTIF_TRACE_EVENT("%s", __FUNCTION__);
     switch (event)
     {
         case BTIF_CORE_STORAGE_REMOTE_WRITE:
         case BTIF_CORE_STORAGE_ADAPTER_WRITE:
         {
             bdcpy(new_req->write_req.bd_addr.address, old_req->write_req.bd_addr.address);
             /* Copy the member variables one at a time */
             new_req->write_req.prop.type = old_req->write_req.prop.type;
             new_req->write_req.prop.len = old_req->write_req.prop.len;

             new_req->write_req.prop.val = (UINT8 *)(p_new_buf + sizeof(btif_storage_req_t));
             memcpy(new_req->write_req.prop.val, old_req->write_req.prop.val,
                    old_req->write_req.prop.len);
         }break;
     }
}

/*******************************************************************************
**
** Function         btif_get_adapter_properties
**
** Description      Fetch all available properties (local & remote)
**
** Returns          bt_status_t
**
*******************************************************************************/

bt_status_t btif_get_adapter_properties(void)
{
    BTIF_TRACE_EVENT("%s", __FUNCTION__);

    if (!btif_is_enabled())
        return BT_STATUS_NOT_READY;

    return btif_transfer_context(execute_storage_request,
                                 BTIF_CORE_STORAGE_ADAPTER_READ_ALL,
                                 NULL, 0, NULL);
}

/*******************************************************************************
**
** Function         btif_get_adapter_property
**
** Description      Fetches property value from local cache
**
** Returns          bt_status_t
**
*******************************************************************************/

bt_status_t btif_get_adapter_property(bt_property_type_t type)
{
    btif_storage_req_t req;

    BTIF_TRACE_EVENT("%s %d", __FUNCTION__, type);

    /* Allow get_adapter_property only for BDADDR and BDNAME if BT is disabled */
    if (!btif_is_enabled() && (type != BT_PROPERTY_BDADDR) && (type != BT_PROPERTY_BDNAME))
        return BT_STATUS_NOT_READY;

    memset(&(req.read_req.bd_addr), 0, sizeof(bt_bdaddr_t));
    req.read_req.type = type;

    return btif_transfer_context(execute_storage_request,
                                 BTIF_CORE_STORAGE_ADAPTER_READ,
                                (char*)&req, sizeof(btif_storage_req_t), NULL);
}

/*******************************************************************************
**
** Function         btif_set_adapter_property
**
** Description      Updates core stack with property value and stores it in
**                  local cache
**
** Returns          bt_status_t
**
*******************************************************************************/

bt_status_t btif_set_adapter_property(const bt_property_t *property)
{
    btif_storage_req_t req;
    bt_status_t status = BT_STATUS_SUCCESS;
    int storage_req_id = BTIF_CORE_STORAGE_NOTIFY_STATUS; /* default */
    char bd_name[BTM_MAX_LOC_BD_NAME_LEN +1];
    UINT16  name_len = 0;

    BTIF_TRACE_EVENT("btif_set_adapter_property type: %d, len %d, 0x%x",
                      property->type, property->len, property->val);

    if (!btif_is_enabled())
        return BT_STATUS_NOT_READY;

    switch(property->type)
    {
        case BT_PROPERTY_BDNAME:
            {
                name_len = property->len > BTM_MAX_LOC_BD_NAME_LEN ? BTM_MAX_LOC_BD_NAME_LEN:
                                                                     property->len;
                memcpy(bd_name,property->val, name_len);
                bd_name[name_len] = '\0';

                BTIF_TRACE_EVENT("set property name : %s", (char *)bd_name);

                BTA_DmSetDeviceName((char *)bd_name);

                storage_req_id = BTIF_CORE_STORAGE_ADAPTER_WRITE;
            }
            break;

        case BT_PROPERTY_ADAPTER_SCAN_MODE:
            {
                bt_scan_mode_t mode = *(bt_scan_mode_t*)property->val;
                tBTA_DM_DISC disc_mode;
                tBTA_DM_CONN conn_mode;

                switch(mode)
                {
                    case BT_SCAN_MODE_NONE:
                        disc_mode = BTA_DM_NON_DISC;
                        conn_mode = BTA_DM_NON_CONN;
                        break;

                    case BT_SCAN_MODE_CONNECTABLE:
                        disc_mode = BTA_DM_NON_DISC;
                        conn_mode = BTA_DM_CONN;
                        break;

                    case BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE:
                        disc_mode = BTA_DM_GENERAL_DISC;
                        conn_mode = BTA_DM_CONN;
                        break;

                    default:
                        BTIF_TRACE_ERROR("invalid scan mode (0x%x)", mode);
                        return BT_STATUS_PARM_INVALID;
                }

                BTIF_TRACE_EVENT("set property scan mode : %x", mode);

                BTA_DmSetVisibility(disc_mode, conn_mode, BTA_DM_IGNORE, BTA_DM_IGNORE);

                storage_req_id = BTIF_CORE_STORAGE_ADAPTER_WRITE;
            }
            break;
        case BT_PROPERTY_ADAPTER_DISCOVERY_TIMEOUT:
            {
                /* Nothing to do beside store the value in NV.  Java
                   will change the SCAN_MODE property after setting timeout,
                   if required */
                storage_req_id = BTIF_CORE_STORAGE_ADAPTER_WRITE;
            }
            break;
        case BT_PROPERTY_BDADDR:
        case BT_PROPERTY_UUIDS:
        case BT_PROPERTY_ADAPTER_BONDED_DEVICES:
        case BT_PROPERTY_REMOTE_FRIENDLY_NAME:
            /* no write support through HAL, these properties are only populated from BTA events */
            status = BT_STATUS_FAIL;
            break;
        default:
            BTIF_TRACE_ERROR("btif_get_adapter_property : invalid type %d",
            property->type);
            status = BT_STATUS_FAIL;
            break;
    }

    if (storage_req_id != BTIF_CORE_STORAGE_NO_ACTION)
    {
        /* pass on to storage for updating local database */

        memset(&(req.write_req.bd_addr), 0, sizeof(bt_bdaddr_t));
        memcpy(&(req.write_req.prop), property, sizeof(bt_property_t));

        return btif_transfer_context(execute_storage_request,
                                     storage_req_id,
                                     (char*)&req,
                                     sizeof(btif_storage_req_t)+property->len,
                                     btif_in_storage_request_copy_cb);
    }

    return status;

}

/*******************************************************************************
**
** Function         btif_get_remote_device_property
**
** Description      Fetches the remote device property from the NVRAM
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_get_remote_device_property(bt_bdaddr_t *remote_addr,
                                                 bt_property_type_t type)
{
    btif_storage_req_t req;

    if (!btif_is_enabled())
        return BT_STATUS_NOT_READY;

    memcpy(&(req.read_req.bd_addr), remote_addr, sizeof(bt_bdaddr_t));
    req.read_req.type = type;
    return btif_transfer_context(execute_storage_remote_request,
                                 BTIF_CORE_STORAGE_REMOTE_READ,
                                 (char*)&req, sizeof(btif_storage_req_t),
                                 NULL);
}

/*******************************************************************************
**
** Function         btif_get_remote_device_properties
**
** Description      Fetches all the remote device properties from NVRAM
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_get_remote_device_properties(bt_bdaddr_t *remote_addr)
{
    btif_storage_req_t req;

    if (!btif_is_enabled())
        return BT_STATUS_NOT_READY;

    memcpy(&(req.read_req.bd_addr), remote_addr, sizeof(bt_bdaddr_t));
    return btif_transfer_context(execute_storage_remote_request,
                                 BTIF_CORE_STORAGE_REMOTE_READ_ALL,
                                 (char*)&req, sizeof(btif_storage_req_t),
                                 NULL);
}

/*******************************************************************************
**
** Function         btif_set_remote_device_property
**
** Description      Writes the remote device property to NVRAM.
**                  Currently, BT_PROPERTY_REMOTE_FRIENDLY_NAME is the only
**                  remote device property that can be set
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_set_remote_device_property(bt_bdaddr_t *remote_addr,
                                                 const bt_property_t *property)
{
    btif_storage_req_t req;

    if (!btif_is_enabled())
        return BT_STATUS_NOT_READY;

    memcpy(&(req.write_req.bd_addr), remote_addr, sizeof(bt_bdaddr_t));
    memcpy(&(req.write_req.prop), property, sizeof(bt_property_t));

    return btif_transfer_context(execute_storage_remote_request,
                                 BTIF_CORE_STORAGE_REMOTE_WRITE,
                                 (char*)&req,
                                 sizeof(btif_storage_req_t)+property->len,
                                 btif_in_storage_request_copy_cb);
}


/*******************************************************************************
**
** Function         btif_get_remote_service_record
**
** Description      Looks up the service matching uuid on the remote device
**                  and fetches the SCN and service_name if the UUID is found
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_get_remote_service_record(bt_bdaddr_t *remote_addr,
                                               bt_uuid_t *uuid)
{
    if (!btif_is_enabled())
        return BT_STATUS_NOT_READY;

    return btif_dm_get_remote_service_record(remote_addr, uuid);
}


/*******************************************************************************
**
** Function         btif_get_enabled_services_mask
**
** Description      Fetches currently enabled services
**
** Returns          tBTA_SERVICE_MASK
**
*******************************************************************************/

tBTA_SERVICE_MASK btif_get_enabled_services_mask(void)
{
    return btif_enabled_services;
}

/*******************************************************************************
**
** Function         btif_enable_service
**
** Description      Enables the service 'service_ID' to the service_mask.
**                  Upon BT enable, BTIF core shall invoke the BTA APIs to
**                  enable the profiles
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_enable_service(tBTA_SERVICE_ID service_id)
{
    tBTA_SERVICE_ID *p_id = &service_id;

    /* If BT is enabled, we need to switch to BTIF context and trigger the
     * enable for that profile
     *
     * Otherwise, we just set the flag. On BT_Enable, the DM will trigger
     * enable for the profiles that have been enabled */

    btif_enabled_services |= (1 << service_id);

    BTIF_TRACE_DEBUG("%s: current services:0x%x", __FUNCTION__, btif_enabled_services);

    if (btif_is_enabled())
    {
        btif_transfer_context(btif_dm_execute_service_request,
                              BTIF_DM_ENABLE_SERVICE,
                              (char*)p_id, sizeof(tBTA_SERVICE_ID), NULL);
    }

    return BT_STATUS_SUCCESS;
}
/*******************************************************************************
**
** Function         btif_disable_service
**
** Description      Disables the service 'service_ID' to the service_mask.
**                  Upon BT disable, BTIF core shall invoke the BTA APIs to
**                  disable the profiles
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_disable_service(tBTA_SERVICE_ID service_id)
{
    tBTA_SERVICE_ID *p_id = &service_id;

    /* If BT is enabled, we need to switch to BTIF context and trigger the
     * disable for that profile so that the appropriate uuid_property_changed will
     * be triggerred. Otherwise, we just need to clear the service_id in the mask
     */

    btif_enabled_services &=  (tBTA_SERVICE_MASK)(~(1<<service_id));

    BTIF_TRACE_DEBUG("%s: Current Services:0x%x", __FUNCTION__, btif_enabled_services);

    if (btif_is_enabled())
    {
        btif_transfer_context(btif_dm_execute_service_request,
                              BTIF_DM_DISABLE_SERVICE,
                              (char*)p_id, sizeof(tBTA_SERVICE_ID), NULL);
    }

    return BT_STATUS_SUCCESS;
}

static void btif_jni_associate(UNUSED_ATTR uint16_t event, UNUSED_ATTR char *p_param) {
  BTIF_TRACE_DEBUG("%s Associating thread to JVM", __func__);
  HAL_CBACK(bt_hal_cbacks, thread_evt_cb, ASSOCIATE_JVM);
}

static void btif_jni_disassociate(UNUSED_ATTR uint16_t event, UNUSED_ATTR char *p_param) {
  BTIF_TRACE_DEBUG("%s Disassociating thread from JVM", __func__);
  HAL_CBACK(bt_hal_cbacks, thread_evt_cb, DISASSOCIATE_JVM);
  bt_hal_cbacks = NULL;
  future_ready(stack_manager_get_hack_future(), FUTURE_SUCCESS);
}

#if (defined(SPRD_FEATURE_NONSIG) && SPRD_FEATURE_NONSIG == TRUE)
/*******************************************************************************
**
** Function         btif_handle_nonsig_rx_data
**
** Description      Receive Non-signal Test Mode RX data from controller.
                    transmit it to APP
**
** Returns          void
**
*******************************************************************************/
static void btif_handle_nonsig_rx_data( tBTM_VSC_CMPL *p )
{
    bt_status_t status;
    uint8_t result,rssi;
    uint32_t pkt_cnt,pkt_err_cnt;
    uint32_t bit_cnt,bit_err_cnt;
    UINT8 *buf;

    BTIF_TRACE_EVENT("%s", __FUNCTION__);
    BTIF_TRACE_EVENT("opcode = 0x%X",p->opcode);
    BTIF_TRACE_EVENT("param_len = 0x%X",p->param_len);

    if(p->param_len != 18) {
       status =  BT_STATUS_FAIL;
       HAL_CBACK(bt_hal_cbacks, nonsig_test_rx_recv_cb, status,0,0,0,0,0);
       return;
    }

    buf = p->p_param_buf;
    result = *buf;
    rssi = *(buf+1);
    pkt_cnt     = *(uint32_t *)(buf+2);
    pkt_err_cnt = *(uint32_t *)(buf+6);
    bit_cnt     = *(uint32_t *)(buf+10);
    bit_err_cnt = *(uint32_t *)(buf+14);

    BTIF_TRACE_EVENT("ret:0x%X, rssi:0x%X, pkt_cnt:0x%X, pkt_err_cnt:0x%X, bit_cnt:0x%X, pkt_err_cnt:0x%X",
                            result,rssi,pkt_cnt,pkt_err_cnt,bit_cnt,bit_err_cnt);

    if(result == 0)
        status = BT_STATUS_SUCCESS;
    else
        status = BT_STATUS_FAIL;

    HAL_CBACK(bt_hal_cbacks, nonsig_test_rx_recv_cb,status,rssi,pkt_cnt,pkt_err_cnt,bit_cnt,bit_err_cnt);

}

/*******************************************************************************
**
** Function         btif_vsc_cback
**
** Description     Callback invoked on completion of vendor specific command
**
** Returns          None
**
*******************************************************************************/
static void btif_vsc_cback( tBTM_VSC_CMPL *p )
{
    BTIF_TRACE_EVENT("%s",__FUNCTION__);

    BTIF_TRACE_EVENT("opcode = 0x%X",p->opcode);
    BTIF_TRACE_EVENT("param_len = 0x%X",p->param_len);

    if(p->opcode == NONSIG_RX_GETDATA || p->opcode == NONSIG_LE_RX_GETDATA){
        btif_handle_nonsig_rx_data(p);
    }
}

bt_status_t btif_set_nonsig_tx_testmode(uint16_t enable,
    uint16_t le, uint16_t pattern, uint16_t channel,
    uint16_t pac_type, uint16_t pac_len, uint16_t power_type,
    uint16_t power_value, uint16_t pac_cnt)
{
    uint16_t opcode;
    BTIF_TRACE_EVENT("%s",__FUNCTION__);

    /* sanity check */
    if (bt_hal_cbacks == NULL)
        return BT_STATUS_NOT_READY;

    BTIF_TRACE_EVENT("enable  : %X",enable);
    BTIF_TRACE_EVENT("le      : %X",le);

    BTIF_TRACE_EVENT("pattern : %X",pattern);
    BTIF_TRACE_EVENT("channel : %X",channel);
    BTIF_TRACE_EVENT("pac_type: %X",pac_type);
    BTIF_TRACE_EVENT("pac_len : %X",pac_len);
    BTIF_TRACE_EVENT("power_type   : %X",power_type);
    BTIF_TRACE_EVENT("power_value  : %X",power_value);
    BTIF_TRACE_EVENT("pac_cnt      : %X",pac_cnt);

    if(enable){
        opcode = le ? NONSIG_LE_TX_ENABLE : NONSIG_TX_ENABLE;
    }else{
        opcode = le ? NONSIG_LE_TX_DISABLE : NONSIG_TX_DISABLE;
    }

    if(enable){
        uint8_t buf[11];
        memset(buf,0x0,sizeof(buf));

        buf[0] = (uint8_t)pattern;
        buf[1] = (uint8_t)channel;
        buf[2] = (uint8_t)(pac_type & 0x00FF);
        buf[3] = (uint8_t)((pac_type & 0xFF00) >> 8);
        buf[4] = (uint8_t)(pac_len & 0x00FF);
        buf[5] = (uint8_t)((pac_len & 0xFF00) >> 8);
        buf[6] = (uint8_t)power_type;
        buf[7] = (uint8_t)(power_value & 0x00FF);
        buf[8] = (uint8_t)((power_value & 0xFF00) >> 8);
        buf[9] = (uint8_t)(pac_cnt & 0x00FF);
        buf[10] = (uint8_t)((pac_cnt & 0xFF00) >> 8);

        BTIF_TRACE_EVENT("send hci cmd, opcode = 0x%X",opcode);
        BTM_VendorSpecificCommand(opcode, sizeof(buf), buf, btif_vsc_cback);
    }else{/* disable */
        BTIF_TRACE_EVENT("send hci cmd, opcode = 0x%X",opcode);
        BTM_VendorSpecificCommand(opcode, 0, NULL, btif_vsc_cback);
    }

    return BT_STATUS_SUCCESS;
}

bt_status_t btif_set_nonsig_rx_testmode(uint16_t enable,
    uint16_t le, uint16_t pattern, uint16_t channel,
    uint16_t pac_type,uint16_t rx_gain, bt_bdaddr_t addr)
{
    uint16_t opcode;
    BTIF_TRACE_EVENT("%s",__FUNCTION__);

    /* sanity check */
    if (bt_hal_cbacks == NULL)
        return BT_STATUS_NOT_READY;


    BTIF_TRACE_EVENT("enable  : %X",enable);
    BTIF_TRACE_EVENT("le      : %X",le);

    BTIF_TRACE_EVENT("pattern : %d",pattern);
    BTIF_TRACE_EVENT("channel : %d",channel);
    BTIF_TRACE_EVENT("pac_type: %d",pac_type);
    BTIF_TRACE_EVENT("rx_gain : %d",rx_gain);
    BTIF_TRACE_EVENT("addr    : %02X:%02X:%02X:%02X:%02X:%02X",
        addr.address[0],addr.address[1],addr.address[2],
        addr.address[3],addr.address[4],addr.address[5]);

    if(enable){
        opcode = le ? NONSIG_LE_RX_ENABLE : NONSIG_RX_ENABLE;
    }else{
        opcode = le ? NONSIG_LE_RX_DISABLE : NONSIG_RX_DISABLE;
    }

    if(enable){
        uint8_t buf[11];
        memset(buf,0x0,sizeof(buf));

        buf[0] = (uint8_t)pattern;
        buf[1] = (uint8_t)channel;
        buf[2] = (uint8_t)(pac_type & 0x00FF);
        buf[3] = (uint8_t)((pac_type & 0xFF00) >> 8);
        buf[4] = (uint8_t)rx_gain;
        buf[5] = addr.address[5];
        buf[6] = addr.address[4];
        buf[7] = addr.address[3];
        buf[8] = addr.address[2];
        buf[9] = addr.address[1];
        buf[10] = addr.address[0];
        BTM_VendorSpecificCommand(opcode, sizeof(buf), buf, btif_vsc_cback);
    }else{
        BTM_VendorSpecificCommand(opcode, 0, NULL, btif_vsc_cback);
    }

    return BT_STATUS_SUCCESS;
}

bt_status_t btif_get_nonsig_rx_data(uint16_t le)
{
    BTIF_TRACE_EVENT("get_nonsig_rx_data LE=%d",le);
    uint16_t opcode;
    opcode = le ? NONSIG_LE_RX_GETDATA : NONSIG_RX_GETDATA;
    BTM_VendorSpecificCommand(opcode, 0, NULL, btif_vsc_cback);
    return BT_STATUS_SUCCESS;
}
#endif