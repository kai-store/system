/******************************************************************************
 *
 *  Copyright (C) 2014 Google, Inc.
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

#define LOG_TAG "bt_stack_manager"

#include <hardware/bluetooth.h>

#include "btif_api.h"
#include "btif_common.h"
#include "device/include/controller.h"
#include "btcore/include/module.h"
#include "btcore/include/osi_module.h"
#include "osi/include/osi.h"
#include "osi/include/log.h"
#include "osi/include/semaphore.h"
#include "stack_manager.h"
#include "osi/include/thread.h"

// Temp includes
#include "btif_config.h"
#include "btif_profile_queue.h"
#include "bt_utils.h"

static thread_t *management_thread;

// If initialized, any of the bluetooth API functions can be called.
// (e.g. turning logging on and off, enabling/disabling the stack, etc)
static bool stack_is_initialized;
// If running, the stack is fully up and able to bluetooth.
static bool stack_is_running;

static void event_init_stack(void *context);
static void event_start_up_stack(void *context);
#if defined (BOARD_HAVE_FM_BCM)
static void event_start_up_radio(void *context);
#endif
static void event_shut_down_stack(void *context);
#if defined (BOARD_HAVE_FM_BCM)
static void event_shut_down_radio(void *context);
#endif
static void event_clean_up_stack(void *context);

static void event_signal_stack_up(void *context);
static void event_signal_stack_down(void *context);
#if defined (BOARD_HAVE_FM_BCM)
static void event_signal_radio_up(void *context);
static void event_signal_radio_down(void *context);
#endif


// Unvetted includes/imports, etc which should be removed or vetted in the future
static future_t *hack_future;
void bte_main_enable();
void btif_thread_post(thread_fn func, void *context);
// End unvetted section

// Interface functions

static void init_stack(void) {
  // This is a synchronous process. Post it to the thread though, so
  // state modification only happens there.
  semaphore_t *semaphore = semaphore_new(0);
  thread_post(management_thread, event_init_stack, semaphore);
  semaphore_wait(semaphore);
  semaphore_free(semaphore);
}

static void start_up_stack_async(void) {
  thread_post(management_thread, event_start_up_stack, NULL);
}

#if defined (BOARD_HAVE_FM_BCM)
static void start_up_radio_async(void){
  thread_post(management_thread, event_start_up_radio, NULL);
}
#endif

static void shut_down_stack_async(void) {
  thread_post(management_thread, event_shut_down_stack, NULL);
}

#if defined (BOARD_HAVE_FM_BCM)
static void shut_down_radio_async(void){
  thread_post(management_thread, event_shut_down_radio, NULL);
}
#endif

static void clean_up_stack_async(void) {
  thread_post(management_thread, event_clean_up_stack, NULL);
}

static bool get_stack_is_running(void) {
  return stack_is_running;
}

// Internal functions

// Synchronous function to initialize the stack
static void event_init_stack(void *context) {
  semaphore_t *semaphore = (semaphore_t *)context;

  if (!stack_is_initialized) {
    module_management_start();

    module_init(get_module(BT_UTILS_MODULE));
    module_init(get_module(BTIF_CONFIG_MODULE));
    btif_init_bluetooth();

    // stack init is synchronous, so no waiting necessary here
    stack_is_initialized = true;
  }

  if (semaphore)
    semaphore_post(semaphore);
}

static void ensure_stack_is_initialized(void) {
  if (!stack_is_initialized) {
    LOG_WARN("%s found the stack was uninitialized. Initializing now.", __func__);
    // No semaphore needed since we are calling it directly
    event_init_stack(NULL);
  }
}

// Synchronous function to start up the stack
static void event_start_up_stack(UNUSED_ATTR void *context) {
  if (stack_is_running) {
    LOG_DEBUG("%s stack already brought up.", __func__);
    return;
  }

  ensure_stack_is_initialized();

  LOG_DEBUG("%s is bringing up the stack.", __func__);
  hack_future = future_new();

#if defined (BOARD_HAVE_FM_BCM)
  btif_enable_bluetooth();

#else
  // Include this for now to put btif config into a shutdown-able state
  module_start_up(get_module(BTIF_CONFIG_MODULE));
  bte_main_enable();
#endif
  if (future_await(hack_future) != FUTURE_SUCCESS) {
    stack_is_running = true; // So stack shutdown actually happens
    event_shut_down_stack(NULL);
    return;
  }

  stack_is_running = true;
  LOG_DEBUG("%s finished", __func__);
  btif_thread_post(event_signal_stack_up, NULL);
}

#if defined (BOARD_HAVE_FM_BCM)
static void event_start_up_radio(UNUSED_ATTR void *context) {
  ensure_stack_is_initialized();

  LOG_DEBUG("%s is bringing up the radio.", __func__);
  hack_future = future_new();
	
  btif_enable_radio();

  if (future_await(hack_future) != FUTURE_SUCCESS) {
	event_shut_down_radio(NULL);
	return;
  }
	
  LOG_DEBUG("%s finished", __func__);
  btif_thread_post(event_signal_radio_up, NULL);
}
#endif

#if defined (BOARD_HAVE_FM_BCM)
// Synchronous function to shut down the stack
static void event_shut_down_stack(UNUSED_ATTR void *context) {
  int ref_count = 0;
  if (!stack_is_running) {
    LOG_DEBUG("%s stack is already brought down.", __func__);
    return;
  }

  LOG_DEBUG("%s is bringing down the stack.", __func__);
  hack_future = future_new();
  btif_disable_bluetooth(&ref_count);
  
  stack_is_running = false;
  if(ref_count == 0){
     module_shut_down(get_module(BTIF_CONFIG_MODULE));
  }

  future_await(hack_future);
  if(ref_count == 0){
     module_shut_down(get_module(CONTROLLER_MODULE)); // Doesn't do any work, just puts it in a restartable state
  }

  LOG_DEBUG("%s finished.", __func__);
  btif_thread_post(event_signal_stack_down, NULL);
}

static void event_shut_down_radio(UNUSED_ATTR void *context) {
  int ref_count = 0;

  LOG_DEBUG("%s is bringing down the radio.", __func__);
  hack_future = future_new();

  btif_disable_radio(&ref_count);
  if(ref_count ==0)
  {
  	module_shut_down(get_module(BTIF_CONFIG_MODULE));

	future_await(hack_future);
	module_shut_down(get_module(CONTROLLER_MODULE)); // Doesn't do any work, just puts it in a restartable state
	btif_thread_post(event_signal_radio_down, NULL);

	LOG_DEBUG("%s finished.", __func__);	 
  }
  else
  {
  	 btif_thread_post(event_signal_radio_down, NULL);
  }
}
#else
// Synchronous function to shut down the stack
static void event_shut_down_stack(UNUSED_ATTR void *context) {
  if (!stack_is_running) {
    LOG_DEBUG("%s stack is already brought down.", __func__);
    return;
  }

  LOG_DEBUG("%s is bringing down the stack.", __func__);
  hack_future = future_new();
  stack_is_running = false;

  btif_disable_bluetooth();
  module_shut_down(get_module(BTIF_CONFIG_MODULE));

  future_await(hack_future);
  module_shut_down(get_module(CONTROLLER_MODULE)); // Doesn't do any work, just puts it in a restartable state

  LOG_DEBUG("%s finished.", __func__);
  btif_thread_post(event_signal_stack_down, NULL);
}
#endif

static void ensure_stack_is_not_running(void) {
  if (stack_is_running) {
    LOG_WARN("%s found the stack was still running. Bringing it down now.", __func__);
    event_shut_down_stack(NULL);
  }
}

// Synchronous function to clean up the stack
static void event_clean_up_stack(UNUSED_ATTR void *context) {
  if (!stack_is_initialized) {
    LOG_DEBUG("%s found the stack already in a clean state.", __func__);
    return;
  }

  ensure_stack_is_not_running();

  LOG_DEBUG("%s is cleaning up the stack.", __func__);
  hack_future = future_new();
  stack_is_initialized = false;

  btif_shutdown_bluetooth();
  module_clean_up(get_module(BTIF_CONFIG_MODULE));
  module_clean_up(get_module(BT_UTILS_MODULE));

  future_await(hack_future);
  module_clean_up(get_module(OSI_MODULE));
  module_management_stop();
  LOG_DEBUG("%s finished.", __func__);
}

static void event_signal_stack_up(UNUSED_ATTR void *context) {
  // Notify BTIF connect queue that we've brought up the stack. It's
  // now time to dispatch all the pending profile connect requests.
  btif_queue_connect_next();
  HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_STATE_ON);
}
#if defined (BOARD_HAVE_FM_BCM)
static void event_signal_radio_up(UNUSED_ATTR void *context) {
  // Notify BTIF connect queue that we've brought up the stack. It's
  // now time to dispatch all the pending profile connect requests.
  btif_queue_connect_next();
  HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_RADIO_ON);
}
#endif

static void event_signal_stack_down(UNUSED_ATTR void *context) {
  HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_STATE_OFF);
}

#if defined (BOARD_HAVE_FM_BCM)
static void event_signal_radio_down(UNUSED_ATTR void *context) {
  HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_RADIO_OFF);
}

void stack_callback(bt_state_t state)
{
	switch(state){
		case BT_STATE_OFF:
			event_signal_stack_down(NULL);
			break;
		case BT_STATE_ON:
			event_signal_stack_up(NULL);
			break;
		case BT_RADIO_ON:
			event_signal_radio_up(NULL);
			break;
		case BT_RADIO_OFF:
			event_signal_radio_down(NULL);
			break;
		default:
			break;
	}
}
#endif

static void ensure_manager_initialized(void) {
  if (management_thread)
    return;

  management_thread = thread_new("stack_manager");
  if (!management_thread) {
    LOG_ERROR("%s unable to create stack management thread.", __func__);
    return;
  }
}

static const stack_manager_t interface = {
  init_stack,
  start_up_stack_async,
#if defined (BOARD_HAVE_FM_BCM)
  start_up_radio_async,
#endif
  shut_down_stack_async,
#if defined (BOARD_HAVE_FM_BCM)
  shut_down_radio_async,
#endif
  clean_up_stack_async,

  get_stack_is_running
};

const stack_manager_t *stack_manager_get_interface() {
  ensure_manager_initialized();
  return &interface;
}

future_t *stack_manager_get_hack_future() {
  return hack_future;
}
