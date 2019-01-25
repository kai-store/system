/******************************************************************************
 *
 *  Copyright (C) 2016 Spreadtrum Corporation
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

#define LOG_TAG "btif_sprd"


#include <ctype.h>
#include <cutils/properties.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <hardware/bluetooth.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "bt_target.h"
#include "bt_utils.h"
#include "bta_api.h"
#include "btm_api.h"
#include "btif_util.h"
#include "btu.h"
//#include "bt_common.h"
#include "btif_common.h"
#include "osi/include/allocator.h"

#include "btif_sprd.h"


/*****************************************************************************
**  Constants & Macros
******************************************************************************/

static btsprd_callbacks_t *bt_sprd_callbacks = NULL;


static bt_status_t init_sprd(btsprd_callbacks_t* callbacks)
{
    BTIF_TRACE_EVENT("%s()", __func__);

    bt_sprd_callbacks = callbacks;

    return BT_STATUS_SUCCESS;
}

static bt_status_t btif_sprd_vendor_cmd_send(uint16_t opcode, uint8_t len, uint8_t* buf, void *p)
{
  BTIF_TRACE_DEBUG("%s, opcode: 0x%02x", __func__, opcode);
  BTM_VendorSpecificCommand(opcode, len, buf, (tBTM_VSC_CMPL_CB*)p);
  return BT_STATUS_SUCCESS;
}

static const btsprd_interface_t bt_sprd_interface = {
    sizeof(btsprd_interface_t),
    init_sprd,
    btif_sprd_vendor_cmd_send
};

const btsprd_interface_t *btif_sprd_get_interface(void)
{
    BTIF_TRACE_EVENT("%s", __FUNCTION__);
    return &bt_sprd_interface;
}
