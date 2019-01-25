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

#pragma once

#include <stdbool.h>

#include "bt_types.h"

#include "bt_target.h"

static const char BTSNOOP_MODULE[] = "btsnoop_module";

typedef struct btsnoop_t {
  // Inform btsnoop whether the API desires to log. If |value| is true.
  // logging will be enabled. Otherwise it defers to the value from the
  // config file.
  void (*set_api_wants_to_log)(bool value);

  // Capture |packet| and dump it to the btsnoop logs. If |is_received| is
  // true, the packet is marked as incoming. Otherwise, the packet is marked
  // as outgoing.
  void (*capture)(const BT_HDR *packet, bool is_received);
} btsnoop_t;

const btsnoop_t *btsnoop_get_interface(void);

#if (defined(SPRD_FEATURE_SLOG) && SPRD_FEATURE_SLOG == TRUE)
static const char BTSNOOP_SPRD_MODULE[] = "btsnoop_sprd_module";
const btsnoop_t *btsnoop_sprd_get_interface(void);
#endif
