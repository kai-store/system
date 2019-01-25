/*
 * Copyright (C) 2015  Mozilla Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#define LOG_TAG "libpdu"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <utils/Log.h>

/* support ICS logging macros */
#if ANDROID_VERSION < 16
#ifndef ALOGV
#define ALOGV LOGV
#endif /* ALOGV */

#ifndef ALOGD
#define ALOGD LOGD
#endif /* ALOGD */

#ifndef ALOGI
#define ALOGI LOGI
#endif /* ALOGI */

#ifndef ALOGW
#define ALOGW LOGW
#endif /* ALOGW */

#ifndef ALOGE
#define ALOGE LOGE
#endif /* ALOGE */
#endif /* ANDROID_VERSION < 16 */

#define _ERRNO_STR(_func, _err) \
  "%s failed: %s", (_func), strerror(_err)

#define ALOGE_ERRNO_NO(_func, _err) \
  ALOGE(_ERRNO_STR(_func, _err))

#define ALOGE_ERRNO(_func) \
  ALOGE_ERRNO_NO(_func, errno)

#define ALOGW_ERRNO_NO(_func, _err) \
  ALOGW(_ERRNO_STR(_func, _err))

#define ALOGW_ERRNO(_func) \
  ALOGW_ERRNO_NO(_func, errno)
