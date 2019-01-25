/*
 * Copyright 2015, The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include <inttypes.h>

#include <binder/IPCThreadState.h>
#include <binder/IServiceManager.h>
#include <binder/PermissionCache.h>
#include <utils/String16.h>
#include <utils/Looper.h>
#include <keystore/IKeystoreService.h>
#include <keystore/keystore.h> // for error code
#include <hardware/hardware.h>
#include <hardware/fingerprint.h>
#include <hardware/hw_auth_token.h>
#include "IFingerprintDaemon.h"
#include "IFingerprintDaemonCallback.h"

namespace android {

static const String16 USE_FINGERPRINT_PERMISSION("android.permission.USE_FINGERPRINT");
static const String16 MANAGE_FINGERPRINT_PERMISSION("android.permission.MANAGE_FINGERPRINT");
static const String16 HAL_FINGERPRINT_PERMISSION("android.permission.MANAGE_FINGERPRINT"); // TODO
static const String16 DUMP_PERMISSION("android.permission.DUMP");

const android::String16
IFingerprintDaemon::descriptor("android.hardware.fingerprint.IFingerprintDaemon");

const android::String16&
IFingerprintDaemon::getInterfaceDescriptor() const {
    return IFingerprintDaemon::descriptor;
}

status_t BnFingerprintDaemon::onTransact(uint32_t code, const Parcel& data, Parcel* reply,
        uint32_t flags) {
    switch(code) {
        case AUTHENTICATE: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            if (!checkPermission(HAL_FINGERPRINT_PERMISSION)) {
                return PERMISSION_DENIED;
            }
            const uint64_t sessionId = data.readInt64();
            const uint32_t groupId = data.readInt32();
            const int32_t ret = authenticate(sessionId, groupId);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        };
        case CANCEL_AUTHENTICATION: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            if (!checkPermission(HAL_FINGERPRINT_PERMISSION)) {
                return PERMISSION_DENIED;
            }
            const int32_t ret = stopAuthentication();
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case ENROLL: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            if (!checkPermission(HAL_FINGERPRINT_PERMISSION)) {
                return PERMISSION_DENIED;
            }
            const ssize_t tokenSize = data.readInt32();
            const uint8_t* token = static_cast<const uint8_t *>(data.readInplace(tokenSize));
            const int32_t groupId = data.readInt32();
            const int32_t timeout = data.readInt32();
            const int32_t ret = enroll(token, tokenSize, groupId, timeout);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case CANCEL_ENROLLMENT: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            if (!checkPermission(HAL_FINGERPRINT_PERMISSION)) {
                return PERMISSION_DENIED;
            }
            const int32_t ret = stopEnrollment();
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case PRE_ENROLL: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            if (!checkPermission(HAL_FINGERPRINT_PERMISSION)) {
                return PERMISSION_DENIED;
            }
            const uint64_t ret = preEnroll();
            reply->writeNoException();
            reply->writeInt64(ret);
            return NO_ERROR;
        }
        case POST_ENROLL: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            if (!checkPermission(HAL_FINGERPRINT_PERMISSION)) {
                return PERMISSION_DENIED;
            }
            const int32_t ret = postEnroll();
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case REMOVE: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            if (!checkPermission(HAL_FINGERPRINT_PERMISSION)) {
                return PERMISSION_DENIED;
            }
            const int32_t fingerId = data.readInt32();
            const int32_t groupId = data.readInt32();
            const int32_t ret = remove(fingerId, groupId);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case GET_AUTHENTICATOR_ID: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            if (!checkPermission(HAL_FINGERPRINT_PERMISSION)) {
                return PERMISSION_DENIED;
            }
            const uint64_t ret = getAuthenticatorId();
            reply->writeNoException();
            reply->writeInt64(ret);
            return NO_ERROR;
        }
        case SET_ACTIVE_GROUP: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            if (!checkPermission(HAL_FINGERPRINT_PERMISSION)) {
                return PERMISSION_DENIED;
            }
            const int32_t group = data.readInt32();
            const ssize_t pathSize = data.readInt32();
            const uint8_t* path = static_cast<const uint8_t *>(data.readInplace(pathSize));
            const int32_t ret = setActiveGroup(group, path, pathSize);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case OPEN_HAL: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            if (!checkPermission(HAL_FINGERPRINT_PERMISSION)) {
                return PERMISSION_DENIED;
            }
            const int64_t ret = openHal();
            reply->writeNoException();
            reply->writeInt64(ret);
            return NO_ERROR;
        }
        case CLOSE_HAL: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            if (!checkPermission(HAL_FINGERPRINT_PERMISSION)) {
                return PERMISSION_DENIED;
            }
            const int32_t ret = closeHal();
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case INIT: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            if (!checkPermission(HAL_FINGERPRINT_PERMISSION)) {
                return PERMISSION_DENIED;
            }
            sp<IFingerprintDaemonCallback> callback =
                    interface_cast<IFingerprintDaemonCallback>(data.readStrongBinder());
            init(callback);
            reply->writeNoException();
            return NO_ERROR;
        }
        // Add by silead begin
        case SET_SCREEN_STATUS: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t screen_status = data.readInt32();
            int32_t ret = setFPScreenStatus(screen_status);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case SET_FP_ENABLE_CREDENTIAL: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t index = data.readInt32();
            const int32_t enable = data.readInt32();
            int32_t ret = setFPEnableCredential(index, enable);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case GET_FP_ENABLE_CREDENTIAL:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t index = data.readInt32();
            int32_t ret = getFPEnableCredential(index);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case GET_FP_VIRTUAL_KEYCODE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            int32_t ret = getFPVirtualKeyCode();
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case SET_FP_VIRTUAL_KEYCODE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t scanCode = data.readInt32();
            int32_t ret = setFPVirtualKeyCode(scanCode);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case GET_FP_LONG_PRESS_VIRTUAL_KEYCODE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            int32_t ret = getFPLongPressVirtualKeyCode();
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case SET_FP_LONG_PRESS_VIRTUAL_KEYCODE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t scanCode = data.readInt32();
            int32_t ret = setFPLongPressVirtualKeyCode(scanCode);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case GET_FP_DOUBLE_CLICK_VIRTUAL_KEYCODE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            int32_t ret = getFPDouClickVirtualKeyCode();
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case SET_FP_DOUBLE_CLICK_VIRTUAL_KEYCODE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t scanCode = data.readInt32();
            int32_t ret = setFPDouClickVirtualKeyCode(scanCode);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case GET_FP_VIRTUAL_KEY_STATE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            int32_t ret = getFPVirtualKeyState();
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case SET_FP_VIRTUAL_KEY_STATE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t virtualState = data.readInt32();
            int32_t ret = setFPVirtualKeyState(virtualState);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case GET_FP_WAKEUP_STATE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            int32_t ret = getFPWakeUpState();
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case SET_FP_WAKEUP_STATE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t wakeupState = data.readInt32();
            int32_t ret = setFPWakeUpState(wakeupState);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
       case GET_FINGERPRINT_STATE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            int32_t ret = getFingerPrintState();
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case SET_FINGERPRINT_STATE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t fingerprintState = data.readInt32();
            int32_t ret = setFingerPrintState(fingerprintState);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case SET_FP_POWER_FUNCTION_KEY_STATE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t powerFunctionKeyState = data.readInt32();
            int32_t ret = setFPPowerFuncKeyState(powerFunctionKeyState);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case GET_FP_POWER_FUNCTION_KEY_STATE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            int32_t ret = getFPPowerFuncKeyState();
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case SET_FP_IDLE_FUNCTION_KEY_STATE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t idleFunctionKeyState = data.readInt32();
            int32_t ret = setFPIdleFuncKeyState(idleFunctionKeyState);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case GET_FP_IDLE_FUNCTION_KEY_STATE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            int32_t ret = getFPIdleFuncKeyState();
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case SET_FP_WHOLE_FUNCTION_KEY_STATE:{
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t wholeFunctionKeyState = data.readInt32();
            int32_t ret = setFPWholeFuncKeyState(wholeFunctionKeyState);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case SET_FP_FUNCTION_KEY_STATE: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t index = data.readInt32();
            const int32_t enable = data.readInt32();
            int32_t ret = setFPFunctionKeyState(index, enable);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        case GET_FP_FUNCTION_KEY_STATE: {
            CHECK_INTERFACE(IFingerprintDaemon, data, reply);
            const int32_t index = data.readInt32();
            int32_t ret = getFPFunctionKeyState(index);
            reply->writeNoException();
            reply->writeInt32(ret);
            return NO_ERROR;
        }
        // Add by silead end
        default:
            return BBinder::onTransact(code, data, reply, flags);
    }
};

bool BnFingerprintDaemon::checkPermission(const String16& permission) {
    const IPCThreadState* ipc = IPCThreadState::self();
    const int calling_pid = ipc->getCallingPid();
    const int calling_uid = ipc->getCallingUid();
    return PermissionCache::checkPermission(permission, calling_pid, calling_uid);
}


}; // namespace android
