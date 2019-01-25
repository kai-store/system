/*
 * Copyright (C) 2015 The Android Open Source Project
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

#ifndef FINGERPRINT_DAEMON_PROXY_H_
#define FINGERPRINT_DAEMON_PROXY_H_

#include "IFingerprintDaemon.h"
#include "IFingerprintDaemonCallback.h"

namespace android {

class FingerprintDaemonProxy : public BnFingerprintDaemon {
    public:
        static FingerprintDaemonProxy* getInstance() {
            if (sInstance == NULL) {
                sInstance = new FingerprintDaemonProxy();
            }
            return sInstance;
        }

        // These reflect binder methods.
        virtual void init(const sp<IFingerprintDaemonCallback>& callback);
        virtual int32_t enroll(const uint8_t* token, ssize_t tokenLength, int32_t groupId, int32_t timeout);
        virtual uint64_t preEnroll();
        virtual int32_t postEnroll();
        virtual int32_t stopEnrollment();
        virtual int32_t authenticate(uint64_t sessionId, uint32_t groupId);
        virtual int32_t stopAuthentication();
        virtual int32_t remove(int32_t fingerId, int32_t groupId);
        virtual uint64_t getAuthenticatorId();
        virtual int32_t setActiveGroup(int32_t groupId, const uint8_t* path, ssize_t pathLen);
        virtual int64_t openHal();
        virtual int32_t closeHal();
        // Add by silead begin
        virtual int32_t setFPScreenStatus(int32_t screenStatus);
        virtual int32_t setFPEnableCredential(int32_t index, int32_t enable);
        virtual int32_t getFPEnableCredential(int32_t index);
        virtual int32_t getFPVirtualKeyCode();
        virtual int32_t setFPVirtualKeyCode(int virtualKeyCode);
        virtual int32_t getFPLongPressVirtualKeyCode();
        virtual int32_t setFPLongPressVirtualKeyCode(int virtualKeyCode);
        virtual int32_t getFPDouClickVirtualKeyCode();
        virtual int32_t setFPDouClickVirtualKeyCode(int virtualKeyCode);
        virtual int32_t getFPVirtualKeyState();
        virtual int32_t setFPVirtualKeyState(int virtualKeyState);
        virtual int32_t getFPWakeUpState();
        virtual int32_t setFPWakeUpState(int wakeUpState);
        virtual int32_t getFingerPrintState();
        virtual int32_t setFingerPrintState(int32_t fingerPrintState);
        virtual int32_t setFPPowerFuncKeyState(int32_t funcKeyState);
        virtual int32_t getFPPowerFuncKeyState();
        virtual int32_t setFPIdleFuncKeyState(int32_t funcKeyState);
        virtual int32_t getFPIdleFuncKeyState();
        virtual int32_t setFPWholeFuncKeyState(int funcKeyState);
        virtual int32_t setFPFunctionKeyState(int index, int enable);
        virtual int32_t getFPFunctionKeyState(int index);
        // Add by silead end

    private:
        FingerprintDaemonProxy();
        virtual ~FingerprintDaemonProxy();
        void binderDied(const wp<IBinder>& who);
        void notifyKeystore(const uint8_t *auth_token, const size_t auth_token_length);
        static void hal_notify_callback(const fingerprint_msg_t *msg);

        static FingerprintDaemonProxy* sInstance;
        fingerprint_module_t const* mModule;
        fingerprint_device_t* mDevice;
        sp<IFingerprintDaemonCallback> mCallback;
};

} // namespace android

#endif // FINGERPRINT_DAEMON_PROXY_H_
