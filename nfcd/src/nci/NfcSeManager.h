/*
 * Copyright (C) 2012 The Android Open Source Project
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

#ifndef mozilla_nfcd_NfcSeManager_h
#define mozilla_nfcd_NfcSeManager_h

extern "C"
{
  #include "nfa_api.h"
  #include "nfa_ee_api.h"
}

class NfcSeManager
{
public:
// These must match the EE_ERROR_ types in NfcService.cpp
  static const int EE_ERROR_INIT = -3;
  static const int EE_ERROR_LISTEN_MODE = -4;
  static const int EE_ERROR_EXT_FIELD = -5;

  NfcSeManager();
  ~NfcSeManager();

  /**
   * Get the NfcSeManager singleton object.
   *
   * @return NfcSeManager object.
   */

  static NfcSeManager& GetInstance();

 /**
  * Connect to the Secure element.
  *
  * @return Handle of secure element.
 */
  int doOpenSecureElementConnection(int handle);

 /**
  * Disconnect from the secure element.
  *
  * @return True if ok.
 */
  bool doDisconnectSecureElementConnection(int handle);

 /**
  * Send data to the secure element; retrieve response.
  *
  * @return Buffer of received data.
 */
  int doTransceive(int handle, UINT8* data, UINT32 len, UINT8* rspBuffer, INT32 rsplen);

 /**
  * Receive HCI registry data.
  *
  * @return Buffer of received data.
 */
  int doGetAtrResponse (int handle, UINT8* recvBuffer, INT32 recvBufferMaxSize);

private:
  static NfcSeManager sNfcSe;

};

#endif
