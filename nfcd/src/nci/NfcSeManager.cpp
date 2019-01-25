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

#include "NfcSeManager.h"

#include "OverrideLog.h"
#include "NfcDebug.h"
#include "SecureElement.h"
#include "PowerSwitch.h"
#include "NfcTag.h"
#include "config.h"
#include <ScopedPrimitiveArray.h>

NfcSeManager NfcSeManager::sNfcSe;

/* START [J14120201] - Generic ESE ID */
extern jint slsiGetGenSeId(tNFA_HANDLE handle);
extern void slsiSetNfcSleepTimeout(unsigned long sec, int option);
/* END [J14120201] - Generic ESE ID */

// These must match the EE_ERROR_ types in NfcService.java
static const int EE_ERROR_INIT = -3;
static const int EE_ERROR_LISTEN_MODE = -4;
static const int EE_ERROR_EXT_FIELD = -5;

NfcSeManager::NfcSeManager()
{
  NCI_DEBUG("%s: Constructor called ", __FUNCTION__);
}

NfcSeManager::~NfcSeManager()
{
  NCI_DEBUG("%s: DeConstructor called ", __FUNCTION__);
}

NfcSeManager& NfcSeManager::GetInstance()
{
  return sNfcSe;
}

/*******************************************************************************
**
** Function:        doOpenSecureElementConnection
**
** Description:     Connect to the secure element.
**
** Returns:         Handle of secure element.  values < 0 represent failure.
**
*******************************************************************************/
int NfcSeManager::doOpenSecureElementConnection (int handle)
{
  NCI_DEBUG("%s: enter", __FUNCTION__);
  bool stat = true;
  int secElemHandle = EE_ERROR_INIT;
  SecureElement &se = SecureElement::GetInstance();

  /* START [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
  if (!SecureElement::GetInstance().isSupportedConcurrentWCE())
  {
    if (se.IsActivatedInListenMode())
    {
      NCI_DEBUG("Denying SE open due to SE listen mode active");
      secElemHandle = EE_ERROR_LISTEN_MODE;
      goto TheEnd;
    }

    if (se.IsRfFieldOn())
    {
      NCI_DEBUG("Denying SE open due to SE in active RF field");
      secElemHandle = EE_ERROR_EXT_FIELD;
      goto TheEnd;
    }
  }
  /* END [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/

  //tell the controller to power up to get ready for sec elem operations
  PowerSwitch::GetInstance ().SetLevel (PowerSwitch::FULL_POWER);
  PowerSwitch::GetInstance ().SetModeOn (PowerSwitch::SE_CONNECTED);

  //if controller is not routing AND there is no pipe connected,
  //then turn on the sec elem
  if (!se.IsBusy())
    stat = se.Activate();

  if (stat)
  {
    //establish a pipe to sec elem
    stat = se.ConnectEE();
    if (stat)
    {
      /* START [J14120201] - Generic ESE ID */
      secElemHandle = slsiGetGenSeId(se.mActiveEeHandle);
      /* END [J14120201] - Generic ESE ID */

      /* START [J15013011] - Override device sleep timer */
      unsigned long sec = 0;
      if (!GetNumValue("CE_DEVICE_SLEEP_TIMEOUT", &sec, sizeof(sec)))
        sec = 300;

      slsiSetNfcSleepTimeout(sec, 1 /* SET_SLEEP_TIME_ONCE */);
      /* END [J15013011] - Override device sleep timer */
    }
    else
    {
      se.Deactivate ();
    }
  }

  //if code fails to connect to the secure element, and nothing is active, then
  //tell the controller to power down
  if ((!stat) && (! PowerSwitch::GetInstance().SetModeOff(PowerSwitch::SE_CONNECTED)))
  {
    PowerSwitch::GetInstance ().SetLevel (PowerSwitch::LOW_POWER);
  }

TheEnd:
  NCI_DEBUG("%s: exit; return handle(gen)=0x%X", __FUNCTION__, secElemHandle);
  return secElemHandle;
}


/*******************************************************************************
 **
 ** Function:        doDisconnectSecureElementConnection
 **
 ** Description:     Disconnect from the secure element.
 **                  handle: Handle of secure element.
 **
 ** Returns:         True if ok.
 **
 *******************************************************************************/
bool NfcSeManager::doDisconnectSecureElementConnection (int handle)
{
  NCI_DEBUG("%s: enter; handle(gen)=0x%04x", __FUNCTION__, handle);
  bool stat = false;

  stat = SecureElement::GetInstance().DisconnectEE (handle);

  //if controller is not routing AND there is no pipe connected,
  //then turn off the sec elem
  if (!SecureElement::GetInstance().IsBusy())
    SecureElement::GetInstance().Deactivate();

  /* START [J15013011] - Override device sleep timer */
  slsiSetNfcSleepTimeout(1, 0 /* SET_SLEEP_TIME_CFG */);
  /* END [J15013011] - Override device sleep timer */

  //if nothing is active after this, then tell the controller to power down
  if (! PowerSwitch::GetInstance ().SetModeOff (PowerSwitch::SE_CONNECTED))
    PowerSwitch::GetInstance ().SetLevel (PowerSwitch::LOW_POWER);

  NCI_DEBUG("%s: exit", __FUNCTION__);
  return stat ? JNI_TRUE : JNI_FALSE;
}


/*******************************************************************************
 **
 ** Function:        doTransceive
 **
 ** Description:     Send data to the secure element; retrieve response.
 **                  handle: Secure element's handle.
 **                  data: Data to send.
 **
 ** Returns:         Buffer of received data.
 **
 *******************************************************************************/
int NfcSeManager::doTransceive (int handle, UINT8* data, UINT32 len, UINT8* rspBuffer, INT32 rspLen)
{
  const INT32 recvBufferMaxSize = 1024;
/* START [D18050301] Added Timeout for Wired Card */
#if(NFC_SEC_WIRED_CARD == TRUE)
  int timeout = NfcTag::GetInstance().GetTransceiveTimeout(TARGET_TYPE_ISO14443_WIRED_CARD);
#else
  //NFC service expects JNI to use ISO-DEP's timeout
  int timeout = NfcTag::GetInstance().GetTransceiveTimeout(TARGET_TYPE_ISO14443_4);
#endif
/* END [D18050301] Added Timeout for Wired Card */

  NCI_DEBUG("enter; handle=0x%X; buf len=%zu", handle, len);
  SecureElement::GetInstance().transceive(data, len, rspBuffer, recvBufferMaxSize, rspLen, timeout);

  NCI_DEBUG("exit: recv len=%ld", rspLen);
  return rspLen;
}

/*******************************************************************************
 **
 ** Function:        doGetAtrResponse
 **
 ** Description:     Receive HCI registry data.
 **                  handle: Secure element's handle.
 **
 ** Returns:         Buffer of received data.
 **
 *******************************************************************************/
int NfcSeManager::doGetAtrResponse (int handle, UINT8* recvBuffer, INT32 recvBufferMaxSize)
{
  NCI_DEBUG("enter; handle=0x%X; buf len=%zu", handle, recvBufferMaxSize);
  int rspLen = 0;

  SecureElement::GetInstance().GetAtrInfo(recvBuffer, rspLen);

  NCI_DEBUG("exit: recv len=%ld", rspLen);

  return rspLen;
}
