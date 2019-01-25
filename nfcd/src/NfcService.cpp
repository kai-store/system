/*
 * Copyright (C) 2013-2014  Mozilla Foundation
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

#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <memory>

#include "IntervalTimer.h"
#include "MessageHandler.h"
#include "INfcManager.h"
#include "INfcTag.h"
#include "IP2pDevice.h"
#include "DeviceHost.h"
#include "NfcService.h"
#include "NfcUtil.h"
#include "NfcDebug.h"
#include "P2pLinkManager.h"
#include "SessionId.h"
#include "NfcSeManager.h"

using namespace android;

typedef enum {
  MSG_UNDEFINED = 0,
  MSG_LLCP_LINK_ACTIVATION,
  MSG_LLCP_LINK_DEACTIVATION,
  MSG_TAG_DISCOVERED,
  MSG_TAG_LOST,
  MSG_SE_FIELD_ACTIVATED,
  MSG_SE_FIELD_DEACTIVATED,
  MSG_SE_NOTIFY_TRANSACTION_EVENT,
  MSG_READ_NDEF,
  MSG_WRITE_NDEF,
  MSG_SOCKET_CONNECTED,
  MSG_MAKE_NDEF_READONLY,
  MSG_LOW_POWER,
  MSG_ENABLE,
  MSG_RECEIVE_NDEF_EVENT,
  MSG_NDEF_FORMAT,
  MSG_TAG_TRANSCEIVE,
  MSG_SE_OPEN_CHANNEL,
  MSG_SE_TRANSMIT,
  MSG_SE_CLOSE_CHANNEL,
  MSG_SE_RESET_SECUREELEMENT,
  MSG_SE_GET_ATR,
  MSG_LS_EXECUTE_SCRIPT,
  MSG_LS_GET_VERSION,
  MSG_MPOS_READER_MODE,
  MSG_SE_NOTIFY_MPOSREADERMODE_EVENT,
  MSG_LISTEN_MODE_ACTIVATED,
  MSG_LISTEN_MODE_DEACTIVATED,
  MSG_NFC_SELF_TEST
} NfcEventType;

typedef enum {
  STATE_NFC_OFF = 0,
  STATE_NFC_ON_LOW_POWER,
  STATE_NFC_ON,
  STATE_NFC_ON_LOW_POWER_SCREEN_OFF,
  STATE_NFC_ON_LOW_POWER_SCREEN_LOCK,
  STATE_NFC_ON_ULTRA_LOW_POWER
} NfcState;

typedef enum {
  SCREEN_STATE_DEFAULT = 0,
  SCREEN_STATE_OFF,
  SCREEN_STATE_LOCKED,
  SCREEN_STATE_UNLOCKED,
  STATE_ULTRA_LOW_POWER
} NfcPwrScreenState;

typedef enum
{
  NFC_MPOS_READER_MODE_FAILED = 0x00,
  NFC_MPOS_READER_MODE_START_SUCCESS,
  NFC_MPOS_READER_MODE_STOP_SUCCESS,
  NFC_MPOS_READER_MODE_TIMEOUT,
  NFC_MPOS_READER_MODE_REMOVE_CARD,
  NFC_MPOS_READER_MODE_RESTART,
  NFC_MPOS_READER_MODE_INVALID
} NfcMposEvent;

class NfcEvent {
public:
  NfcEvent(NfcEventType aType)
   : mType(aType)
  {}

  NfcEventType GetType() { return mType; }

  int arg1;
  int arg2;
  void* obj;

private:
  NfcEventType mType;
};

class PollingThreadParam {
public:
  INfcTag* pINfcTag;
  int sessionId;
};

static pthread_t thread_id;
static sem_t thread_sem;
// static IntervalTimer card_removal_timer;
static IntervalTimer enable_disable_timer;

#define ENABLE_DISABLE_TIMEOUT 30000 // 30 seconds

NfcService* NfcService::sInstance = NULL;
NfcManager* NfcService::sNfcManager = NULL;

extern void EnableDisableLog(bool type);

NfcService::NfcService()
 : mState(STATE_NFC_OFF)
 , mIsTagPresent(false)
 , mIsRecovery(false)
{
  mP2pLinkManager = new P2pLinkManager(this);
}

NfcService::~NfcService()
{
  delete mP2pLinkManager;
}

static void* ServiceThreadFunc(void* aArg)
{
  pthread_setname_np(pthread_self(), "NFCService thread");
  NfcService* service = reinterpret_cast<NfcService*>(aArg);
  return service->EventLoop();
}

void NfcService::Initialize(NfcManager* aNfcManager, MessageHandler* aMsgHandler)
{
  if (sem_init(&thread_sem, 0, 0) == -1) {
    NFCD_ERROR("init_nfc_service Semaphore creation failed");
    abort();
  }

  if (pthread_create(&thread_id, NULL, ServiceThreadFunc, this) != 0) {
    NFCD_ERROR("init_nfc_service pthread_create failed");
    abort();
  }

  mMsgHandler = aMsgHandler;
  sNfcManager = aNfcManager;
}

void NfcService::NotifyLlcpLinkActivated(IP2pDevice* aDevice)
{
  NFCD_DEBUG("enter");
  NfcEvent* event = new NfcEvent(MSG_LLCP_LINK_ACTIVATION);
  event->obj = reinterpret_cast<void*>(aDevice);
  NfcService::Instance()->mQueue.push_back(event);
  sem_post(&thread_sem);
}

void NfcService::NotifyLlcpLinkDeactivated(IP2pDevice* aDevice)
{
  NFCD_DEBUG("enter");
  NfcEvent* event = new NfcEvent(MSG_LLCP_LINK_DEACTIVATION);
  event->obj = reinterpret_cast<void*>(aDevice);
  NfcService::Instance()->mQueue.push_back(event);
  sem_post(&thread_sem);
}

void NfcService::NotifyTagDiscovered(INfcTag* aTag)
{
  NFCD_DEBUG("enter");
  NfcEvent* event = new NfcEvent(MSG_TAG_DISCOVERED);
  event->obj = reinterpret_cast<void*>(aTag);
  NfcService::Instance()->mQueue.push_back(event);
  sem_post(&thread_sem);
}

void NfcService::NotifyTagLost(int aSessionId)
{
  NFCD_DEBUG("enter");
  NfcEvent* event = new NfcEvent(MSG_TAG_LOST);
  event->obj = reinterpret_cast<void*>(aSessionId);
  NfcService::Instance()->mQueue.push_back(event);
  sem_post(&thread_sem);
}

void NfcService::NotifySEFieldActivated()
{
  NFCD_DEBUG("enter");
  NfcEvent* event = new NfcEvent(MSG_SE_FIELD_ACTIVATED);
  NfcService::Instance()->mQueue.push_back(event);
  sem_post(&thread_sem);
}

void NfcService::NotifySEFieldDeactivated()
{
  NFCD_DEBUG("enter");
  NfcEvent* event = new NfcEvent(MSG_SE_FIELD_DEACTIVATED);
  NfcService::Instance()->mQueue.push_back(event);
  sem_post(&thread_sem);
}

void NfcService::NotifyListenModeActivated()
{
  NFCD_DEBUG("enter");
  NfcEvent* event = new NfcEvent(MSG_LISTEN_MODE_ACTIVATED);
  NfcService::Instance()->mQueue.push_back(event);
  sem_post(&thread_sem);
}

void NfcService::NotifyListenModeDeactivated()
{
  NFCD_DEBUG("enter");
  NfcEvent* event = new NfcEvent(MSG_LISTEN_MODE_DEACTIVATED);
  NfcService::Instance()->mQueue.push_back(event);
  sem_post(&thread_sem);
}

void NfcService::NotifySETransactionEvent(TransactionEvent* aEvent)
{
  NFCD_DEBUG("enter");
  NfcEvent* event = new NfcEvent(MSG_SE_NOTIFY_TRANSACTION_EVENT);
  event->obj = reinterpret_cast<void*>(aEvent);
  NfcService::Instance()->mQueue.push_back(event);
  sem_post(&thread_sem);
}

void NfcService::NotifySEMPOSReaderModeEvent(MPOSReaderModeEvent* aEvent)
{
  NFCD_DEBUG("enter");
  NfcEvent* event = new NfcEvent(MSG_SE_NOTIFY_MPOSREADERMODE_EVENT);
  event->obj = reinterpret_cast<void*>(aEvent);
  NfcService::Instance()->mQueue.push_back(event);
  sem_post(&thread_sem);
}

void NfcService::NotifySEMPOSReaderModeErrorEvent(MPOSReaderModeEvent* aEvent)
{
  NFCD_DEBUG("enter");

  int type = aEvent->eventType;

  // Call SEC API to notify error, workaround only.

  delete aEvent;
}

void NfcService::HandleLlcpLinkDeactivation(NfcEvent* aEvent)
{
  NFCD_DEBUG("enter");

  void* pDevice = aEvent->obj;
  IP2pDevice* pIP2pDevice = reinterpret_cast<IP2pDevice*>(pDevice);

  if (pIP2pDevice->GetMode() == NfcDepEndpoint::MODE_P2P_TARGET) {
    pIP2pDevice->Disconnect();
  }

  mP2pLinkManager->OnLlcpDeactivated();
  mMsgHandler->ProcessNotification(NFC_NOTIFICATION_TECH_LOST,
                                   reinterpret_cast<void*>(mP2pLinkManager->GetSessionId()));
  mP2pLinkManager->SetSessionId(-1);
}

void NfcService::HandleLlcpLinkActivation(NfcEvent* aEvent)
{
  NFCD_DEBUG("enter");
  void* pDevice = aEvent->obj;
  IP2pDevice* pIP2pDevice = reinterpret_cast<IP2pDevice*>(pDevice);

  if (pIP2pDevice->GetMode() == NfcDepEndpoint::MODE_P2P_TARGET ||
      pIP2pDevice->GetMode() == NfcDepEndpoint::MODE_P2P_INITIATOR) {
    if (pIP2pDevice->GetMode() == NfcDepEndpoint::MODE_P2P_TARGET) {
      if (pIP2pDevice->Connect()) {
        NFCD_DEBUG("Connected to device!");
      }
      else {
        NFCD_ERROR("Cannot connect remote Target. Polling loop restarted.");
      }
    }

    INfcManager* pINfcManager = NfcService::GetNfcManager();
    bool ret = pINfcManager->CheckLlcp();
    if (ret == true) {
      ret = pINfcManager->ActivateLlcp();
      if (ret == true) {
        NFCD_DEBUG("Target Activate LLCP OK");
      } else {
        NFCD_ERROR("doActivateLLcp failed");
      }
    } else {
      NFCD_ERROR("doCheckLLcp failed");
    }
  } else {
    NFCD_ERROR("Unknown LLCP P2P mode");
    //stop();
  }

  mP2pLinkManager->OnLlcpActivated();

  TechDiscoveredEvent* data = new TechDiscoveredEvent();

  mP2pLinkManager->SetSessionId(SessionId::GenerateNewId());
  data->sessionId = mP2pLinkManager->GetSessionId();
  data->isP2P = true;
  data->techCount = 0;
  data->techList = NULL;
  data->tagIdCount = 0;
  data->tagId = NULL;
  data->ndefMsgCount = 0;
  data->ndefMsg = NULL;
  data->ndefInfo = NULL;
  mMsgHandler->ProcessNotification(NFC_NOTIFICATION_TECH_DISCOVERED, data);
  delete data;
  NFCD_DEBUG("exit");
}

static void *PollingThreadFunc(void *aArg)
{
  PollingThreadParam* param = reinterpret_cast<PollingThreadParam*>(aArg);
  INfcTag* pINfcTag = param->pINfcTag;
  int sessionId = param->sessionId;

  NfcService::Instance()->TagDetected();

  while (pINfcTag->PresenceCheck()) {
    sleep(1);
  }

  NfcService::Instance()->TagRemoved();

  pINfcTag->Disconnect();

  NfcService::Instance()->NotifyTagLost(sessionId);

  delete param;

  pthread_detach(pthread_self());
  pthread_exit(NULL);
}

void NfcService::HandleTagDiscovered(NfcEvent* aEvent)
{
  // Do not support multiple tag discover.
  if (IsTagPresent()) {
    return;
  }

  INfcTag* pINfcTag = reinterpret_cast<INfcTag*>(aEvent->obj);

  // To get complete tag information, need to call read ndef first.
  // In readNdef function, it will add NDEF related info in NfcTagManager.
  std::auto_ptr<NdefMessage> pNdefMessage(pINfcTag->ReadNdef());
  std::auto_ptr<NdefInfo> pNdefInfo(pINfcTag->ReadNdefInfo());

  // Do the following after read ndef.
  std::vector<TagTechnology>& techList = pINfcTag->GetTechList();
  int techCount = techList.size();

  uint8_t* gonkTechList = new uint8_t[techCount];
  std::copy(techList.begin(), techList.end(), gonkTechList);

  TechDiscoveredEvent* data = new TechDiscoveredEvent();
  data->sessionId = SessionId::GenerateNewId();
  data->isP2P = false;
  data->techCount = techCount;
  data->techList = gonkTechList;
  data->tagIdCount = pINfcTag->GetUid().size();
  uint8_t* tagId = new uint8_t[data->tagIdCount];
  memcpy(tagId, &pINfcTag->GetUid()[0], data->tagIdCount);
  data->tagId = tagId;
  data->ndefMsgCount = pNdefMessage.get() ? 1 : 0;
  data->ndefMsg = pNdefMessage.get();
  data->ndefInfo = pNdefInfo.get();
  mMsgHandler->ProcessNotification(NFC_NOTIFICATION_TECH_DISCOVERED, data);

  PollingThreadParam* param = new PollingThreadParam();
  param->sessionId = data->sessionId;
  param->pINfcTag = pINfcTag;

  delete tagId;
  delete gonkTechList;
  delete data;

  pthread_t tid;
  pthread_create(&tid, NULL, PollingThreadFunc, param);
}

void NfcService::HandleTagLost(NfcEvent* aEvent)
{
  mMsgHandler->ProcessNotification(NFC_NOTIFICATION_TECH_LOST, aEvent->obj);
}

void NfcService::HandleTransactionEvent(NfcEvent* aEvent)
{
  mMsgHandler->ProcessNotification(NFC_NOTIFICATION_TRANSACTION_EVENT, aEvent->obj);
}

void NfcService::EnableTimeout(union sigval)
{
  NfcService* pNfcService = NfcService::Instance();
  if (pNfcService) {
    pNfcService->HandleEnableDisableTimeout(true);
    NFCD_DEBUG("Timeout enable nfc");
  }
  exit(EXIT_FAILURE);
}

void NfcService::DisableTimeout(union sigval)
{
  NfcService* pNfcService = NfcService::Instance();
  if (pNfcService) {
    pNfcService->HandleEnableDisableTimeout(false);
    NFCD_DEBUG("Timeout disable nfc");
  }
  exit(EXIT_FAILURE);
}

void NfcService::HandleMPOSReaderModeEvent(NfcEvent* aEvent)
{
  MPOSReaderModeEvent* pMPOSReaderModeEvent = reinterpret_cast<MPOSReaderModeEvent*>(aEvent->obj);
  int eventType = pMPOSReaderModeEvent->eventType;
  NFCD_DEBUG("Receive mPOS reader mode event:%d", eventType);

  mMsgHandler->ProcessNotification(NFC_NOTIFICATION_MPOS_READER_MODE_EVENT, aEvent->obj);
}

void NfcService::HandleSEFieldActivatedEvent()
{
  mMsgHandler->ProcessNotification(NFC_NOTIFICATION_RF_FIELD_ACTIVATED_EVENT, NULL);
}

void NfcService::HandleSEFieldDeActivatedEvent()
{
  mMsgHandler->ProcessNotification(NFC_NOTIFICATION_RF_FIELD_DEACTIVATED_EVENT, NULL);
}

void NfcService::HandleListenModeActivatedEvent()
{
  mMsgHandler->ProcessNotification(NFC_NOTIFICATION_LISTEN_MODE_ACTIVATED_EVENT, NULL);
}

void NfcService::HandleListenModeDeActivatedEvent()
{
  mMsgHandler->ProcessNotification(NFC_NOTIFICATION_LISTEN_MODE_DEACTIVATED_EVENT, NULL);
}

void* NfcService::EventLoop()
{
  NFCD_DEBUG("NFCService started");
  while(true) {
    if(sem_wait(&thread_sem)) {
      NFCD_ERROR("Failed to wait for semaphore");
      abort();
    }

    while (!mQueue.empty()) {
      NfcEvent* event = *mQueue.begin();
      mQueue.erase(mQueue.begin());
      NfcEventType eventType = event->GetType();

      NFCD_DEBUG("NFCService msg=%d", eventType);
      switch(eventType) {
        case MSG_LLCP_LINK_ACTIVATION:
          HandleLlcpLinkActivation(event);
          break;
        case MSG_LLCP_LINK_DEACTIVATION:
          HandleLlcpLinkDeactivation(event);
          break;
        case MSG_TAG_DISCOVERED:
          HandleTagDiscovered(event);
          break;
        case MSG_TAG_LOST:
          HandleTagLost(event);
          break;
        case MSG_SE_NOTIFY_TRANSACTION_EVENT:
          HandleTransactionEvent(event);
          break;
        case MSG_READ_NDEF:
          HandleReadNdefResponse(event);
          break;
        case MSG_WRITE_NDEF:
          HandleWriteNdefResponse(event);
          break;
        case MSG_SOCKET_CONNECTED:
          mMsgHandler->ProcessNotification(NFC_NOTIFICATION_INITIALIZED , NULL);
          break;
        case MSG_MAKE_NDEF_READONLY:
          HandleMakeNdefReadonlyResponse(event);
          break;
        case MSG_LOW_POWER:
          HandleEnterLowPowerResponse(event);
          break;
        case MSG_ENABLE:
          HandleEnableResponse(event);
          break;
        case MSG_RECEIVE_NDEF_EVENT:
          HandleReceiveNdefEvent(event);
          break;
        case MSG_NDEF_FORMAT:
          HandleNdefFormatResponse(event);
          break;
        case MSG_TAG_TRANSCEIVE:
          HandleTagTransceiveResponse(event);
          break;
        case MSG_SE_OPEN_CHANNEL:
          HandleOpenChannelResponse(event);
          break;
        case MSG_SE_TRANSMIT:
          HandleTransmitResponse(event);
          break;
        case MSG_SE_CLOSE_CHANNEL:
          HandleCloseChannelResponse(event);
          break;
        case MSG_SE_GET_ATR:
          HandleGetAtrResponse(event);
          break;
        case MSG_SE_RESET_SECUREELEMENT:
          HandleResetSecureElementResponse(event);
          break;
        case MSG_LS_EXECUTE_SCRIPT:
          HandleLsExecuteScriptResponse(event);
          break;
        case MSG_LS_GET_VERSION:
          HandleLsGetVersionResponse(event);
          break;
        case MSG_MPOS_READER_MODE:
          HandleMPOSReaderModeResponse(event);
          break;
        case MSG_SE_NOTIFY_MPOSREADERMODE_EVENT:
          HandleMPOSReaderModeEvent(event);
          break;
        case MSG_SE_FIELD_ACTIVATED:
          HandleSEFieldActivatedEvent();
          break;
        case MSG_SE_FIELD_DEACTIVATED:
          HandleSEFieldDeActivatedEvent();
          break;
        case MSG_LISTEN_MODE_ACTIVATED:
          HandleListenModeActivatedEvent();
          break;
        case MSG_LISTEN_MODE_DEACTIVATED:
          HandleListenModeDeActivatedEvent();
          break;
        case MSG_NFC_SELF_TEST:
          HandleNfcSelfTestResponse(event);
          break;
        default:
          NFCD_ERROR("NFCService bad message");
          abort();
      }

      //TODO delete event->data?
      delete event;
    }
  }
}

void NfcService::HandleSelfCrash(int signal) {
  NFCD_DEBUG("signal:%d", signal);
  mMsgHandler->ProcessNotification(NFC_NOTIFICATION_UNINITIALIZED , NULL);
}

void NfcService::HandleEnableDisableTimeout(bool enable) {
  if (enable) {
    mMsgHandler->ProcessNotification(NFC_NOTIFICATION_ENABLE_TIMEOUT , NULL);
  } else {
    mMsgHandler->ProcessNotification(NFC_NOTIFICATION_DISABLE_TIMEOUT , NULL);
  }
}

NfcService* NfcService::Instance() {
  if (!sInstance) {
    sInstance = new NfcService();
  }

  return sInstance;
}

INfcManager* NfcService::GetNfcManager()
{
  return reinterpret_cast<INfcManager*>(NfcService::sNfcManager);
}

bool NfcService::HandleDisconnect()
{
  INfcTag* pINfcTag = reinterpret_cast<INfcTag*>
                      (sNfcManager->QueryInterface(INTERFACE_TAG_MANAGER));
  bool result = pINfcTag->Disconnect();
  return result;
}

bool NfcService::HandleReadNdefRequest()
{
  NfcEvent *event = new NfcEvent(MSG_READ_NDEF);
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleReadNdefResponse(NfcEvent* aEvent)
{
  NfcResponseType resType = NFC_RESPONSE_READ_NDEF;

  INfcTag* pINfcTag = reinterpret_cast<INfcTag*>(sNfcManager->QueryInterface(INTERFACE_TAG_MANAGER));
  if (!pINfcTag) {
    mMsgHandler->ProcessResponse(resType, NFC_ERROR_NOT_SUPPORTED, NULL);
    return;
  }

  std::auto_ptr<NdefMessage> pNdefMessage(pINfcTag->ReadNdef());
  if (!pNdefMessage.get()) {
    mMsgHandler->ProcessResponse(resType, NFC_ERROR_READ, NULL);
    return;
  }

  mMsgHandler->ProcessResponse(resType, NFC_SUCCESS, pNdefMessage.get());
}

void NfcService::HandleReceiveNdefEvent(NfcEvent* aEvent)
{
  NdefMessage* ndef = reinterpret_cast<NdefMessage*>(aEvent->obj);

  NdefReceivedEvent* data = new NdefReceivedEvent();
  data->sessionId = SessionId::GetCurrentId();
  data->ndefMsgCount = ndef ? 1 : 0;
  data->ndefMsg = ndef;
  mMsgHandler->ProcessNotification(NFC_NOTIFICATION_NDEF_RECEIVED, data);

  delete data;
  delete ndef;
}

bool NfcService::HandleWriteNdefRequest(NdefMessage* aNdef, bool aIsP2P)
{
  NfcEvent *event = new NfcEvent(MSG_WRITE_NDEF);
  event->arg1 = aIsP2P;
  event->obj = aNdef;
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleWriteNdefResponse(NfcEvent* aEvent)
{
  NfcResponseType resType = NFC_RESPONSE_WRITE_NDEF;
  NfcErrorCode code = NFC_SUCCESS;

  std::auto_ptr<NdefMessage> pNdef(reinterpret_cast<NdefMessage*>(aEvent->obj));
  if (!pNdef.get()) {
    mMsgHandler->ProcessResponse(resType, NFC_ERROR_INVALID_PARAM, NULL);
    return;
  }

  bool isP2P = aEvent->arg1;
  if (isP2P && mP2pLinkManager->IsLlcpActive()) {
    mP2pLinkManager->Push(*pNdef.get());
  } else if (!isP2P && IsTagPresent()) {
    INfcTag* pINfcTag = reinterpret_cast<INfcTag*>
                        (sNfcManager->QueryInterface(INTERFACE_TAG_MANAGER));

    code = !!pINfcTag ?
           (pINfcTag->WriteNdef(*pNdef.get()) ? NFC_SUCCESS : NFC_ERROR_IO) :
           NFC_ERROR_NOT_SUPPORTED;
  } else {
    code = NFC_ERROR_IO;
  }

  mMsgHandler->ProcessResponse(resType, code, NULL);
}

void NfcService::OnConnected()
{
  NfcEvent *event = new NfcEvent(MSG_SOCKET_CONNECTED);
  mQueue.push_back(event);
  sem_post(&thread_sem);
}

bool NfcService::HandleMakeNdefReadonlyRequest()
{
  NfcEvent *event = new NfcEvent(MSG_MAKE_NDEF_READONLY);
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleMakeNdefReadonlyResponse(NfcEvent* aEvent)
{
  INfcTag* pINfcTag = reinterpret_cast<INfcTag*>
                      (sNfcManager->QueryInterface(INTERFACE_TAG_MANAGER));

  NfcErrorCode code = !!pINfcTag ?
                      (pINfcTag->MakeReadOnly() ? NFC_SUCCESS : NFC_ERROR_IO) :
                      NFC_ERROR_NOT_SUPPORTED;

  mMsgHandler->ProcessResponse(NFC_RESPONSE_MAKE_READ_ONLY, code, NULL);
}

bool NfcService::HandleNdefFormatRequest()
{
  NfcEvent *event = new NfcEvent(MSG_NDEF_FORMAT);
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

bool NfcService::HandleTagTransceiveRequest(int aTech, const uint8_t* aBuf, uint32_t aBufLen)
{
  std::vector<uint8_t>* cmd = new std::vector<uint8_t>(aBuf, aBuf + aBufLen);

  NfcEvent *event = new NfcEvent(MSG_TAG_TRANSCEIVE);
  event->arg1 = aTech;
  event->obj = reinterpret_cast<void*>(cmd);
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleTagTransceiveResponse(NfcEvent* aEvent)
{
  INfcTag* pINfcTag = reinterpret_cast<INfcTag*>
                      (sNfcManager->QueryInterface(INTERFACE_TAG_MANAGER));

  int tech = aEvent->arg1;
  std::vector<uint8_t>* command = reinterpret_cast<std::vector<uint8_t>*>(aEvent->obj);
  std::vector<uint8_t> response;

  NfcErrorCode code = pINfcTag->Connect(static_cast<TagTechnology>(tech)) ?
                      NFC_SUCCESS : NFC_ERROR_IO;

  if (NFC_SUCCESS == code) {
    code = !!pINfcTag ? (pINfcTag->Transceive(*command, response) ?
           NFC_SUCCESS : NFC_ERROR_IO) : NFC_ERROR_NOT_SUPPORTED;
  }

  delete command;

  mMsgHandler->ProcessResponse(NFC_RESPONSE_TAG_TRANSCEIVE, code,
                               reinterpret_cast<void*>(&response));
}

bool NfcService::HandleOpenChannelRequest()
{
  NFCD_DEBUG("NFCService HandleOpenChannelRequest");
  NfcEvent *event = new NfcEvent(MSG_SE_OPEN_CHANNEL);
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleOpenChannelResponse(NfcEvent* aEvent)
{
  NFCD_DEBUG("NFCService HandleOpenChannelResponse");
  NfcErrorCode code = NFC_SUCCESS;

  int handle = NfcSeManager::GetInstance().doOpenSecureElementConnection(0xC0);
  if (handle < 0) {
    code = NFC_ERROR_CONNECT;
  }

  mMsgHandler->ProcessResponse(NFC_RESPONSE_OPEN_CHANNEL, code, reinterpret_cast<void*>(&handle));
}

bool NfcService::HandleTransmitRequest(int handle, const uint8_t* aBuf, uint32_t aBufLen)
{
  std::vector<uint8_t>* cmd = new std::vector<uint8_t>(aBuf, aBuf + aBufLen);

  NfcEvent *event = new NfcEvent(MSG_SE_TRANSMIT);
  event->arg1 = handle;
  event->obj = reinterpret_cast<void*>(cmd);
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleTransmitResponse(NfcEvent* aEvent)
{
  NfcErrorCode code = NFC_SUCCESS;

  int handle = aEvent->arg1;
  std::vector<uint8_t>*command = reinterpret_cast<std::vector<uint8_t>*>(aEvent->obj);
  NFCD_DEBUG("NFCService HandleTransmitResponse handle = %d ", handle);

  //1024; 34k
  const INT32 recvBufferMaxSize = 1024;//SEC API buffer MAX_ESE_BUFFER_SIZE;
  UINT8 recvBuffer[recvBufferMaxSize];
  int size = NfcSeManager::GetInstance().doTransceive(handle, command->data(), command->size(), recvBuffer, recvBufferMaxSize);

  TransmitResponse *data = new TransmitResponse();
  data->handle = handle;

  if (size < 0) {
    NFCD_DEBUG("NFCService HandleTransmitResponse handle = %d size = %d", handle, size);
    code = NFC_ERROR_READ;
    data->response = NULL;
    mMsgHandler->ProcessResponse(NFC_RESPONSE_TRANSMIT, code,
                                 reinterpret_cast<void*>(data));
    return;
  }

  std::vector<uint8_t> response(recvBuffer, recvBuffer + size);
  data->response = reinterpret_cast<void*>(&response);
  mMsgHandler->ProcessResponse(NFC_RESPONSE_TRANSMIT, code,
                               reinterpret_cast<void*>(data));

  delete data;
}

bool NfcService::HandleCloseChannelRequest(int handle)
{
  NFCD_DEBUG("NFCService HandleCloseChannelRequest");
  NfcEvent *event = new NfcEvent(MSG_SE_CLOSE_CHANNEL);
  event->arg1 = handle;
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleCloseChannelResponse(NfcEvent* aEvent)
{
  NFCD_DEBUG("NFCService HandleCloseChannelResponse");
  NfcErrorCode code = NFC_SUCCESS;
  int handle = aEvent->arg1;

  bool status = NfcSeManager::GetInstance().doDisconnectSecureElementConnection(0xC0);
  if (!status) {
    code = NFC_ERROR_DISCONNECT;
  }
  mMsgHandler->ProcessResponse(NFC_RESPONSE_CLOSE_CHANNEL, code, reinterpret_cast<void*>(&handle));
}

bool NfcService::HandleGetAtrRequest(int handle)
{
  NFCD_DEBUG("NFCService HandleGetAtrRequest");
  NfcEvent *event = new NfcEvent(MSG_SE_GET_ATR);
  event->arg1 = handle;
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleGetAtrResponse(NfcEvent* aEvent)
{
  NFCD_DEBUG("NFCService HandleGetAtrResponse");
  NfcErrorCode code = NFC_SUCCESS;
  int handle = aEvent->arg1;

  const INT32 recvBufferMaxSize = 1024;// SEC API buffer MAX_ESE_BUFFER_SIZE;
  UINT8 recvBuffer[recvBufferMaxSize];

  int size = NfcSeManager::GetInstance().doGetAtrResponse(handle, recvBuffer, recvBufferMaxSize);

  TransmitResponse *data = new TransmitResponse();
  data->handle = handle;

  if (size < 0) {
    code = NFC_ERROR_READ;
    data->response = NULL;
    mMsgHandler->ProcessResponse(NFC_RESPONSE_GET_ATR, code,
                                 reinterpret_cast<void*>(data));
    return;
  }

  std::vector<uint8_t> response(recvBuffer, recvBuffer + size);
  data->response = reinterpret_cast<void*>(&response);
  mMsgHandler->ProcessResponse(NFC_RESPONSE_GET_ATR, code,
                               reinterpret_cast<void*>(data));
}

bool NfcService::HandleResetSecureElementRequest(int handle)
{
  NFCD_DEBUG("Enter");
  NfcEvent *event = new NfcEvent(MSG_SE_RESET_SECUREELEMENT);
  event->arg1 = handle;
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleResetSecureElementResponse(NfcEvent* aEvent)
{
  NFCD_DEBUG("Enter");
  // Reset secure element is not supported at the moment.
  NfcErrorCode code = NFC_ERROR_NOT_SUPPORTED;

  int handle = aEvent->arg1;

  mMsgHandler->ProcessResponse(NFC_RESPONSE_RESET_SECUREELEMENT, code, reinterpret_cast<void*>(&handle));

}

bool NfcService::HandleLsExecuteScriptRequest(std::string& lsScriptFile,
                                              std::string& lsResponseFile,
                                              std::string& uniqueAppID)
{
  NFCD_DEBUG("Enter");
  NfcEvent *event = new NfcEvent(MSG_LS_EXECUTE_SCRIPT);

  LoaderServiceEvent *data = new LoaderServiceEvent();
  data->lsExecuteScriptFile = lsScriptFile;
  data->lsResponseFile = lsResponseFile;
  data->uniqueAppID = uniqueAppID;

  event->obj = reinterpret_cast<void*>(data);
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleLsExecuteScriptResponse(NfcEvent* aEvent)
{
  NfcErrorCode code = NFC_SUCCESS;

  void* pLoaderService = aEvent->obj;
  int status = -1;

  // status = call SEC API;
  std::vector<char> response;
  if (status != 0) {
    code = NFC_ERROR_IO;
  }

  mMsgHandler->ProcessResponse(NFC_RESPONSE_LS_EXECUTE_SCRIPT, code, reinterpret_cast<void*>(&response));
}

bool NfcService::HandleLsGetVersionRequest()
{
  NFCD_DEBUG("NFCService HandleLsGetVersionRequest");
  NfcEvent *event = new NfcEvent(MSG_LS_GET_VERSION);
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleLsGetVersionResponse(NfcEvent* aEvent)
{
  NFCD_DEBUG("NFCService HandleLsScriptExecuteResponse");
  NfcErrorCode code = NFC_SUCCESS;

  std::vector<char> response;
  // call SEC API to get version;
  mMsgHandler->ProcessResponse(NFC_RESPONSE_LS_GET_VERSION, code, reinterpret_cast<void*>(&response));
}

bool NfcService::HandleMPOSReaderModeRequest(bool enabled)
{
  NfcEvent *event = new NfcEvent(MSG_MPOS_READER_MODE);
  event->arg1 = enabled;
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleMPOSReaderModeResponse(NfcEvent* aEvent)
{
  bool enabled = aEvent->arg1;

  NfcErrorCode code = NFC_SUCCESS;

  NFCD_DEBUG("NFCService HandleMPOSReaderModeResponse %d", enabled);

  bool result = sNfcManager->SetmPOSDedicatedMode(enabled);
  if (!result) {
    code = NFC_ERROR_BUSY;
  }

  mMsgHandler->ProcessResponse(NFC_RESPONSE_MPOS_READER_MODE, code, NULL);
}

bool NfcService::HandleNfcSelfTestRequest(int type)
{
  NfcEvent *event = new NfcEvent(MSG_NFC_SELF_TEST);
  event->arg1 = type;
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleNfcSelfTestResponse(NfcEvent* aEvent)
{
  int type = aEvent->arg1;

  NfcErrorCode code = NFC_SUCCESS;

  NFCD_DEBUG("NFCService HandleNfcSelfTestResponse %d", type);
  int status = sNfcManager->NfcSelfTest(type);
  if (status != 0) {
    code = NFC_ERROR_IO;
  }

  mMsgHandler->ProcessResponse(NFC_RESPONSE_NFC_SELF_TEST, code, NULL);
}

void NfcService::HandleNdefFormatResponse(NfcEvent* aEvent)
{
  INfcTag* pINfcTag = reinterpret_cast<INfcTag*>
                      (sNfcManager->QueryInterface(INTERFACE_TAG_MANAGER));

  NfcErrorCode code = !!pINfcTag ?
                      (pINfcTag->FormatNdef() ? NFC_SUCCESS : NFC_ERROR_IO) :
                      NFC_ERROR_NOT_SUPPORTED;

  mMsgHandler->ProcessResponse(NFC_RESPONSE_FORMAT, code, NULL);
}

bool NfcService::HandleEnterLowPowerRequest(int powerMode)
{
  NfcEvent *event = new NfcEvent(MSG_LOW_POWER);
  event->arg1 = powerMode;
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

void NfcService::HandleEnterLowPowerResponse(NfcEvent* aEvent)
{
  NfcStopPollPowerMode powerMode = (NfcStopPollPowerMode)aEvent->arg1;

  NfcErrorCode code = SetLowPowerMode(powerMode);

  int newState = mState;

  if (mState == STATE_NFC_ON_LOW_POWER_SCREEN_OFF || mState == STATE_NFC_ON_LOW_POWER_SCREEN_LOCK ||
    mState == STATE_NFC_ON_ULTRA_LOW_POWER) {
    newState = STATE_NFC_ON_LOW_POWER;
  }
  NFCD_DEBUG("mState=%d newState=%d", mState, newState);
  mMsgHandler->ProcessResponse(NFC_RESPONSE_CHANGE_RF_STATE, code, &newState);
}

bool NfcService::HandleEnableRequest(bool aEnable)
{
  NfcEvent *event = new NfcEvent(MSG_ENABLE);
  event->arg1 = aEnable;
  mQueue.push_back(event);
  sem_post(&thread_sem);
  return true;
}

/**
 * There are two case for enable:
 * 1. NFC is off -> enable NFC and then enable discovery.
 * 2. NFC is already on but discovery mode is off -> enable discovery.
 */
void NfcService::HandleEnableResponse(NfcEvent* aEvent)
{
  NfcErrorCode code = NFC_SUCCESS;

  bool enable = aEvent->arg1;
  if (enable) {
    // Disable low power mode if already in low power mode
    if (mState == STATE_NFC_ON_LOW_POWER ||
        mState == STATE_NFC_ON_LOW_POWER_SCREEN_OFF ||
        mState == STATE_NFC_ON_LOW_POWER_SCREEN_LOCK ||
        mState == STATE_NFC_ON_ULTRA_LOW_POWER) {
      code = SetLowPowerMode(MODE_FULL_POWER);
    } else if (mState == STATE_NFC_OFF) {
      code = EnableNfc();
    }
  } else {
    code = DisableNfc();
  }

  NFCD_DEBUG("mState=%d", mState);
  mMsgHandler->ProcessResponse(NFC_RESPONSE_CHANGE_RF_STATE, code, &mState);
}

NfcErrorCode NfcService::EnableNfc()
{
  NFCD_DEBUG("Enable NFC");

  if (mState != STATE_NFC_OFF) {
    return NFC_SUCCESS;
  }

  enable_disable_timer.Set (ENABLE_DISABLE_TIMEOUT, NfcService::EnableTimeout);

  if (!sNfcManager->Initialize()) {
    enable_disable_timer.Kill();
    return NFC_ERROR_INITIALIZE_FAIL;
  }

  if (mP2pLinkManager) {
    mP2pLinkManager->EnableDisable(true);
  }

  // TODO: Emulator doesn't support SE now so do not do fail return here.
  // Put EnableSecureEleemnt before EnableDiscovery to avoid redundant
  // turning on/off RF
  if (!sNfcManager->EnableSecureElement()) {
    NFCD_DEBUG("Enable secure element not succeed");
  }
  // we need to change SCREEN ON
#if 1
  sNfcManager->SetScreenOrPowerState(SCREEN_STATE_UNLOCKED);
#endif

  if (!sNfcManager->EnableDiscovery()) {
    enable_disable_timer.Kill();
    return NFC_ERROR_FAIL_ENABLE_DISCOVERY;
  }

  mState = STATE_NFC_ON;
  enable_disable_timer.Kill();

  return NFC_SUCCESS;
}

NfcErrorCode NfcService::DisableNfc()
{
  NFCD_DEBUG("Disable NFC");

  if (mState == STATE_NFC_OFF) {
    return NFC_SUCCESS;
  }

  enable_disable_timer.Set (ENABLE_DISABLE_TIMEOUT, NfcService::DisableTimeout);

  if (mP2pLinkManager) {
    mP2pLinkManager->EnableDisable(false);
  }

  // TODO: Emulator doesn't support SE now so do not do fail return here.
  // Put DisableSecureEleemnt before DisableDiscovery to avoid redundant
  // turning on/off RF
  if (!sNfcManager->DisableSecureElement()) {
    NFCD_DEBUG("Disable secure element not succeed");
  }

  if (!sNfcManager->DisableDiscovery()) {
    enable_disable_timer.Kill();
    return NFC_ERROR_FAIL_DISABLE_DISCOVERY;
  }

  if (!sNfcManager->Deinitialize()) {
    enable_disable_timer.Kill();
    return NFC_ERROR_DEINITIALIZE_FAIL;
  }

  mState = STATE_NFC_OFF;
  enable_disable_timer.Kill();

  return NFC_SUCCESS;
}

NfcErrorCode NfcService::SetLowPowerMode(NfcStopPollPowerMode mode) {

 if ((mode == MODE_LOW_POWER  && mState == STATE_NFC_ON_LOW_POWER) ||
     (mode == MODE_FULL_POWER && mState == STATE_NFC_ON)) {
    return NFC_SUCCESS;
 }

  NFCD_DEBUG("SetLowPowerMode %d", mode);
  switch (mode) {
    case MODE_LOW_POWER:
      if (!sNfcManager->DisableDiscovery()) {
        return NFC_ERROR_FAIL_ENABLE_LOW_POWER_MODE;
      }
      mState = STATE_NFC_ON_LOW_POWER;
      break;

    case MODE_FULL_POWER:
    // we need to change SCREEN ON
    #if 1
      mState = STATE_NFC_ON;
      sNfcManager->SetScreenOrPowerState(3);// 3 means SCREEN_UN_LOCK
      sNfcManager->EnableDiscovery();
    #else
      if (!sNfcManager->EnableP2pListening() ||
          !sNfcManager->EnablePolling()) {
        return NFC_ERROR_FAIL_DISABLE_LOW_POWER_MODE;
      }
      mState = STATE_NFC_ON;
    #endif
      break;

    case MODE_SCREEN_OFF:
      mState = STATE_NFC_ON_LOW_POWER_SCREEN_OFF;
      sNfcManager->SetScreenOrPowerState(SCREEN_STATE_OFF);
      //SEC call DisableDiscovery
      sNfcManager->DisableDiscovery();
      break;

    case MODE_SCREEN_LOCK:
      mState = STATE_NFC_ON_LOW_POWER_SCREEN_LOCK;
      sNfcManager->SetScreenOrPowerState(SCREEN_STATE_LOCKED);
      //SEC call DisableDiscovery
      sNfcManager->DisableDiscovery();
      break;

    case MODE_ULTRA_LOW_POWER:
      mState = STATE_NFC_ON_ULTRA_LOW_POWER;
      sNfcManager->SetScreenOrPowerState(STATE_ULTRA_LOW_POWER);
      //SEC call DisableDiscovery
      sNfcManager->DisableDiscovery();
      break;

    default:
      break;
  }

  return NFC_SUCCESS;
}

void NfcService::OnP2pReceivedNdef(NdefMessage* aNdef)
{
  NfcEvent *event = new NfcEvent(MSG_RECEIVE_NDEF_EVENT);

  event->obj = aNdef ? new NdefMessage(aNdef) : NULL;

  mQueue.push_back(event);
  sem_post(&thread_sem);
}
