/*
 * Copyright (C) 2014  Mozilla Foundation
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
#include <stdlib.h>
#include "SecureElement.h"
#include "NfcDebug.h"
#include "PowerSwitch.h"
#include "config.h"
#include "NfcNciUtil.h"
#include "DeviceHost.h"
#include "NfcManager.h"

SecureElement SecureElement::sSecElem;
const char* SecureElement::APP_NAME = "nfc";

/* START [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
int gEseChipType             = 0x00;  // 0x00 : Oberthur, 0x01 : Gemalto
int gSEId                    = 0x02;  // secure element ID to use in connectEE(), -1 means not set
int gGatePipe                = -1;    // gate id or static pipe id to use in connectEE(), -1 means not set
bool gUseStaticPipe          = true;  // if true, use gGatePipe as static pipe id.  if false, use as gate id
bool gIsSupportConcurrentWCE = false; // if true, support concurrent wired C/E.
bool gIsSetWcebyPropCmd      = false;
/* END [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/

extern void StartRfDiscovery (bool isStart);
/* START [17041401J] Adjust device sleep timeout for RaboBank*/
extern void slsiSetNfcSleepTimeout(unsigned long sec, int option);
/* END [17041401J] Adjust device sleep timeout for RaboBank*/

// [START] System LSI - event source
struct transactionData {
  int     len;
  UINT8   pipe;
  UINT8*  data;
  struct transactionData *next;
} *trsDataFirst, *trsDataLast;
Mutex           trsMutext; // protects fields below

void SecureElement::notifyTransaction(void)
{
  struct transactionData *trsDataRemove = NULL;

  NCI_DEBUG ("%s: enter;", __FUNCTION__);
  trsMutext.Lock();
  if (trsDataFirst == NULL)
  {
    trsMutext.Unlock();
    NCI_DEBUG ("%s: exit, its empty;", __FUNCTION__);
    return;
  }

  do {
    trsDataRemove = trsDataFirst;
    trsDataFirst = trsDataFirst->next;

    int evtSrc = SecureElement::sSecElem.findDestByPipe(trsDataRemove->pipe);
    int aidLen = trsDataRemove->data[1];
    UINT8 *aid = &trsDataRemove->data[2];
    UINT8 dataLen = 0;
    UINT8 *data = NULL;

    NCI_DEBUG ("%s: evtSrc is %d", __FUNCTION__, evtSrc);
    NCI_DEBUG ("%s: trsDataRemove len = %d", __FUNCTION__, trsDataRemove->len);
    NCI_DEBUG ("%s: trsDataRemove data = %d", __FUNCTION__, trsDataRemove->data[2+aidLen]);
    if ((trsDataRemove->len > 2+aidLen) && (trsDataRemove->data[2+aidLen] == 0x82))
    {
      if (trsDataRemove->data[2+aidLen+1] == 0x81)
      {
        dataLen = trsDataRemove->data[2 + aidLen + 2];
        data = &trsDataRemove->data[2 + aidLen + 3];
        NCI_DEBUG ("%s: first: dataLen is %d", __FUNCTION__, dataLen);
      }
      else
      {
        dataLen = trsDataRemove->data[2 + aidLen + 1];
        data = &trsDataRemove->data[2 + aidLen + 2];
        NCI_DEBUG ("%s: second: dataLen is %d", __FUNCTION__, dataLen);
      }
    }
    SecureElement::sSecElem.notifyTransactionListenersOfAid (aid, aidLen, data, dataLen, evtSrc);

    free(trsDataRemove->data);
    free(trsDataRemove);

  } while (trsDataFirst != NULL);

  trsDataLast = NULL;
  trsMutext.Unlock();
  NCI_DEBUG ("%s: exit;", __FUNCTION__);
}

bool SecureElement::requestNotifyTransaction(UINT8 pipe, UINT8 *data, int len)
{
  struct transactionData *trsDataNew = NULL;

  NCI_DEBUG ("%s: enter;", __FUNCTION__);
  trsMutext.Lock();

  trsDataNew = (struct transactionData *)malloc(sizeof(struct transactionData));
  if (trsDataNew == NULL)
  {
    NCI_DEBUG ("%s: struct allocate failed.", __FUNCTION__);
    trsMutext.Unlock();
    return false;
  }
  trsDataNew->data = (UINT8*)malloc(len);
  if (trsDataNew == NULL)
  {
    NCI_DEBUG ("%s: data allocate failed.", __FUNCTION__);
    free(trsDataNew);
    trsMutext.Unlock();
    return false;
  }
  memcpy(trsDataNew->data, data, len);
  trsDataNew->len = len;
  trsDataNew->pipe = pipe;
  trsDataNew->next = NULL;

  if (trsDataLast != NULL)
    trsDataLast->next = trsDataNew;
  trsDataLast = trsDataNew;

  if (trsDataFirst == NULL)
  {
    NCI_DEBUG ("%s: Get pipe list to find the source of the event", __FUNCTION__);
    tNFA_STATUS nfaStat = NFA_HciGetGateAndPipeList (SecureElement::sSecElem.mNfaHciHandle);
    if (nfaStat != NFA_STATUS_OK)
    {
      NCI_DEBUG ("%s: NFA_HciGetGateAndPipeList failed.", __FUNCTION__);
      free(trsDataNew->data);
      free(trsDataNew);
      trsMutext.Unlock();
      return false;
    }
    trsDataFirst = trsDataNew;
  }

  trsMutext.Unlock();
  NCI_DEBUG ("%s: exit;", __FUNCTION__);
  return true;
}
// [END] System LSI - event source

SecureElement::SecureElement()
 : mActiveEeHandle(NFA_HANDLE_INVALID)
 , mDestinationGate (4) //loopback gate
 , mNfaHciHandle(NFA_HANDLE_INVALID)
 , mIsInit(false)
 , mActualNumEe(0)
 , mNumEePresent(0)
 , mbNewEE(true)   // by default we start w/thinking there are new EE
 , mNewPipeId (0)
 , mNewSourceGate (0)
 , mActiveSeOverride(0)
 , mIsPiping(false)
 , mCurrentRouteSelection(NoRoute)
 ,mActualResponseSize(0)
 , mUseOberthurWarmReset (false)
 , mActivatedInListenMode(false)
/* START [J14111101_Part3] - pending enable discovery during listen mode */
 , mPeerFieldInListenMode (false)
/* END [J14111101_Part3] - pending enable discovery during listen mode */
 , mOberthurWarmResetCommand (3)
 , mRfFieldIsOn(false)
{
  memset(&mEeInfo, 0, sizeof(mEeInfo));
  memset(&mUiccInfo, 0, sizeof(mUiccInfo));
  memset (&mHciCfg, 0, sizeof(mHciCfg));
  memset(&mLastRfFieldToggle, 0, sizeof(mLastRfFieldToggle));
// [START] System LSI - event source
  trsDataFirst = trsDataLast = NULL;
// [END] System LSI - event source

  memset(&mSeTechMaskForSubScreen, 0x03, sizeof(mSeTechMaskForSubScreen));
  memset(&mSeProtoMaskForSubScreen, 0x08, sizeof(mSeProtoMaskForSubScreen));
}

SecureElement::~SecureElement()
{
}

SecureElement& SecureElement::GetInstance()
{
  return sSecElem;
}

void SecureElement::SetActiveSeOverride(uint8_t aActiveSeOverride)
{
  NCI_DEBUG("seid=0x%X", aActiveSeOverride);
  mActiveSeOverride = aActiveSeOverride;
}

bool SecureElement::Initialize(NfcManager* aNfcManager)
{
  tNFA_STATUS nfaStat;
  unsigned long num = 0;

  // active SE, if not set active all SEs.
  if (GetNumValue("ACTIVE_SE", &num, sizeof(num)))
  {
    mActiveSeOverride = num;
  }
  NCI_DEBUG("Active SE override: 0x%X", mActiveSeOverride);

  mNfcManager = aNfcManager;

  mActiveEeHandle = NFA_HANDLE_INVALID;
  mNfaHciHandle   = NFA_HANDLE_INVALID;
  mActualNumEe    = MAX_NUM_EE;
  mbNewEE         = true;
  mRfFieldIsOn    = false;
  mActivatedInListenMode = false;
/* START [J14111101_Part3] - pending enable discovery during listen mode */
  mPeerFieldInListenMode = false;
/* END [J14111101_Part3] - pending enable discovery during listen mode */
  mCurrentRouteSelection = NoRoute;
  mNumEePresent = 0;
  mIsPiping = false;
  memset(mEeInfo, 0, sizeof(mEeInfo));
  memset(&mUiccInfo, 0, sizeof(mUiccInfo));

  // Get Fresh EE info.
  if (!GetEeInfo())
  {
    return false;
  }

  {
    SyncEventGuard guard(mEeRegisterEvent);
    NCI_DEBUG("try ee register");
    nfaStat = NFA_EeRegister(NfaEeCallback);
    if (nfaStat != NFA_STATUS_OK)
    {
      NCI_ERROR("fail ee register; error=0x%X", nfaStat);
      return false;
    }
    mEeRegisterEvent.Wait();
  }

  // If the controller has an HCI Network, register for that.
  for (size_t i = 0; i < mActualNumEe; i++)
  {
    if ((mEeInfo[i].num_interface <= 0) ||
        (mEeInfo[i].ee_interface[0] != NCI_NFCEE_INTERFACE_HCI_ACCESS))
    {
      continue;
    }

    NCI_DEBUG("Found HCI network, try hci register");

    SyncEventGuard guard(mHciRegisterEvent);

    nfaStat = NFA_HciRegister(const_cast<char*>(APP_NAME), NfaHciCallback, true);
    if (nfaStat != NFA_STATUS_OK) {
      NCI_ERROR("fail hci register; error=0x%X", nfaStat);
      return false;
    }
    mHciRegisterEvent.Wait();
    break;
  }
  mIsInit = true;

  return true;
}

void SecureElement::Finalize()
{
  NCI_DEBUG("enter");

  NFA_EeDeregister(NfaEeCallback);

  if (mNfaHciHandle != NFA_HANDLE_INVALID)
  {
    NFA_HciDeregister(const_cast<char*>(APP_NAME));
  }

  mNfaHciHandle = NFA_HANDLE_INVALID;
  mIsInit = false;
  mActualNumEe  = 0;
  mNumEePresent = 0;
  mNewPipeId    = 0;
  mNewSourceGate = 0;
  mIsPiping = false;
  memset (mEeInfo, 0, sizeof(mEeInfo));
  memset (&mUiccInfo, 0, sizeof(mUiccInfo));
}

bool SecureElement::GetEeInfo()
{
  NCI_DEBUG("enter; mbNewEE=%d, mActualNumEe=%d", mbNewEE, mActualNumEe);
  tNFA_STATUS nfaStat = NFA_STATUS_FAILED;

  if (!mbNewEE)
  {
    return (mActualNumEe != 0);
  }

  // If mbNewEE is true then there is new EE info.
  mActualNumEe = MAX_NUM_EE;

  if ((nfaStat = NFA_EeGetInfo(&mActualNumEe, mEeInfo)) != NFA_STATUS_OK)
  {
    NCI_ERROR("fail get info; error=0x%X", nfaStat);
    mActualNumEe = 0;
    return false;
  }

  mbNewEE = false;

  NCI_DEBUG("num EEs discovered: %u", mActualNumEe);
  for (uint8_t i = 0; i < mActualNumEe; i++)
  {
    if ((mEeInfo[i].num_interface != 0) &&
        (mEeInfo[i].ee_interface[0] != NCI_NFCEE_INTERFACE_HCI_ACCESS))
    {
      mNumEePresent++;
    }

    NCI_DEBUG("EE[%u] Handle: 0x%04x  Status: %s  Num I/f: %u: (0x%02x, 0x%02x)  Num TLVs: %u",
              i,
              mEeInfo[i].ee_handle,
              EeStatusToString(mEeInfo[i].ee_status),
              mEeInfo[i].num_interface,
              mEeInfo[i].ee_interface[0],
              mEeInfo[i].ee_interface[1],
              mEeInfo[i].num_tlvs);

    for (size_t j = 0; j < mEeInfo[i].num_tlvs; j++)
    {
      NCI_DEBUG("EE[%u] TLV[%u]  Tag: 0x%02x  Len: %u  Values[]: 0x%02x  0x%02x  0x%02x ...",
                i, j,
                mEeInfo[i].ee_tlv[j].tag,
                mEeInfo[i].ee_tlv[j].len,
                mEeInfo[i].ee_tlv[j].info[0],
                mEeInfo[i].ee_tlv[j].info[1],
                mEeInfo[i].ee_tlv[j].info[2]);
    }
  }
  NCI_DEBUG("exit; mActualNumEe=%d, mNumEePresent=%d", mActualNumEe, mNumEePresent);
  return (mActualNumEe != 0);
}

/**
 * Computes time difference in milliseconds.
 */
static uint32_t TimeDiff(timespec aStart, timespec aEnd)
{
  aEnd.tv_sec -= aStart.tv_sec;
  aEnd.tv_nsec -= aStart.tv_nsec;

  if (aEnd.tv_nsec < 0)
  {
    aEnd.tv_nsec += 10e8;
    aEnd.tv_sec -= 1;
  }

  return (aEnd.tv_sec * 1000) + (aEnd.tv_nsec / 10e5);
}

bool SecureElement::IsRfFieldOn()
{
  AutoMutex mutex(mMutex);
  if (mRfFieldIsOn)
  {
    return true;
  }

  struct timespec now;
  int ret = clock_gettime(CLOCK_MONOTONIC, &now);
  if (ret == -1)
  {
    NCI_ERROR("isRfFieldOn(): clock_gettime failed");
    return false;
  }

  // If it was less than 50ms ago that RF field
  // was turned off, still return ON.
  return (TimeDiff(mLastRfFieldToggle, now) < 50);
}

bool SecureElement::IsActivatedInListenMode()
{
  return mActivatedInListenMode;
}

/* START [J14111101_Part3] - pending enable discovery during listen mode */
/*******************************************************************************
**
** Function:        setIsActivatedInListenMode
**
** Description:     Can be used to determine if the SE is activated in listen mode
**
** Returns:         True if the SE is activated in listen mode
**
*******************************************************************************/
void SecureElement::SetIsPeerInListenMode(bool isActivated)
{
	mPeerFieldInListenMode = isActivated;
}

bool SecureElement::IsPeerInListenMode()
{
    return mPeerFieldInListenMode;
}
/* END [J14111101_Part3] - pending enable discovery during listen mode */

void SecureElement::GetListOfEeHandles(std::vector<uint32_t>& aListSe)
{
  NCI_DEBUG("enter");

  int cnt = 0;

  if (mNumEePresent == 0 || !mIsInit || !GetEeInfo())
  {
    return;
  }

  for (int i = 0; i < mActualNumEe && cnt < mNumEePresent; i++)
  {
    NCI_DEBUG("%u = 0x%X", i, mEeInfo[i].ee_handle);
    if ((mEeInfo[i].num_interface == 0) ||
        (mEeInfo[i].ee_interface[0] == NCI_NFCEE_INTERFACE_HCI_ACCESS))
    {
      continue;
    }
    uint32_t handle = mEeInfo[i].ee_handle & ~NFA_HANDLE_GROUP_EE;
    aListSe.push_back(handle);
    cnt++;
  }
}

bool SecureElement::Activate()
{
  int numActivatedEe = 0;

  NCI_DEBUG("enter;");

  if (!mIsInit)
  {
    NCI_ERROR("not init");
    return false;
  }

  if (mActiveEeHandle != NFA_HANDLE_INVALID)
  {
    NCI_DEBUG("already active");
    return true;
  }

  // Get Fresh EE info if needed.
  if (!GetEeInfo())
  {
    NCI_ERROR("no EE info");
    return false;
  }

  uint16_t overrideEeHandle =
    mActiveSeOverride ? NFA_HANDLE_GROUP_EE | mActiveSeOverride : 0;

  if (mRfFieldIsOn)
  {
    NCI_ERROR("RF field indication still on, resetting");
    mRfFieldIsOn = false;
  }

  NCI_DEBUG("override ee h=0x%X", overrideEeHandle);
  //activate every discovered secure element
  for (int i = 0; i < mActualNumEe; i++)
  {
    tNFA_EE_INFO& eeItem = mEeInfo[i];
    if ((eeItem.ee_handle != EE_HANDLE_ESE) &&
        (eeItem.ee_handle != EE_HANDLE_UICC))
    {
      continue;
    }

    if (overrideEeHandle != eeItem.ee_handle)
    {
      continue;   // do not enable all SEs; only the override one
    }

    if (eeItem.ee_status != NFC_NFCEE_STATUS_INACTIVE)
    {
      NCI_DEBUG("h=0x%X already activated", eeItem.ee_handle);
      numActivatedEe++;
      continue;
    }

    {
      tNFA_STATUS nfaStat = NFA_STATUS_FAILED;
      SyncEventGuard guard(mEeSetModeEvent);
      NCI_DEBUG("set EE mode activate; h=0x%X", eeItem.ee_handle);
      if ((nfaStat = NFA_EeModeSet(eeItem.ee_handle, NFA_EE_MD_ACTIVATE)) == NFA_STATUS_OK)
      {
        mEeSetModeEvent.Wait(); //wait for NFA_EE_MODE_SET_EVT
        if (eeItem.ee_status == NFC_NFCEE_STATUS_ACTIVE)
        {
          numActivatedEe++;
        }
      }
      else
      {
        NCI_ERROR("NFA_EeModeSet failed; error=0x%X", nfaStat);
      }
    } //for
  }

  mActiveEeHandle = GetDefaultEeHandle();

  NCI_DEBUG("exit; active ee h=0x%X", mActiveEeHandle);

  return mActiveEeHandle != NFA_HANDLE_INVALID;
}

bool SecureElement::Deactivate()
{
  bool retval = false;

  NCI_DEBUG("enter; mActiveEeHandle=0x%X", mActiveEeHandle);

  if (!mIsInit)
  {
    NCI_ERROR("not init");
    return retval;
  }

  // if the controller is routing to sec elems or piping,
  // then the secure element cannot be deactivated
  if (IsBusy())
  {
    NCI_ERROR("still busy");
    return retval;
  }
  else if (mActiveEeHandle == NFA_HANDLE_INVALID)
  {
    NCI_ERROR("invalid EE handle");
    return retval;
  }

  mActiveEeHandle = NFA_HANDLE_INVALID;
  retval = true;

  NCI_DEBUG("exit; ok=%u", retval);
  return retval;
}

/* START [D18020702] - Notify application EVT_Transaction */
void SecureElement::notifyTransactionListenersOfAid (const UINT8* aidBuffer, UINT8 aidBufferLen, const UINT8* dataBuffer, UINT8 dataBufferLen, UINT32 evtSrc)
{
    static const char fn [] = "SecureElement::notifyTransactionListenersOfAid";
    NCI_DEBUG ("enter; aid len=%u", aidBufferLen);
    NCI_DEBUG ("enter; dataBuffer len=%u", dataBufferLen);
    NCI_DEBUG ("event source %lu", evtSrc);

    if (aidBufferLen == 0) {
        return;
    }

    NotifyTransactionEvent(aidBuffer, aidBufferLen, dataBuffer, dataBufferLen);

TheEnd:
    NCI_DEBUG ("%s: exit", fn);
}
/* END [D18020702] - Notify application EVT_Transaction */

void SecureElement::NotifyTransactionEvent(const uint8_t* aAid,
                                           uint32_t aAidLen,
                                           const uint8_t* aPayload,
                                           uint32_t aPayloadLen)
{
  if (aAidLen == 0)
  {
    return;
  }

  TransactionEvent* pTransaction = new TransactionEvent();

  // TODO: For now, we dodn't have a solution to get aid origin from nfcd.
  //       So use eSE as dfault value.
/* START [D18020702] - Notify application EVT_Transaction */
  pTransaction->originType = TransactionEvent::ESE;
/* END [D18020702] - Notify application EVT_Transaction */
  pTransaction->originIndex = 1;

  pTransaction->aidLen = aAidLen;
  pTransaction->aid = new uint8_t[aAidLen];
  memcpy(pTransaction->aid, aAid, aAidLen);

  pTransaction->payloadLen = aPayloadLen;
  pTransaction->payload = new uint8_t[aPayloadLen];
  memcpy(pTransaction->payload, aPayload, aPayloadLen);

  mNfcManager->NotifyTransactionEvent(pTransaction);
}

void SecureElement::NotifyListenModeState(bool aIsActivated)
{
  NCI_DEBUG("enter; listen mode active=%u", aIsActivated);

  // TODO Implement notify.
  mActivatedInListenMode = aIsActivated;
}

void SecureElement::NotifyRfFieldEvent(bool aIsActive)
{
  NCI_DEBUG("enter; is active=%u", aIsActive);

  // TODO Implement
  mMutex.Lock();
  int ret = clock_gettime(CLOCK_MONOTONIC, &mLastRfFieldToggle);
  if (ret == -1) {
    NCI_ERROR("clock_gettime failed");
    // There is no good choice here...
  }

  mRfFieldIsOn = aIsActive;
  mMutex.Unlock();
}

/*******************************************************************************
**
** Function:        connectEE
**
** Description:     Connect to the execution environment.
**
** Returns:         True if ok.
**
*******************************************************************************/
bool SecureElement::ConnectEE ()
{
  static const char fn [] = "SecureElement::connectEE";
  tNFA_STATUS nfaStat = NFA_STATUS_FAILED;
  bool        retVal = false;
  uint8_t       destHost = 0;
  unsigned long num = 0;
  char pipeConfName[40];
  tNFA_HANDLE  eeHandle = mActiveEeHandle;

  NCI_DEBUG ("enter, mActiveEeHandle: 0x%04x, SEID: 0x%x, pipe_gate_num=%d, use pipe=%d",
      mActiveEeHandle, gSEId, gGatePipe, gUseStaticPipe);

  if (!mIsInit)
  {
    NCI_ERROR ("not init");
    return (false);
  }

  if (gSEId != -1)
  {
    eeHandle = gSEId | NFA_HANDLE_GROUP_EE;
    NCI_DEBUG ("Using SEID: 0x%x", eeHandle );
  }

  if (eeHandle == NFA_HANDLE_INVALID)
  {
    NCI_ERROR ("invalid handle 0x%X", eeHandle);
    return (false);
  }

  tNFA_EE_INFO *pEE = FindEeByHandle (eeHandle);

  if (pEE == NULL)
  {
    NCI_ERROR ("Handle 0x%04x  NOT FOUND !!", eeHandle);
    return (false);
  }

  /* START [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
  if(gIsSupportConcurrentWCE != true)
  {
    // Disable RF discovery completely while the DH is connected
    StartRfDiscovery(false);
  }
  /* END [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
  mNewSourceGate = 0;

  /* START [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
  if (gUseStaticPipe)
  {
    if (gGatePipe == -1)
    {
      // pipe/gate num was not specifed by app, get from config file
      mNewPipeId     = 0;

      // Construct the PIPE name based on the EE handle (e.g. NFA_HCI_STATIC_PIPE_ID_F3 for UICC0).
      snprintf (pipeConfName, sizeof(pipeConfName), "NFA_HCI_STATIC_PIPE_ID_%02X", eeHandle & NFA_HANDLE_MASK);

      if (GetNumValue(pipeConfName, &num, sizeof(num)) && (num != 0))
      {
        mNewPipeId = num;
        NCI_DEBUG ("%s: Using static pipe id: 0x%X", __FUNCTION__, mNewPipeId);
      }
      else
      {
        NCI_DEBUG ("%s: Did not find value '%s' defined in the .conf", __FUNCTION__, pipeConfName);
      }
    }
    else
    {
      mNewPipeId     = gGatePipe;
    }
  }
  else
  {
    mNewPipeId      = 0;
    mDestinationGate= gGatePipe;
  }

  if (gUseStaticPipe)
/* END [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
  {
    // [START] wired C/E
    uint8_t host = 0x01;
    uint8_t gate = 0xF0;
    // [END] wired C/E

    nfaStat = NFA_HciAddStaticPipe(mNfaHciHandle, host, gate, mNewPipeId);
    if (nfaStat != NFA_STATUS_OK)
    {
      NCI_ERROR ("fail create static pipe; error=0x%X", nfaStat);
      retVal = false;
      goto TheEnd;
    }
  }
  else
  {
    // Convert Host_ID by SE_ID
    //   UICC (Host_ID : 0x02, SE_ID : 0x03)
    //   eSE (Host_ID : 0x03, SE_ID : 0x02)
    if(gSEId == 0x03)
      destHost = 0x02;
    else
      destHost = 0x03;

    // Get a list of existing gates and pipes
    {
      NCI_DEBUG ("get gate, pipe list");
      SyncEventGuard guard (mPipeListEvent);
      nfaStat = NFA_HciGetGateAndPipeList (mNfaHciHandle);
      if (nfaStat == NFA_STATUS_OK)
      {
        mPipeListEvent.Wait();
        if (mHciCfg.status == NFA_STATUS_OK)
        {
          for (uint8_t xx = 0; xx < mHciCfg.num_pipes; xx++)
          {
            if ((mHciCfg.pipe[xx].dest_host == destHost)
                && (mHciCfg.pipe[xx].dest_gate == mDestinationGate))
            {
              mNewSourceGate = mHciCfg.pipe[xx].local_gate;
              mNewPipeId     = mHciCfg.pipe[xx].pipe_id;

              NCI_DEBUG ("found configured gate: 0x%02x  pipe: 0x%02x", mNewSourceGate, mNewPipeId);
              break;
            }
          }
        }
      }
    }

    if (mNewSourceGate == 0)
    {
      NCI_DEBUG ("allocate gate");

      //allocate a source gate and store in mNewSourceGate
      SyncEventGuard guard (mAllocateGateEvent);
      if ((nfaStat = NFA_HciAllocGate (mNfaHciHandle, 0)) != NFA_STATUS_OK)
      {
        NCI_ERROR ("fail allocate source gate; error=0x%X", nfaStat);
        goto TheEnd;
      }
      mAllocateGateEvent.Wait ();
      if (mCommandStatus != NFA_STATUS_OK)
        goto TheEnd;
    }

    if (mNewPipeId == 0)
    {
      NCI_DEBUG ("create pipe");
      SyncEventGuard guard (mCreatePipeEvent);
      nfaStat = NFA_HciCreatePipe (mNfaHciHandle, mNewSourceGate, destHost, mDestinationGate);
      if (nfaStat != NFA_STATUS_OK)
      {
        NCI_ERROR ("fail create pipe; error=0x%X", nfaStat);
        goto TheEnd;
      }
      mCreatePipeEvent.Wait ();
      if (mCommandStatus != NFA_STATUS_OK)
        goto TheEnd;
    }

    {
      NCI_DEBUG ("open pipe");
      SyncEventGuard guard (mPipeOpenedEvent);
      nfaStat = NFA_HciOpenPipe (mNfaHciHandle, mNewPipeId);
      if (nfaStat != NFA_STATUS_OK)
      {
        NCI_ERROR ("fail open pipe; error=0x%X", nfaStat);
        goto TheEnd;
      }
      mPipeOpenedEvent.Wait ();
      if (mCommandStatus != NFA_STATUS_OK)
        goto TheEnd;
    }
  }
  retVal = true;

TheEnd:
  mIsPiping = retVal;
  if (!retVal)
  {
    // if open failed we need to de-allocate the gate
    DisconnectEE(0);
  }

  NCI_DEBUG ("exit; ok=%u", retVal);
  return retVal;
}

/*******************************************************************************
**
** Function:        DisconnectEE
**
** Description:     Disconnect from the execution environment.
**                  seID: ID of secure element.
**
** Returns:         True if ok.
**
*******************************************************************************/
bool SecureElement::DisconnectEE (int seID)
{
    static const char fn [] = "SecureElement::DisconnectEE";
    tNFA_STATUS nfaStat = NFA_STATUS_FAILED;
    tNFA_HANDLE eeHandle = seID;

    NCI_DEBUG("seID=0x%X; handle=0x%04x", seID, eeHandle);

    if (mUseOberthurWarmReset)
    {
        // send warm-reset command to Oberthur secure element which deselects the applet;
        // this is an Oberthur-specific command;
        NCI_DEBUG("try warm-reset on pipe id 0x%X; cmd=0x%X", mNewPipeId, mOberthurWarmResetCommand);
        SyncEventGuard guard (mRegistryEvent);
        nfaStat = NFA_HciSetRegistry (mNfaHciHandle, mNewPipeId,
                1, 1, &mOberthurWarmResetCommand);
        if (nfaStat == NFA_STATUS_OK)
        {
            mRegistryEvent.Wait ();
            NCI_DEBUG("completed warm-reset on pipe 0x%X", mNewPipeId);
        }
    }

/* START [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
    if ((gEseChipType == 0x01 ) && (gUseStaticPipe != true)) // Gemalto eSE
    {
        NCI_DEBUG("NFA_HCI_EVT_EVT_SOFT_RESET");

        SyncEventGuard guard (mTransceiveEvent);
        nfaStat = NFA_HciSendEvent (mNfaHciHandle, mNewPipeId, EVT_SOFT_RESET, 0x00, NULL, 00, NULL, 100);

        if (nfaStat == NFA_STATUS_OK)
        {
            NCI_DEBUG("wait to terminate NFA_HCI_EVT_EVT_SOFT_RESET");
            mTransceiveEvent.Wait (100);
        }
    }
/* END [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/

    mIsPiping = false;

    // Re-enable RF discovery
    // Note that it only effactuates the current configuration,
    // so if polling/listening were configured OFF (forex because
    // the screen was off), they will stay OFF with this call.
/* START [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
    if(gIsSupportConcurrentWCE != true)
    {
      // Disable RF discovery completely while the DH is connected
      StartRfDiscovery(true);
    }
/* END [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/

    return true;
}


/*******************************************************************************
**
** Function:        transceive
**
** Description:     Send data to the secure element; read it's response.
**                  xmitBuffer: Data to transmit.
**                  xmitBufferSize: Length of data.
**                  recvBuffer: Buffer to receive response.
**                  recvBufferMaxSize: Maximum size of buffer.
**                  recvBufferActualSize: Actual length of response.
**                  timeoutMillisec: timeout in millisecond.
**
** Returns:         True if ok.
**
*******************************************************************************/
bool SecureElement::transceive (uint8_t* xmitBuffer, INT32 xmitBufferSize, uint8_t* recvBuffer,
        INT32 recvBufferMaxSize, INT32& recvBufferActualSize, INT32 timeoutMillisec)
{
    static const char fn [] = "SecureElement::transceive";
    tNFA_STATUS nfaStat = NFA_STATUS_FAILED;
    bool isSuccess = false;
    bool waitOk = false;
    uint8_t newSelectCmd[NCI_MAX_AID_LEN + 10];


    NCI_DEBUG ("enter; xmitBufferSize=%ld; recvBufferMaxSize=%ld; timeout=%ld",
        xmitBufferSize, recvBufferMaxSize, timeoutMillisec);

    // Check if we need to replace an "empty" SELECT command.
    // 1. Has there been a AID configured, and
    // 2. Is that AID a valid length (i.e 16 bytes max), and
    // 3. Is the APDU at least 4 bytes (for header), and
    // 4. Is INS == 0xA4 (SELECT command), and
    // 5. Is P1 == 0x04 (SELECT by AID), and
    // 6. Is the APDU len 4 or 5 bytes.
    //
    // Note, the length of the configured AID is in the first
    //   byte, and AID starts from the 2nd byte.
    if (mAidForEmptySelect[0]                           // 1
        && (mAidForEmptySelect[0] <= NCI_MAX_AID_LEN)   // 2
        && (xmitBufferSize >= 4)                        // 3
        && (xmitBuffer[1] == 0xA4)                      // 4
        && (xmitBuffer[2] == 0x04)                      // 5
        && (xmitBufferSize <= 5))                       // 6
    {
        UINT8 idx = 0;

        // Copy APDU command header from the input buffer.
        memcpy(&newSelectCmd[0], &xmitBuffer[0], 4);
        idx = 4;

        // Set the Lc value to length of the new AID
        newSelectCmd[idx++] = mAidForEmptySelect[0];

        // Copy the AID
        memcpy(&newSelectCmd[idx], &mAidForEmptySelect[1], mAidForEmptySelect[0]);
        idx += mAidForEmptySelect[0];

        // If there is an Le (5th byte of APDU), add it to the end.
        if (xmitBufferSize == 5)
            newSelectCmd[idx++] = xmitBuffer[4];

        // Point to the new APDU
        xmitBuffer = &newSelectCmd[0];
        xmitBufferSize = idx;

        NCI_DEBUG ("Empty AID SELECT cmd detected, substituting AID from config file, new length=%d", idx);
    }

    {
        SyncEventGuard guard (mTransceiveEvent);
        mActualResponseSize = 0;
        memset (mResponseData, 0, sizeof(mResponseData));

/* START [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
        if ((mNewPipeId == STATIC_PIPE_0x72) ||
             (mNewSourceGate ==  APDU_GATE_0xF0 && gUseStaticPipe != 0x01))
        {
            nfaStat = NFA_HciSendEvent(mNfaHciHandle, mNewPipeId,
                EVT_SEND_DATA, xmitBufferSize, xmitBuffer,
                sizeof(mResponseData), mResponseData, timeoutMillisec);
        }
        else
        {
            nfaStat = NFA_HciSendEvent (mNfaHciHandle, mNewPipeId,
                NFA_HCI_EVT_POST_DATA, xmitBufferSize, xmitBuffer,
                sizeof(mResponseData), mResponseData, timeoutMillisec);
        }
/* END [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/

        if (nfaStat == NFA_STATUS_OK)
        {
            waitOk = mTransceiveEvent.Wait (timeoutMillisec);
            if (waitOk == false) //timeout occurs
            {
                NCI_ERROR("wait response timeout");
                goto TheEnd;
            }
        }
        else
        {
            NCI_ERROR("fail send data; error=0x%X", nfaStat);
            goto TheEnd;
        }
    }

    if (mActualResponseSize > recvBufferMaxSize)
        recvBufferActualSize = recvBufferMaxSize;
    else
        recvBufferActualSize = mActualResponseSize;

    memcpy (recvBuffer, mResponseData, recvBufferActualSize);
    isSuccess = true;

TheEnd:
    NCI_DEBUG ("exit; isSuccess: %d; recvBufferActualSize: %ld", isSuccess, recvBufferActualSize);
    return (isSuccess);
}


void SecureElement::ResetRfFieldStatus()
{
  NCI_DEBUG("enter;");

  mMutex.Lock();
  mRfFieldIsOn = false;
  int ret = clock_gettime(CLOCK_MONOTONIC, &mLastRfFieldToggle);
  if (ret == -1)
  {
    NCI_ERROR("clock_gettime failed");
    // There is no good choice here...
  }
  mMutex.Unlock();
}

void SecureElement::StoreUiccInfo(tNFA_EE_DISCOVER_REQ& aInfo)
{
  NCI_DEBUG("Status: %u   Num EE: %u", aInfo.status, aInfo.num_ee);

  SyncEventGuard guard(mUiccInfoEvent);
  memcpy(&mUiccInfo, &aInfo, sizeof(mUiccInfo));
  for (uint8_t i = 0; i < aInfo.num_ee; i++)
  {
    //for each technology (A, B, F, B'), print the bit field that shows
    //what protocol(s) is support by that technology
    NCI_DEBUG("EE[%u] Handle: 0x%04x  techA: 0x%02x  techB: 0x%02x  techF: 0x%02x  techBprime: 0x%02x",
              i,
              aInfo.ee_disc_info[i].ee_handle,
              aInfo.ee_disc_info[i].la_protocol,
              aInfo.ee_disc_info[i].lb_protocol,
              aInfo.ee_disc_info[i].lf_protocol,
              aInfo.ee_disc_info[i].lbp_protocol);
  }
  mUiccInfoEvent.NotifyOne();
}

/*******************************************************************************
**
** Function         GetAtrInfo
**
** Description      Gets version information and id for a secure element.  The
**                  seIndex parmeter is the zero based index of the secure
**                  element to get verion info for.  The version infommation
**                  is returned as a string int the verInfo parameter.
**
** Returns          ture on success, false on failure
**
*******************************************************************************/
bool SecureElement::GetAtrInfo(uint8_t* recvBuffer, int& rspLen)
{
    NCI_DEBUG("%s: enter, ", __FUNCTION__);

    uint8_t pipe = 0x72;
    uint8_t host = 0x01;
    uint8_t gate = 0xF0;

    memset (mResponseData, 0, sizeof(mResponseData));
    mActualResponseSize = 0;

    tNFA_STATUS nfaStat = NFA_HciAddStaticPipe(mNfaHciHandle, host, gate, pipe);
    if (nfaStat != NFA_STATUS_OK)
    {
        NCI_ERROR("%s: NFA_HciAddStaticPipe() failed, pipe = 0x%x, error=0x%X", __FUNCTION__, pipe, nfaStat);
        return true;
    }

    SyncEventGuard guard (mVerInfoEvent);
    if (NFA_STATUS_OK == (nfaStat = NFA_HciGetRegistry (mNfaHciHandle, pipe, 0x02)))
    {
        if (false == mVerInfoEvent.Wait(200))
        {
             NCI_ERROR("%s: wait response timeout", __FUNCTION__);
        }
        else
        {
            rspLen = mActualResponseSize;
            memcpy (recvBuffer, mResponseData, rspLen);
        }
    }
    else
    {
        NCI_ERROR("%s: NFA_HciGetRegistry () failed: 0x%X", __FUNCTION__, nfaStat);
    }
    return true;
}

void SecureElement::AdjustRoutes(RouteSelection aSelection)
{
  NCI_DEBUG("enter; selection=%u", aSelection);

  mCurrentRouteSelection = aSelection;
  AdjustProtocolRoutes(aSelection);
  AdjustTechnologyRoutes(aSelection);

  NFA_EeUpdateNow(); //apply new routes now.
}

void SecureElement::AdjustProtocolRoutes(RouteSelection aRouteSelection)
{
  NCI_DEBUG("enter");

  tNFA_STATUS nfaStat = NFA_STATUS_FAILED;
  const tNFA_PROTOCOL_MASK protoMask = NFA_PROTOCOL_MASK_ISO_DEP;

  for (int i = 0; i < 3; i++)
    mSeProtoMaskForSubScreen[i] = 0;

  /* delete route to host */
  {
    NCI_DEBUG("delete route to host");
    SyncEventGuard guard(mRoutingEvent);
    nfaStat = NFA_EeSetDefaultProtoRouting(NFA_EE_HANDLE_DH, 0, 0, 0, mSeProtoMaskForSubScreen);
    if (nfaStat == NFA_STATUS_OK)
    {
      mRoutingEvent.Wait();
    }
    else
    {
      NCI_ERROR("fail delete route to host; error=0x%X", nfaStat);
    }
  }

  /**
   * delete route to every sec elem
   */
  for (int i = 0; i < mActualNumEe; i++)
  {
    if ((mEeInfo[i].num_interface != 0) &&
        (mEeInfo[i].ee_interface[0] != NFC_NFCEE_INTERFACE_HCI_ACCESS) &&
        (mEeInfo[i].ee_status == NFA_EE_STATUS_ACTIVE))
    {
      NCI_DEBUG("delete route to EE h=0x%X", mEeInfo[i].ee_handle);
      SyncEventGuard guard(mRoutingEvent);
      nfaStat = NFA_EeSetDefaultProtoRouting(mEeInfo[i].ee_handle, 0, 0, 0,mSeProtoMaskForSubScreen);
      if (nfaStat == NFA_STATUS_OK)
      {
        mRoutingEvent.Wait();
      }
      else
      {
        NCI_ERROR("fail delete route to EE; error=0x%X", nfaStat);
      }
    }
  }
  NCI_DEBUG(" set protocol routing");
  for(int i = 0; i < 3; i++)
    mSeProtoMaskForSubScreen[i] = NFA_PROTOCOL_MASK_ISO_DEP;

  /* if route database is empty, setup a default route */
  if (true) // SLSI_TODO
  {
    tNFA_HANDLE eeHandle =
      (aRouteSelection == SecElemRoute) ? mActiveEeHandle : NFA_EE_HANDLE_DH;

    NCI_DEBUG("route to default EE h=0x%X", eeHandle);
    SyncEventGuard guard(mRoutingEvent);
    nfaStat = NFA_EeSetDefaultProtoRouting(eeHandle, protoMask, protoMask, protoMask, mSeProtoMaskForSubScreen);
    if (nfaStat == NFA_STATUS_OK)
    {
      mRoutingEvent.Wait();
    }
    else
    {
      NCI_ERROR("fail route to EE; error=0x%X", nfaStat);
    }
  }
  NCI_DEBUG("exit");
}

void SecureElement::AdjustTechnologyRoutes(RouteSelection aRouteSelection)
{
  NCI_DEBUG("enter");

  tNFA_STATUS nfaStat = NFA_STATUS_FAILED;
  const tNFA_TECHNOLOGY_MASK techMask = NFA_TECHNOLOGY_MASK_A | NFA_TECHNOLOGY_MASK_B;

  for(int i = 0; i < 3; i++)
    mSeTechMaskForSubScreen[i] = 0;

   /* delete route to host */
  {
    NCI_DEBUG("delete route to host");
    SyncEventGuard guard(mRoutingEvent);
    nfaStat = NFA_EeSetDefaultTechRouting(NFA_EE_HANDLE_DH, 0, 0, 0, mSeTechMaskForSubScreen);
    if (nfaStat  == NFA_STATUS_OK)
    {
      mRoutingEvent.Wait();
    }
    else
    {
      NCI_ERROR("fail delete route to host; error=0x%X", nfaStat);
    }
  }

  /**
   * delete route to every sec elem.
   */
  for (int i = 0; i < mActualNumEe; i++)
  {
    if ((mEeInfo[i].num_interface != 0) &&
        (mEeInfo[i].ee_interface[0] != NFC_NFCEE_INTERFACE_HCI_ACCESS) &&
        (mEeInfo[i].ee_status == NFA_EE_STATUS_ACTIVE))
    {
      NCI_DEBUG("delete route to EE h=0x%X", mEeInfo[i].ee_handle);
      SyncEventGuard guard(mRoutingEvent);
      nfaStat = NFA_EeSetDefaultTechRouting(mEeInfo[i].ee_handle, 0, 0, 0, mSeTechMaskForSubScreen);
      if (nfaStat == NFA_STATUS_OK)
      {
        mRoutingEvent.Wait();
      }
      else
      {
        NCI_ERROR("fail delete route to EE; error=0x%X", nfaStat);
      }
    }
  }

   /* if route database is empty, setup a default route */
  for(int i = 0; i < 3; i++)
    mSeTechMaskForSubScreen[i] = techMask;

  if (true) // SLSI_TODO
  {
    tNFA_HANDLE eeHandle =
      (aRouteSelection == SecElemRoute) ? mActiveEeHandle : NFA_EE_HANDLE_DH;

    NCI_DEBUG("route to default EE h=0x%X", eeHandle);
    SyncEventGuard guard(mRoutingEvent);
    nfaStat = NFA_EeSetDefaultTechRouting(eeHandle, techMask, techMask, techMask, mSeTechMaskForSubScreen);
    if (nfaStat == NFA_STATUS_OK)
    {
      mRoutingEvent.Wait();
    }
    else
    {
      NCI_ERROR("fail route to EE; error=0x%X", nfaStat);
    }
  }
}

void SecureElement::NfaEeCallback(tNFA_EE_EVT aEvent,
                                  tNFA_EE_CBACK_DATA* aEventData)
{
  NCI_DEBUG("event=0x%X", aEvent);
  switch (aEvent)
  {
    case NFA_EE_REGISTER_EVT:
    {
      SyncEventGuard guard (sSecElem.mEeRegisterEvent);
      NCI_DEBUG("NFA_EE_REGISTER_EVT; status=%u", aEventData->ee_register);
      sSecElem.mEeRegisterEvent.NotifyOne();
      break;
    }
    case NFA_EE_MODE_SET_EVT:
    {
      NCI_DEBUG("NFA_EE_MODE_SET_EVT; status: 0x%04X  handle: 0x%04X  mActiveEeHandle: 0x%04X",
                aEventData->mode_set.status,
                aEventData->mode_set.ee_handle,
                sSecElem.mActiveEeHandle);

      if (aEventData->mode_set.status == NFA_STATUS_OK)
      {
        tNFA_EE_INFO* pEE = sSecElem.FindEeByHandle (aEventData->mode_set.ee_handle);
        if (pEE)
        {
          pEE->ee_status ^= 1;
          NCI_DEBUG("NFA_EE_MODE_SET_EVT; pEE->ee_status: %s (0x%04x)",
                    SecureElement::EeStatusToString(pEE->ee_status), pEE->ee_status);
        }
        else
        {
          NCI_ERROR("NFA_EE_MODE_SET_EVT; EE: 0x%04x not found.  mActiveEeHandle: 0x%04x",
                    aEventData->mode_set.ee_handle, sSecElem.mActiveEeHandle);
        }
      }
      SyncEventGuard guard(sSecElem.mEeSetModeEvent);
      sSecElem.mEeSetModeEvent.NotifyOne();
      break;
    }
    case NFA_EE_SET_TECH_CFG_EVT:
    {
      NCI_DEBUG("NFA_EE_SET_TECH_CFG_EVT; status=0x%X", aEventData->status);
      SyncEventGuard guard(sSecElem.mRoutingEvent);
      sSecElem.mRoutingEvent.NotifyOne();
      break;
    }
    case NFA_EE_SET_PROTO_CFG_EVT:
    {
      NCI_DEBUG("NFA_EE_SET_PROTO_CFG_EVT; status=0x%X", aEventData->status);
      SyncEventGuard guard(sSecElem.mRoutingEvent);
      sSecElem.mRoutingEvent.NotifyOne();
      break;
    }
    case NFA_EE_DISCOVER_REQ_EVT:
      NCI_DEBUG("NFA_EE_DISCOVER_REQ_EVT; status=0x%X; num ee=%u",
                aEventData->discover_req.status,
                aEventData->discover_req.num_ee);
      sSecElem.StoreUiccInfo(aEventData->discover_req);
      break;
    case NFA_EE_ADD_AID_EVT:
    {
      NCI_DEBUG("NFA_EE_ADD_AID_EVT  status=%u", aEventData->status);
      SyncEventGuard guard(sSecElem.mAidAddRemoveEvent);
      sSecElem.mAidAddRemoveEvent.NotifyOne();
      break;
    }
    case NFA_EE_REMOVE_AID_EVT:
    {
      NCI_DEBUG("NFA_EE_REMOVE_AID_EVT  status=%u", aEventData->status);
      SyncEventGuard guard(sSecElem.mAidAddRemoveEvent);
      sSecElem.mAidAddRemoveEvent.NotifyOne();
      break;
    }
    case NFA_EE_NEW_EE_EVT:
      NCI_DEBUG("NFA_EE_NEW_EE_EVT  h=0x%X; status=%u",
                aEventData->new_ee.ee_handle, aEventData->new_ee.ee_status);
      // Indicate there are new EE
      sSecElem.mbNewEE = true;
      break;
    default:
      NCI_ERROR("unknown event=%u ????", aEvent);
      break;
  }
}

tNFA_EE_INFO* SecureElement::FindEeByHandle(tNFA_HANDLE aEeHandle)
{
  for (uint8_t i = 0; i < mActualNumEe; i++)
  {
    if (mEeInfo[i].ee_handle == aEeHandle)
    {
      return &mEeInfo[i];
    }
  }
  return (NULL);
}

void SecureElement::NfaHciCallback(tNFA_HCI_EVT aEvent,
                                   tNFA_HCI_EVT_DATA* aEventData)
{
  NCI_DEBUG("event=0x%X", aEvent);

  switch (aEvent)
  {
    case NFA_HCI_REGISTER_EVT:
    {
      NCI_DEBUG("NFA_HCI_REGISTER_EVT; status=0x%X; handle=0x%X",
                aEventData->hci_register.status,
                aEventData->hci_register.hci_handle);
      SyncEventGuard guard(sSecElem.mHciRegisterEvent);
      sSecElem.mNfaHciHandle = aEventData->hci_register.hci_handle;
      sSecElem.mHciRegisterEvent.NotifyOne();
      break;
    }
    case NFA_HCI_ALLOCATE_GATE_EVT:
    {
      NCI_DEBUG ("NFA_HCI_ALLOCATE_GATE_EVT; status=0x%X; gate=0x%X",
          aEventData->status, aEventData->allocated.gate);
      SyncEventGuard guard (sSecElem.mAllocateGateEvent);
      sSecElem.mCommandStatus = aEventData->status;
      sSecElem.mNewSourceGate = (aEventData->allocated.status == NFA_STATUS_OK)?
                                  aEventData->allocated.gate : 0;
      sSecElem.mAllocateGateEvent.NotifyOne();
      break;
    }
    case NFA_HCI_DEALLOCATE_GATE_EVT:
    {
      tNFA_HCI_DEALLOCATE_GATE& deallocated = aEventData->deallocated;
      NCI_DEBUG ("NFA_HCI_DEALLOCATE_GATE_EVT; status=0x%X; gate=0x%X",
          deallocated.status, deallocated.gate);
      SyncEventGuard guard (sSecElem.mDeallocateGateEvent);
      sSecElem.mDeallocateGateEvent.NotifyOne();
      break;
    }
    case NFA_HCI_GET_GATE_PIPE_LIST_EVT:
    {
      NCI_DEBUG ("NFA_HCI_GET_GATE_PIPE_LIST_EVT; status=0x%X; num_pipes: %u  num_gates: %u",
          aEventData->gates_pipes.status, aEventData->gates_pipes.num_pipes,
          aEventData->gates_pipes.num_gates);
      SyncEventGuard guard (sSecElem.mPipeListEvent);
      sSecElem.mCommandStatus = aEventData->gates_pipes.status;
      sSecElem.mHciCfg = aEventData->gates_pipes;
      sSecElem.mPipeListEvent.NotifyOne();
// [START] System LSI - event source
      sSecElem.notifyTransaction();
// [END] System LSI - event source
      break;
    }
    case NFA_HCI_CREATE_PIPE_EVT:
    {
      NCI_DEBUG ("NFA_HCI_CREATE_PIPE_EVT; status=0x%X; pipe=0x%X; src gate=0x%X; dest host=0x%X; dest gate=0x%X",
          aEventData->created.status, aEventData->created.pipe, aEventData->created.source_gate,
          aEventData->created.dest_host, aEventData->created.dest_gate);
      SyncEventGuard guard (sSecElem.mCreatePipeEvent);
      sSecElem.mCommandStatus = aEventData->created.status;
      sSecElem.mNewPipeId = aEventData->created.pipe;
      sSecElem.mCreatePipeEvent.NotifyOne();
      break;
    }
    case NFA_HCI_OPEN_PIPE_EVT:
    {
      NCI_DEBUG ("NFA_HCI_OPEN_PIPE_EVT; status=0x%X; pipe=0x%X",
          aEventData->opened.status, aEventData->opened.pipe);
      SyncEventGuard guard (sSecElem.mPipeOpenedEvent);
      sSecElem.mCommandStatus = aEventData->opened.status;
      sSecElem.mPipeOpenedEvent.NotifyOne();
      break;
    }
    case NFA_HCI_EVENT_SENT_EVT:
      NCI_DEBUG ("NFA_HCI_EVENT_SENT_EVT; status=0x%X", aEventData->evt_sent.status);
      break;
    case NFA_HCI_RSP_RCVD_EVT: //response received from secure element
    {
      tNFA_HCI_RSP_RCVD& rsp_rcvd = aEventData->rsp_rcvd;
      NCI_DEBUG ("NFA_HCI_RSP_RCVD_EVT; status: 0x%X; code: 0x%X; pipe: 0x%X; len: %u",
          rsp_rcvd.status, rsp_rcvd.rsp_code, rsp_rcvd.pipe, rsp_rcvd.rsp_len);
      break;
    }
    case NFA_HCI_GET_REG_RSP_EVT :
      NCI_DEBUG ("NFA_HCI_GET_REG_RSP_EVT; status: 0x%X; pipe: 0x%X, len: %d",
          aEventData->registry.status, aEventData->registry.pipe, aEventData->registry.data_len);
      sSecElem.mActualResponseSize = aEventData->registry.data_len;
      memcpy(sSecElem.mResponseData, aEventData->registry.reg_data, aEventData->registry.data_len);
      // [START] wired C/E
      if (aEventData->registry.data_len >= 19 && (aEventData->registry.pipe == STATIC_PIPE_0x72))
      // [END] wired C/E
      {
        SyncEventGuard guard (sSecElem.mVerInfoEvent);
        // Oberthur OS version is in bytes 16,17, and 18
        sSecElem.mVerInfo[0] = aEventData->registry.reg_data[16];
        sSecElem.mVerInfo[1] = aEventData->registry.reg_data[17];
        sSecElem.mVerInfo[2] = aEventData->registry.reg_data[18];
        sSecElem.mVerInfoEvent.NotifyOne ();
      }
      break;
    case NFA_HCI_EVENT_RCVD_EVT:
      NCI_DEBUG ("NFA_HCI_EVENT_RCVD_EVT; code: 0x%X; pipe: 0x%X; data len: %u",
          aEventData->rcvd_evt.evt_code, aEventData->rcvd_evt.pipe, aEventData->rcvd_evt.evt_len);

/* START [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
      if ( (aEventData->rcvd_evt.pipe == STATIC_PIPE_0x72) ||
          ((sSecElem.mNewSourceGate == APDU_GATE_0xF0) && (gUseStaticPipe != 0x01)) )
/* END [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
      {
        NCI_DEBUG ("NFA_HCI_EVENT_RCVD_EVT; data from static pipe");
        SyncEventGuard guard (sSecElem.mTransceiveEvent);
        sSecElem.mActualResponseSize = (aEventData->rcvd_evt.evt_len > MAX_RESPONSE_SIZE)?
                                          MAX_RESPONSE_SIZE : aEventData->rcvd_evt.evt_len;
        sSecElem.mTransceiveEvent.NotifyOne ();
      }
      else if (aEventData->rcvd_evt.evt_code == NFA_HCI_EVT_POST_DATA)
      {
        NCI_DEBUG ("NFA_HCI_EVENT_RCVD_EVT; NFA_HCI_EVT_POST_DATA");
        SyncEventGuard guard (sSecElem.mTransceiveEvent);
        sSecElem.mActualResponseSize = (aEventData->rcvd_evt.evt_len > MAX_RESPONSE_SIZE)?
                                          MAX_RESPONSE_SIZE : aEventData->rcvd_evt.evt_len;
        sSecElem.mTransceiveEvent.NotifyOne ();
      }
      else if (aEventData->rcvd_evt.evt_code == NFA_HCI_EVT_TRANSACTION)
      {
        NCI_DEBUG ("NFA_HCI_EVENT_RCVD_EVT; NFA_HCI_EVT_TRANSACTION");
        /* START [17041401J] Adjust device sleep timeout for RaboBank*/
        unsigned long sec = 0;
        if (!GetNumValue("HCI_CE_DEVICE_SLEEP_TIMEOUT", &sec, sizeof(sec)))
          sec = 5;

        slsiSetNfcSleepTimeout(sec, 1 /* SET_SLEEP_TIME_ONCE */);
        /* END [17041401J] Adjust device sleep timeout for RaboBank*/
        // If we got an AID, notify any listeners
        if ((aEventData->rcvd_evt.evt_len > 3) && (aEventData->rcvd_evt.p_evt_buf[0] == 0x81) )
        {
          // [START] System LSI - event source
          if (!sSecElem.requestNotifyTransaction(aEventData->rcvd_evt.pipe,
                aEventData->rcvd_evt.p_evt_buf, aEventData->rcvd_evt.evt_len))
          {
            int aidlen = aEventData->rcvd_evt.p_evt_buf[1];
            UINT8* data = NULL;
            UINT8 datalen = 0;
            if((aEventData->rcvd_evt.evt_len > 2+aidlen) && (aEventData->rcvd_evt.p_evt_buf[2+aidlen] == 0x82))
            {
              if (aEventData->rcvd_evt.p_evt_buf[2+aidlen+1] == 0x81)
              {
                datalen = aEventData->rcvd_evt.p_evt_buf[2+aidlen+2];
                data  = &aEventData->rcvd_evt.p_evt_buf[2+aidlen+3];
              }
              else
              {
                datalen = aEventData->rcvd_evt.p_evt_buf[2+aidlen+1];
                data  = &aEventData->rcvd_evt.p_evt_buf[2+aidlen+2];
              }
            }

            NCI_DEBUG ("request NotifyTransaction() failed.");
            sSecElem.notifyTransactionListenersOfAid (&aEventData->rcvd_evt.p_evt_buf[2],aidlen,data,datalen,0);
          }
          // [END] System LSI - event source
        }
      }
      else if (aEventData->rcvd_evt.evt_code == NFA_HCI_EVT_CONNECTIVITY)
      {
        NCI_DEBUG ("NFA_HCI_EVENT_RCVD_EVT; NFA_HCI_EVT_CONNECTIVITY");
      }
      else
      {
        NCI_DEBUG ("NFA_HCI_EVENT_RCVD_EVT; aEventData->rcvd_evt.evt_code=0x%x, NFA_HCI_EVT_CONNECTIVITY=0x%x",
            aEventData->rcvd_evt.evt_code, NFA_HCI_EVT_CONNECTIVITY);
      }
      break;
    default:
      NCI_ERROR("unknown event code=0x%X ????", aEvent);
      break;
  }
}

tNFA_HANDLE SecureElement::GetDefaultEeHandle()
{
  uint16_t overrideEeHandle = NFA_HANDLE_GROUP_EE | mActiveSeOverride;
  // Find the first EE that is not the HCI Access i/f.
  for (uint8_t i = 0; i < mActualNumEe; i++)
  {
    if (mActiveSeOverride && (overrideEeHandle != mEeInfo[i].ee_handle))
    {
      continue; //skip all the EE's that are ignored
    }
    if ((mEeInfo[i].num_interface != 0) &&
        (mEeInfo[i].ee_interface[0] != NCI_NFCEE_INTERFACE_HCI_ACCESS) &&
        (mEeInfo[i].ee_status != NFC_NFCEE_STATUS_INACTIVE))
    {
      return mEeInfo[i].ee_handle;
    }
  }
  NCI_ERROR("ee handle not found");
  return NFA_HANDLE_INVALID;
}

const char* SecureElement::EeStatusToString(uint8_t aStatus)
{
  switch (aStatus)
  {
    case NFC_NFCEE_STATUS_ACTIVE:
      return "Connected/Active";
    case NFC_NFCEE_STATUS_INACTIVE:
      return "Connected/Inactive";
    case NFC_NFCEE_STATUS_REMOVED:
      return "Removed";
    default:
      return "?? Unknown ??";
  }
}

void SecureElement::ConnectionEventHandler(uint8_t aEvent,
                                           tNFA_CONN_EVT_DATA* /*aaaEventData*/)
{
  switch (aEvent)
  {
    case NFA_CE_UICC_LISTEN_CONFIGURED_EVT:
    {
      SyncEventGuard guard(mUiccListenEvent);
      mUiccListenEvent.NotifyOne();
      break;
    }
  }
}

bool SecureElement::RouteToSecureElement()
{
  NCI_DEBUG("enter");
  tNFA_STATUS nfaStat = NFA_STATUS_FAILED;
  tNFA_TECHNOLOGY_MASK tech_mask = NFA_TECHNOLOGY_MASK_A | NFA_TECHNOLOGY_MASK_B;
  bool retval = false;

  if (!mIsInit)
  {
    NCI_ERROR("not init");
    return false;
  }

  if (mCurrentRouteSelection == SecElemRoute)
  {
    NCI_ERROR("already sec elem route");
    return true;
  }

  if (mActiveEeHandle == NFA_HANDLE_INVALID)
  {
    NCI_ERROR("invalid EE handle");
    return false;
  }

  AdjustRoutes(SecElemRoute);
  {
    unsigned long num = 0;
    if (GetNumValue("UICC_LISTEN_TECH_MASK", &num, sizeof(num)))
    {
      tech_mask = num;
    }

    NCI_DEBUG("start UICC listen; h=0x%X; tech mask=0x%X",
            mActiveEeHandle, tech_mask);
    SyncEventGuard guard(mUiccListenEvent);
    nfaStat = NFA_CeConfigureUiccListenTech(mActiveEeHandle, tech_mask);
    if (nfaStat == NFA_STATUS_OK)
    {
      mUiccListenEvent.Wait();
      retval = true;
    }
    else
    {
      NCI_ERROR("fail to start UICC listen");
    }
  }

  return retval;
}

bool SecureElement::RouteToDefault()
{
  tNFA_STATUS nfaStat = NFA_STATUS_FAILED;
  bool retval = false;

  NCI_DEBUG("enter");
  if (!mIsInit)
  {
    NCI_ERROR("not init");
    return false;
  }

  if (mCurrentRouteSelection == DefaultRoute)
  {
    NCI_DEBUG("already default route");
    return true;
  }

  if (mActiveEeHandle != NFA_HANDLE_INVALID)
  {
    NCI_DEBUG("stop UICC listen; EE h=0x%X", mActiveEeHandle);
    SyncEventGuard guard(mUiccListenEvent);
    nfaStat = NFA_CeConfigureUiccListenTech(mActiveEeHandle, 0);
    if (nfaStat == NFA_STATUS_OK)
    {
      mUiccListenEvent.Wait();
      retval = true;
    }
    else
    {
      NCI_ERROR("fail to stop UICC listen");
    }
  }
  else
  {
    retval = true;
  }

  AdjustRoutes(DefaultRoute);

  NCI_DEBUG("exit; ok=%u", retval);
  return retval;
}

// [START] System LSI - event source
/*******************************************************************************
**
** Function:        findDestByPipe
**
** Description:     Find a destination of the pipe.
**                  pipe: pipe of the EVT_TRANSACTION.
**
** Returns:         Destination of the pipe.
**
*******************************************************************************/
int SecureElement::findDestByPipe (UINT8 pipe)
{
  int dest = -1;
  for (UINT8 xx = 0; xx < sSecElem.mHciCfg.num_pipes; xx++)
  {
    NCI_DEBUG ("SecureElement::findDestByPipe: find event source from mHciCfg: pipe(%d)[%d], recved pipe[%d]", xx, sSecElem.mHciCfg.pipe[xx].pipe_id, pipe);
    if (sSecElem.mHciCfg.pipe[xx].pipe_id == pipe)
    {
      dest = sSecElem.mHciCfg.pipe[xx].dest_host;
      break;
    }
  }

  if (dest == -1)
  {
    for (UINT8 xx = 0; xx < sSecElem.mHciCfg.num_uicc_created_pipes; xx++)
    {
      NCI_DEBUG ("SecureElement::findDestByPipe: find event source from mHciCfg: pipe(%d)[%d], recved pipe[%d]", xx, sSecElem.mHciCfg.uicc_created_pipe[xx].pipe_id, pipe);
      if (sSecElem.mHciCfg.uicc_created_pipe[xx].pipe_id == pipe)
      {
        dest = sSecElem.mHciCfg.uicc_created_pipe[xx].dest_host;
        break;
      }
    }
  }

  switch (dest)
  {
    case NFA_HCI_HOST_CONTROLLER:
      return 0x01;            // HCI network
    case NFA_HCI_DH_HOST:
      return 0x00;
    case NFA_HCI_UICC_HOST:
      return EE_HANDLE_UICC & ~NFA_HANDLE_GROUP_EE;
    case (NFA_HCI_UICC_HOST+1): // eSE
      return EE_HANDLE_ESE & ~NFA_HANDLE_GROUP_EE;
  }
  return -1;
}
// [END] System LSI - event source

bool SecureElement::IsBusy()
{
  bool retval = (mCurrentRouteSelection == SecElemRoute) || mIsPiping;
  NCI_DEBUG("%u", retval);
  return retval;
}

/* START [D18020201] SE ModeSet */
void SecureElement::SetSecureElementModeset(void)
{
  NCI_DEBUG("enter");

  tNFA_STATUS nfaStat = NFA_STATUS_FAILED;
  uint8_t actualNumEe = MAX_NUM_EE;
  tNFA_EE_INFO eeInfo[MAX_NUM_EE];

  memset (&eeInfo, 0, sizeof(eeInfo));

  if ((nfaStat = NFA_EeGetInfo (&actualNumEe, eeInfo)) != NFA_STATUS_OK)
  {
    NCI_ERROR ("fail get info; error=0x%X", nfaStat);
    return;
  }

  if (actualNumEe != 0)
  {
    for (uint8_t xx = 0; xx < actualNumEe; xx++)
    {
      if ((eeInfo[xx].num_interface != 0)
          && (eeInfo[xx].ee_interface[0] != NCI_NFCEE_INTERFACE_HCI_ACCESS)
          && (eeInfo[xx].ee_status == NFA_EE_STATUS_ACTIVE))
      {
        NCI_ERROR ("Handle: 0x%04x Change Status Active to Inactive", eeInfo[xx].ee_handle);
        SyncEventGuard guard (mEeSetModeEvent);
        if ((nfaStat = NFA_EeModeSet (eeInfo[xx].ee_handle, NFA_EE_MD_DEACTIVATE)) == NFA_STATUS_OK)
        {
          mEeSetModeEvent.Wait (); //wait for NFA_EE_MODE_SET_EVT
        }
        else
        {
          NCI_ERROR ("Failed to set EE inactive");
        }
      }
    }
  }
  else
  {
    NCI_DEBUG ("No active EEs found");
  }
}
/* END [D18020201] eSE ModeSet */

/* START [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
bool SecureElement::isSupportedConcurrentWCE ()
{
  NCI_DEBUG ("SecureElement::isSupportedConcurrentWCE: %u", gIsSupportConcurrentWCE);
  return gIsSupportConcurrentWCE;
}
/* END [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
