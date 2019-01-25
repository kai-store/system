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

#pragma once

#include <vector>
#include <string>
#include "SyncEvent.h"
#include "RouteDataSet.h"

extern "C"
{
  #include "nfa_ee_api.h"
  #include "nfa_hci_api.h"
  #include "nfa_hci_defs.h"
  #include "nfa_ce_api.h"
}

class NfcManager;

class SecureElement
{
public:
  tNFA_HANDLE mActiveEeHandle;

  /**
   * Get the SecureElement singleton object.
   *
   * @return SecureElement object.
   */
  static SecureElement& GetInstance();

  /**
   * Initialize all member variables.
   *
   * @param  aNfcManager NFC manager class instance.
   * @return True if ok.
   */
  bool Initialize(NfcManager* aNfcManager);

  /**
   * Release all resources.
   *
   * @return None.
   */
  void Finalize();

  /**
   * Get the list of handles of all execution environments.
   *
   * @return List of handles of all execution environments.
   */
  void GetListOfEeHandles(std::vector<uint32_t>& aListSe);

  /**
   * Turn on the secure element.
   *
   * @return True if ok.
   */
  bool Activate();

  /**
   * Turn off the secure element.
   *
   * @return True if ok.
   */
  bool Deactivate();

  /**
   * Connect to the execution environment.
   *
   * @return True if ok.
   */
  bool ConnectEE();

  /**
   * Disconnect to the execution environment.
   * seID : ID of secure element
   *
   * @return True if ok.
   */

  bool DisconnectEE(int seID);

  /**
   * Send data to the secure element; read it's response.
   * xmitBuffer: Data to transmit.
   * xmitBufferSize: Length of data.
   * recvBuffer: Buffer to receive response.
   * recvBufferMaxSize: Maximum size of buffer.
   * recvBufferActualSize: Actual length of response.
   * timeoutMillisec: timeout in millisecond
   *
   * @return True if ok.
   */
  bool transceive (uint8_t* xmitBuffer, INT32 xmitBufferSize, uint8_t* recvBuffer,
                     INT32 recvBufferMaxSize, INT32& recvBufferActualSize, INT32 timeoutMillisec);

  /**
   * Resets the RF field status.
   *
   * @return None.
   */
  void ResetRfFieldStatus();

  /**
   * Store a copy of the execution environment information from the stack.
   *
   * @param  info execution environment information.
   * @return None.
   */
  void StoreUiccInfo(tNFA_EE_DISCOVER_REQ& info);

  /**
   * Notify the NFC service about whether the SE was activated
   * in listen mode
   *
   * @return None.
   */
  void NotifyListenModeState(bool isActivated);

  /**
   * Notify the NFC service about RF field events from the stack.
   *
   * @return None.
   */
  void NotifyRfFieldEvent(bool isActive);

  /**
   * Notify the NFC service about a transaction event from secure element.
   *
   * @param  aAid Buffer contains AID.
   * @param  aAidLen Length of AID.
   * @param  aPayload Buffer contains payload.
   * @param  aPayloadLen Length of payload.
   * @return None.
   */
  void NotifyTransactionEvent(const uint8_t* aAid,
                              uint32_t aAidLen,
                              const uint8_t* aPayload,
                              uint32_t aPayloadLen);
/* START [D18020702] - Notify application EVT_Transaction */
  /**
   * Notify the NFC service about a transaction event from secure element
   *
   * @param  aid.
   * @param  aidLen.
   * @param  data.
   * @param  dataLen.
   * @param  evtSrc.
   * @return None.
   */
  void notifyTransactionListenersOfAid (const UINT8* aid, UINT8 aidLen, const UINT8* data,
                                        UINT8 dataLen, UINT32 evtSrc);
/* END [D18020702] - Notify application EVT_Transaction */

  /**
   * Receive card-emulation related events from stack.
   *
   * @param  aEvent Event code.
   * @param  aEventData Event data.
   * @return None.
   */
  void ConnectionEventHandler(uint8_t aEvent,
                              tNFA_CONN_EVT_DATA* aEventData);

  /**
   * Specify which secure element to turn on.
   *
   * @param  aActiveSeOverride ID of secure element.
   * @return None.
   */
  void SetActiveSeOverride(uint8_t aActiveSeOverride);

  /**
   * Adjust controller's listen-mode routing table so transactions
   * are routed to the secure elements as specified in route.xml.
   *
   * @return True if ok.
   */

  /**
   * [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)
   *
   * @return True if ok.
   */
  bool isSupportedConcurrentWCE ();


  bool RouteToSecureElement();

  // TODO : route.xml ???
  /**
   * Adjust controller's listen-mode routing table so transactions
   * are routed to the default destination specified in route.xml.
   *
   * @return True if ok.
   */
  bool RouteToDefault();

  /**
   * Whether NFC controller is routing listen-mode events or a pipe is connected.
   *
   * @return True if either case is true.
   */
  bool IsBusy();

  /**
   * Can be used to determine if the SE is activated in listen mode.
   *
   * @return True if the SE is activated in listen mode.
   */
  bool IsActivatedInListenMode();

  /**
   * [J14111101_Part3] - pending enable discovery during listen mode
   * Can be used to determine if the SE is activated in listen mode
   *
   * @return True if the SE is activated in listen mode
   */
  void SetIsPeerInListenMode(bool isActivated);

  /**
   * [J14111101_Part3] - pending enable discovery during listen mode
   *
   * @return
   */
  bool IsPeerInListenMode();

  /**
   * Can be used to determine if the SE is activated in an RF field.
   *
   * @return True if the SE is activated in an RF field.
   */
  bool IsRfFieldOn();

  /**
   * Get version of information.
   *
   * @return True if at least 1 EE is available.
   */
  bool GetAtrInfo(uint8_t* recvBuffer, int& rspLen);

  /* START [D18020201] SE ModeSet */
  void SetSecureElementModeset();
  /* END [D18020201] SE ModeSet */

private:
  static const unsigned int MAX_RESPONSE_SIZE = 1024;
  enum RouteSelection {NoRoute, DefaultRoute, SecElemRoute};
  static const int MAX_NUM_EE = 5;  // max number of EE's

  //see specification ETSI TS 102 622 v9.0.0 (Host Controller Interface); section 9.3.3.3
  static const uint8_t EVT_SEND_DATA = 0x10;

/* START [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
  static const uint8_t EVT_SOFT_RESET = 0x11;    //see specification GlobalPlatform ver 0.5
  static const uint8_t EVT_END_OF_APDU_TRANSFER = 0x12;    //see specification GlobalPlatform ver 0.5
/* END [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/

// [START] SE ID
//    static const tNFA_HANDLE EE_HANDLE_0xF3 = 0x4F3; //handle to secure element in slot 0
//    static const tNFA_HANDLE EE_HANDLE_0xF4 = 0x4F4; //handle to secure element in slot 1

/* START [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
    static const uint8_t APDU_PIPE_0x15 = 0x15;
    static const uint8_t APDU_GATE_0xF0 = 0xF0;
/* END [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/

	static const uint8_t STATIC_PIPE_0x72 = 0x72;
  static const tNFA_HANDLE EE_HANDLE_ESE  = 0x402; //handle to secure element in slot 0
  static const tNFA_HANDLE EE_HANDLE_UICC = 0x403; //handle to secure element in slot 1
// [END]
  // TODO: 0x01 & 0x02 is for Flame, we should use a more general way to specify this.
  //static const tNFA_HANDLE EE_HANDLE_0x01 = 0x401;  //handle to secure element in slot 0
  //static const tNFA_HANDLE EE_HANDLE_0x02 = 0x402;  //handle to secure element in slot 1
  static SecureElement sSecElem;
  static const char* APP_NAME;

  NfcManager* mNfcManager;
  uint8_t           mDestinationGate;       //destination gate of the UICC
  tNFA_HANDLE mNfaHciHandle;  //NFA handle to NFA's HCI component
  bool mIsInit;  // whether EE is initialized
  uint8_t mActualNumEe;  // actual number of EE's reported by the stack
  uint8_t mNumEePresent;  // actual number of usable EE's
  bool mbNewEE;
  uint8_t   mNewPipeId;
  uint8_t   mNewSourceGate;
  uint16_t mActiveSeOverride;  // active "enable" seid, 0 means activate all SEs
  tNFA_STATUS mCommandStatus;     //completion status of the last command
  bool mIsPiping;  //is a pipe connected to the controller?
  RouteSelection mCurrentRouteSelection;
  int mActualResponseSize;  //number of bytes in the response received from secure element
  bool    mUseOberthurWarmReset;  //whether to use warm-reset command
  bool mActivatedInListenMode; // whether we're activated in listen mode
/* START [J14111101_Part3] - pending enable discovery during listen mode */
  bool    mPeerFieldInListenMode; 	// whether we're activated in listen mode
/* END [J14111101_Part3] - pending enable discovery during listen mode */
  uint8_t   mOberthurWarmResetCommand; //warm-reset command byte
  tNFA_EE_INFO mEeInfo[MAX_NUM_EE];  //actual size stored in mActualNumEe
  tNFA_EE_DISCOVER_REQ mUiccInfo;
  tNFA_HCI_GET_GATE_PIPE_LIST mHciCfg;
  SyncEvent mEeRegisterEvent;
/* START [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
  SyncEvent       mSecVsCmdEvent;
/* END [160503001J] Configure HCI setting for wired C/E by eSE chip type (Oberthure or Gemalto)*/
  SyncEvent mHciRegisterEvent;
  SyncEvent mEeSetModeEvent;
  SyncEvent       mPipeListEvent;
  SyncEvent       mCreatePipeEvent;
  SyncEvent       mPipeOpenedEvent;
  SyncEvent       mAllocateGateEvent;
  SyncEvent mDeallocateGateEvent;
  SyncEvent mRoutingEvent;
  SyncEvent mUiccInfoEvent;
  SyncEvent mUiccListenEvent;
  SyncEvent mAidAddRemoveEvent;
  SyncEvent       mTransceiveEvent;
  SyncEvent       mVerInfoEvent;
  SyncEvent       mRegistryEvent;
  uint8_t           mVerInfo [3];
  uint8_t           mResponseData [MAX_RESPONSE_SIZE];
  RouteDataSet mRouteDataSet; //routing data
  uint8_t           mAidForEmptySelect[NCI_MAX_AID_LEN+1];
  Mutex mMutex;  // protects fields below
  bool mRfFieldIsOn;  // last known RF field state

  tNFA_PROTOCOL_MASK	mSeProtoMaskForSubScreen[3];
  tNFA_TECHNOLOGY_MASK	mSeTechMaskForSubScreen[3];

  struct timespec mLastRfFieldToggle;  // last time RF field went off

  SecureElement();
  ~SecureElement();

  /**
   * Receive execution environment-related events from stack.
   *
   * @param aEvent Event code.
   * @param aEventData Event data.
   * return None.
   */
  static void NfaEeCallback(tNFA_EE_EVT aEvent,
                            tNFA_EE_CBACK_DATA* aEventData);

  /**
   * Receive Host Controller Interface-related events from stack.
   *
   * @param aEvent Event code.
   * @param aEventData Event data.
   * return None.
   */
  static void NfaHciCallback(tNFA_HCI_EVT aEvent,
                             tNFA_HCI_EVT_DATA* aEventData);

  /**
   * Find information about an execution environment.
   *
   * @param  aEeHandle Handle to execution environment.
   * @return Information about an execution environment.
   */
  tNFA_EE_INFO* FindEeByHandle(tNFA_HANDLE aEeHandle);

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
  int findDestByPipe (UINT8 pipe);

  void notifyTransaction(void);
  bool requestNotifyTransaction(UINT8 pipe, UINT8 *data, int len);
// [END] System LSI - event source

  /**
   * Get the handle to the execution environment.
   *
   * @return Handle to the execution environment.
   */
  tNFA_HANDLE GetDefaultEeHandle();

  /**
   * Adjust routes in the controller's listen-mode routing table.
   *
   * @param  aSelection which set of routes to configure the controller.
   * @return None.
   */
  void AdjustRoutes(RouteSelection aSelection);

  /**
   * Adjust default routing based on protocol in NFC listen mode.
   *
   * @param  isRouteToEe Whether routing to EE (true) or host (false).
   * @return None.
   */
  void AdjustProtocolRoutes(RouteSelection aRouteSelection);

  /**
   * Adjust default routing based on technology in NFC listen mode.
   *
   * @param  isRouteToEe Whether routing to EE (true) or host (false).
   * @return None.
   */
  void AdjustTechnologyRoutes(RouteSelection aRouteSelection);

  /**
   * Get latest information about execution environments from stack.
   *
   * @return True if at least 1 EE is available.
   */
  bool GetEeInfo();

  /**
   * Convert status code to status text.
   *
   * @param  status Status code
   * @return None
   */
  static const char* EeStatusToString(uint8_t aStatus);
};
