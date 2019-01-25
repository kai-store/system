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

#include "NfcManager.h"

#include "OverrideLog.h"
#include "config.h"
#include "NfcAdaptation.h"
#include "NfcDebug.h"
#include "SyncEvent.h"
#include "SecureElement.h"
#include "PeerToPeer.h"
#include "PowerSwitch.h"
#include "NfcTag.h"
#include "Pn544Interop.h"
#include "LlcpSocket.h"
#include "LlcpServiceSocket.h"
#include "NfcTagManager.h"
#include "P2pDevice.h"

extern "C"
{
  #include "nfa_p2p_api.h"
  #include "rw_api.h"
  #include "nfa_ee_api.h"
  #include "nfc_brcm_defs.h"
  #include "ce_api.h"
}

/*****************************************************************************
 ** Extern variables and functions
 *****************************************************************************/
// [J14111103] - rf_info after P2P
#define SLSI_PATCHFLAG_WAIT_ENABLE_DISCOVERY    0x0002
#define SLSI_PATCHFLAG_WAIT_DISABLE_DISCOVERY   0x0004
#define SLSI_WRFLAG_SKIP_RF_NTF                 0x0010

// [J16120501] - eSE Stable time
#define ESE_POWER_STABLE_TIME                   50

extern bool gIsTagDeactivating;
extern bool gIsSelectingRfInterface;

extern void enableDiscoveryAfterDeactivation (tNFA_TECHNOLOGY_MASK tech_mask);
extern void disableDiscoveryAfterDeactivation (void);

// S.LSI Additional Function
extern void slsiEventHandler (UINT8 event, tNFA_CONN_EVT_DATA* data);
extern void slsiDeinitialize();
extern void slsiInitialize();
extern void slsiSetFlag(UINT16 flag);
extern bool slsiIsFlag(UINT16 flag);
extern void slsiClearFlag(UINT16 flag);

// [J14120201] - Generic ESE ID
extern int slsiGetSeId(int genSeId);

// [P150309001] - mPOS
extern bool SLSI_mPOSDedicatedMode(bool enable);
/* START [D18020701] - mPOS and EMV PCD test */
extern bool SLSI_mPOSConfigforTest(int type);
/* END [D18020701] - mPOS and EMV PCD test */

// [J15091601] - GCF Reader Test
extern bool SLSI_GcfReaderTestMode(bool enable);

// [J14111303] - Screen State
extern void doSetScreenOrPowerState (int state);

// [J14111100] - support secure element
extern void doSelectSecureElement(int seId);
extern int getSecureElementTechList();
extern void setSecureElementListenTechMask(int tech_mask);
extern void doDeselectSecureElement(int seId);

// [J14121101] - Remaining AID Size
extern int getRemainingAidTableSize();

// [J14121201] - Aid Table Size
extern int getAidTableSize();

// [J15100101] - Get Tech information
extern int getSeSupportedTech();

// [J16101001] - Read RF register project number
extern bool isUsingPollActiveF(void);


/**
 * public variables and functions
 */
nfc_data                gNat;
bool                    gActivated = false;
int                     gGeneralTransceiveTimeout = 1000;
/* START [J17021501] Add Polling or Listening Related Global Variable.*/
/* "gDiscoveryEnabled" variable setting to precede than RF_DEACTIVATED. */
bool    gDiscoveryEnabled = false;
/* END [J17021501] Add Polling or Listening Related Global Variable.*/


void                    DoStartupConfig();
void                    StartRfDiscovery(bool aIsStart);
bool                    StartStopPolling(bool aIsStartPolling);

// [J14111101]
void nfcManager_enableDiscoveryImpl (tNFA_TECHNOLOGY_MASK tech_mask);
void nfcManager_disableDiscoveryImpl (bool screenoffCE);

/**
 * private variables and functions
 */
static SyncEvent            sNfaEnableEvent;                // Event for NFA_Enable().
static SyncEvent            sNfaDisableEvent;               // Event for NFA_Disable().
static SyncEvent            sNfaEnableDisablePollingEvent;  // Event for NFA_EnablePolling(), NFA_DisablePolling().
static SyncEvent            sNfaSetConfigEvent;             // Event for Set_Config....
static SyncEvent            sNfaGetConfigEvent;             // Event for Get_Config....

static bool                 sIsNfaEnabled = false;
static bool                 sDiscoveryEnabled = false;      // Is polling for tag?
static bool                 sPollingEnabled = false;  //is polling for tag?
static bool                 sIsDisabling = false;
static bool                 sRfEnabled = false;             // Whether RF discovery is enabled.
static bool                 sSeRfActive = false;            // Whether RF with SE is likely active.
static bool                 sReaderModeEnabled = false; // whether we're only reading tags, not allowing P2p/card emu
static bool                 sP2pEnabled = false;
static bool                 sP2pActive = false;             // Whether p2p was last active.
static bool                 sIsSecElemSelected = false;     // Has NFC service selected a sec elem.

#define CONFIG_UPDATE_TECH_MASK     (1 << 1)
#define DEFAULT_TECH_MASK           (NFA_TECHNOLOGY_MASK_A \
                                     | NFA_TECHNOLOGY_MASK_B \
                                     | NFA_TECHNOLOGY_MASK_F \
                                     | NFA_TECHNOLOGY_MASK_ISO15693 \
                                     | NFA_TECHNOLOGY_MASK_B_PRIME \
                                     | NFA_TECHNOLOGY_MASK_A_ACTIVE \
                                     | NFA_TECHNOLOGY_MASK_F_ACTIVE \
                                     | NFA_TECHNOLOGY_MASK_KOVIO)
#define DEFAULT_DISCOVERY_DURATION       500
#define READER_MODE_DISCOVERY_DURATION   200

static void NfaConnectionCallback(uint8_t aEvent, tNFA_CONN_EVT_DATA* aEventData);
static void NfaDeviceManagementCallback(uint8_t aEvent, tNFA_DM_CBACK_DATA* aEventData);
static bool IsPeerToPeer(tNFA_ACTIVATED& aActivated);
static bool IsListenMode(tNFA_ACTIVATED& aActivated);
tNFA_STATUS stopPolling_rfDiscoveryDisabled();
tNFA_STATUS startPolling_rfDiscoveryDisabled(tNFA_TECHNOLOGY_MASK tech_mask);

static uint16_t sCurrentConfigLen;
static uint8_t sConfig[256];

NfcManager::NfcManager()
 : mP2pDevice(NULL)
 , mNfcTagManager(NULL)
{
  mP2pDevice = new P2pDevice();
  mNfcTagManager = new NfcTagManager();
}

NfcManager::~NfcManager()
{
  delete mP2pDevice;
  delete mNfcTagManager;
}

/**
 * Interfaces.
 */

void* NfcManager::QueryInterface(const char* aName)
{
  if (0 == strcmp(aName, INTERFACE_P2P_DEVICE)) {
    return reinterpret_cast<void*>(mP2pDevice);
  } else if (0 == strcmp(aName, INTERFACE_TAG_MANAGER)) {
    return reinterpret_cast<void*>(mNfcTagManager);
  }

  return NULL;
}

bool NfcManager::Initialize()
{
  tNFA_STATUS stat = NFA_STATUS_OK;
  unsigned long num = 5;

  // Initialize PowerSwitch.
  PowerSwitch::GetInstance().Initialize(PowerSwitch::FULL_POWER);

  // Start GKI, NCI task, NFC task.
  NfcAdaptation& theInstance = NfcAdaptation::GetInstance();
  theInstance.Initialize();

  {
    SyncEventGuard guard(sNfaEnableEvent);
    tHAL_NFC_ENTRY* halFuncEntries = theInstance.GetHalEntryFuncs();
    NFA_Init(halFuncEntries);

    stat = NFA_Enable(NfaDeviceManagementCallback, NfaConnectionCallback);
    if (stat == NFA_STATUS_OK) {
      num = initializeGlobalAppLogLevel();
      CE_SetTraceLevel(num);
      LLCP_SetTraceLevel(num);
      NFC_SetTraceLevel(num);
      RW_SetTraceLevel(num);
      NFA_SetTraceLevel(num);
      NFA_P2pSetTraceLevel(num);

      sNfaEnableEvent.Wait(); // Wait for NFA command to finish.
    } else {
      NCI_ERROR("NFA_Enable fail, error = 0x%X", stat);
    }
#if 0 // SLSI_MIFARE
    EXTNS_Init(NfaDeviceManagementCallback, NfaConnectionCallback);
#endif
  }

  if (stat == NFA_STATUS_OK) {
    if (sIsNfaEnabled) {
      SecureElement::GetInstance().Initialize(this);
/* START [J14111100] - (REV150731) support secure element */
      SecureElement::GetInstance().Activate();
/* END [J14111100] (REV150731) support secure element */

      NfcTagManager::DoRegisterNdefTypeHandler();

      NfcTag::GetInstance().Initialize(this);

      PeerToPeer::GetInstance().Initialize(this);
      PeerToPeer::GetInstance().HandleNfcOnOff(true);

      // Add extra configuration here (work-arounds, etc.).
      {
        if (GetNumValue(NAME_POLLING_TECH_MASK, &num, sizeof(num)))
          gNat.tech_mask = num;
        else
          gNat.tech_mask = DEFAULT_TECH_MASK;

        NCI_DEBUG("tag polling tech mask = 0x%X", gNat.tech_mask);
      }

      // If this value exists, set polling interval.
      if (GetNumValue(NAME_NFA_DM_DISC_DURATION_POLL, &num, sizeof(num)))
        NFA_SetRfDiscoveryDuration(num);

      // Do custom NFCA startup configuration.
      DoStartupConfig();
      goto TheEnd;
    }
  }

  if (sIsNfaEnabled) {
    stat = Disable(FALSE /* ungraceful */);
  }

  theInstance.Finalize();

TheEnd:
  if (sIsNfaEnabled) {
    PowerSwitch::GetInstance().SetLevel(PowerSwitch::LOW_POWER);
  }

  return sIsNfaEnabled;
}

bool NfcManager::Deinitialize()
{
  NCI_DEBUG("enter");

  sIsDisabling = true;
  Pn544InteropAbortNow();
/* START [J16120501] - eSE Stable time */
  unsigned long delay = ESE_POWER_STABLE_TIME;

  if(sRfEnabled)
    StartRfDiscovery (false);

  GetNumValue(NAME_ESE_POWER_STABLE_TIME, &delay, sizeof(delay));
  NCI_DEBUG("%s: eSE Power stable time : %lu ms", __FUNCTION__, delay);
  GKI_delay(delay);
/* END [J16120501] - eSE Stable time */

/* START [J00000003] - SLSI deintialize */
  slsiDeinitialize();
/* END [J00000003] - SLSI deintialize */

  if (sIsNfaEnabled)
  {
    SyncEventGuard guard(sNfaDisableEvent);

    tNFA_STATUS stat = Disable(TRUE /* graceful */);
    if (stat == NFA_STATUS_OK)
    {
      NCI_DEBUG("wait for completion");
      sNfaDisableEvent.Wait(); // Wait for NFA command to finish.
      PeerToPeer::GetInstance().HandleNfcOnOff(false);
    }
    else
    {
      NCI_ERROR("NFA_Disable fail; error = 0x%X", stat);
    }
  }

  SecureElement::GetInstance().Finalize();
  NfcTagManager::DoAbortWaits();
  NfcTag::GetInstance().Abort();
  // TODO : Implement LLCP.
  sIsNfaEnabled = false;
  sDiscoveryEnabled = false;
  sPollingEnabled = false;
  sIsDisabling = false;
  sIsSecElemSelected = false;
/* START [J14121204] - clear sRfEnabled */
  sRfEnabled = false;
/* END [J14121204] */
/* START [J150511] - clear sP2pEnabled */
  sP2pEnabled = false;
/* END [J150511] - clear sP2pEnabled */

  {
    // Unblock NFA_EnablePolling() and NFA_DisablePolling().
    SyncEventGuard guard(sNfaEnableDisablePollingEvent);
    sNfaEnableDisablePollingEvent.NotifyOne();
  }

  NfcAdaptation& theInstance = NfcAdaptation::GetInstance();
  theInstance.Finalize();

  NCI_DEBUG("exit");
  return true;
}

bool NfcManager::EnableDiscovery()
{
  NCI_DEBUG ("enter");
  tNFA_TECHNOLOGY_MASK tech_mask = DEFAULT_TECH_MASK;

  tech_mask = (tNFA_TECHNOLOGY_MASK)gNat.tech_mask;
  tNFA_STATUS stat = NFA_STATUS_OK;

/* START [J16030201] - Update RF discovery pending flag */
  slsiClearFlag(SLSI_PATCHFLAG_WAIT_DISABLE_DISCOVERY);
/* END [J16030201] - Update RF discovery pending flag */

/* START [J14111101_Part3] - pending enable discovery during listen mode */
  if (SecureElement::GetInstance().IsPeerInListenMode())
  {
    enableDiscoveryAfterDeactivation (tech_mask);
    NCI_ERROR ("%s: pending new RF discovery in RF field", __FUNCTION__);
    goto TheEnd;
  }
/* END [J14111101_Part3] - pending enable discovery during listen mode */
  else if (sDiscoveryEnabled)
  {
    NCI_ERROR ("%s: already discovering", __FUNCTION__);
    goto TheEnd;
  }
  else
  {
/* START [J15033101] - TECH recovery after NXP_P2P workaround */
    enableDiscoveryAfterDeactivation (tech_mask);
/* END [J15033101] - TECH recovery after NXP_P2P workaround */
    slsiClearFlag(SLSI_PATCHFLAG_WAIT_DISABLE_DISCOVERY);
    slsiClearFlag(SLSI_PATCHFLAG_WAIT_ENABLE_DISCOVERY);
    nfcManager_enableDiscoveryImpl (tech_mask);
  }

TheEnd:
  NCI_DEBUG("exit");
  return (stat == NFA_STATUS_OK);
}

void nfcManager_enableDiscoveryImpl (tNFA_TECHNOLOGY_MASK tech_mask)
{
  NCI_DEBUG ("%s: enter; tech mask=0x%X", __FUNCTION__, tech_mask);

  PowerSwitch::GetInstance ().SetLevel (PowerSwitch::FULL_POWER);

  if (sRfEnabled)
  {
    // Stop RF discovery to reconfigure
    StartRfDiscovery(false);
  }

  // Check polling configuration
  if (tech_mask != 0)
  {
    stopPolling_rfDiscoveryDisabled();
    startPolling_rfDiscoveryDisabled(tech_mask);

    sP2pEnabled = true;
    PeerToPeer::GetInstance().EnableP2pListening (true);

  }
  else
  {
    sP2pEnabled = false;
    PeerToPeer::GetInstance().EnableP2pListening (false);
    // No technologies configured, stop polling
    stopPolling_rfDiscoveryDisabled();

    NCI_DEBUG ("%s: Disable p2pListening", __FUNCTION__);
  }

  //if NFC service has deselected the sec elem, then apply default routes.
  if (!sIsSecElemSelected)
  {
    // TODO : Emulator to support SE routing .
    SecureElement::GetInstance().RouteToDefault() ? NFA_STATUS_OK : NFA_STATUS_FAILED;
  }

/* START [J17021501] Add Polling or Listening Related Global Variable */
  gDiscoveryEnabled = true;
/* END [J17021501] Add Polling or Listening Related Global Variable */

  // Actually start discovery.
  StartRfDiscovery (true);
  sDiscoveryEnabled = true;

  PowerSwitch::GetInstance ().SetModeOn (PowerSwitch::DISCOVERY);

  NCI_DEBUG ("%s: exit", __FUNCTION__);
}


bool NfcManager::DisableDiscovery()
{
  tNFA_STATUS status = NFA_STATUS_OK;
  NCI_DEBUG("enter");

  Pn544InteropAbortNow();
/* START [J16030201] - Update RF discovery pending flag */
  slsiClearFlag(SLSI_PATCHFLAG_WAIT_ENABLE_DISCOVERY);
/* END [J16030201] - Update RF discovery pending flag */

/* START [J14111101_Part3] - pending enable discovery during listen mode */
  if (SecureElement::GetInstance().IsPeerInListenMode() && !sP2pActive)
  {
    disableDiscoveryAfterDeactivation ();
    NCI_ERROR("%s: pending new RF discovery in RF field", __FUNCTION__);
    goto TheEnd;
  }
/* END [J14111101_Part3] - pending enable discovery during listen mode */
  else if (!sDiscoveryEnabled)
  {
    NCI_ERROR("%s: already disabled", __FUNCTION__);
    goto TheEnd;
  }
  else
  {
    slsiClearFlag(SLSI_PATCHFLAG_WAIT_DISABLE_DISCOVERY);
    slsiClearFlag(SLSI_PATCHFLAG_WAIT_ENABLE_DISCOVERY);
    nfcManager_disableDiscoveryImpl (true);
  }

TheEnd:
  NCI_DEBUG("exit");
  return (status == NFA_STATUS_OK);
}


void nfcManager_disableDiscoveryImpl (bool screenoffCE)
{
  tNFA_STATUS status = NFA_STATUS_OK;
  NCI_DEBUG ("%s: enter", __FUNCTION__);

/* START [J17021501] Add Polling or Listening Related Global Variable.*/
  gDiscoveryEnabled = false;
/* END [J17021501] Add Polling or Listening Related Global Variable.*/

  // Stop RF Discovery.
  StartRfDiscovery (false);

  if (sPollingEnabled) //KKW
    stopPolling_rfDiscoveryDisabled();

  PeerToPeer::GetInstance().EnableP2pListening (false);
  sP2pEnabled = false;

/* START [J14111100-1] - support secure element */
  if (screenoffCE)
  {
    {
      NCI_DEBUG ("%s: Force Set Tech mask to 0x0 and NFA_EnablePolling", __FUNCTION__);
      SyncEventGuard guard (sNfaEnableDisablePollingEvent);
      status = NFA_EnablePolling (0x0);
      if (status == NFA_STATUS_OK)
      {
        NCI_DEBUG ("%s: wait for enable event", __FUNCTION__);
        sNfaEnableDisablePollingEvent.Wait (); //wait for NFA_POLL_ENABLED_EVT
      }
      else
      {
        NCI_ERROR ("%s: fail enable polling; error=0x%X", __FUNCTION__, status);
      }
    }

    // Enable C/E during screen off
    NCI_DEBUG ("%s: StartRfDiscovery() for Card enable", __FUNCTION__);
    StartRfDiscovery (true);
  }
/* END [J14111100-1] - support secure element */

  sDiscoveryEnabled = false;
  //if nothing is active after this, then tell the controller to power down
  if (! PowerSwitch::GetInstance ().SetModeOff (PowerSwitch::DISCOVERY))
    PowerSwitch::GetInstance ().SetLevel (PowerSwitch::LOW_POWER);

  // We may have had RF field notifications that did not cause
  // any activate/deactive events. For example, caused by wireless
  // charging orbs. Those may cause us to go to sleep while the last
  // field event was indicating a field. To prevent sticking in that
  // state, always reset the rf field status when we disable discovery.
  SecureElement::GetInstance().ResetRfFieldStatus();

  NCI_DEBUG ("%s: exit", __FUNCTION__);
}


bool NfcManager::EnablePolling()
{
  return StartStopPolling(true);
}

bool NfcManager::DisablePolling()
{
  return StartStopPolling(false);
}

bool NfcManager::EnableP2pListening()
{
  return PeerToPeer::GetInstance().EnableP2pListening(true);
}

bool NfcManager::DisableP2pListening()
{
  return PeerToPeer::GetInstance().EnableP2pListening(false);
}

bool NfcManager::CheckLlcp()
{
  // Not used in NCI case.
  return true;
}

bool NfcManager::ActivateLlcp()
{
  // Not used in NCI case.
  return true;
}

ILlcpSocket* NfcManager::CreateLlcpSocket(int aSap,
                                          int aMiu,
                                          int aRw,
                                          int aBufLen)
{
  NCI_DEBUG("enter; sap=%d; miu=%d; rw=%d; buffer len=%d", aSap, aMiu, aRw, aBufLen);

  const uint32_t handle = PeerToPeer::GetInstance().GetNewHandle();
  if (!(PeerToPeer::GetInstance().CreateClient(handle, aMiu, aRw)))
  {
    NCI_ERROR("fail create p2p client");
  }

  LlcpSocket* pLlcpSocket = new LlcpSocket(handle, aSap, aMiu, aRw);

  NCI_DEBUG("exit");
  return static_cast<ILlcpSocket*>(pLlcpSocket);
}

ILlcpServerSocket* NfcManager::CreateLlcpServerSocket(int aSap,
                                                      const char* aSn,
                                                      int aMiu,
                                                      int aRw,
                                                      int aBufLen)
{
  NCI_DEBUG("enter; sap=%d; sn =%s; miu=%d; rw=%d; buffer len= %d", aSap, aSn, aMiu, aRw, aBufLen);
  const uint32_t handle = PeerToPeer::GetInstance().GetNewHandle();
  LlcpServiceSocket* pLlcpServiceSocket = new LlcpServiceSocket(handle, aBufLen, aMiu, aRw);

  if (!(PeerToPeer::GetInstance().RegisterServer(handle, aSn)))
  {
    NCI_ERROR("register server fail");
    return NULL;
  }

  NCI_DEBUG("exit");
  return static_cast<ILlcpServerSocket*>(pLlcpServiceSocket);
}

bool NfcManager::EnableSecureElement()
{
  NCI_DEBUG("enter");
  bool result = true;

  if (sIsSecElemSelected)
  {
    NCI_DEBUG("already selected");
    return result;
  }

  PowerSwitch::GetInstance().SetLevel(PowerSwitch::FULL_POWER);

  if (sRfEnabled)
  {
    // Stop RF Discovery if we were polling.
    StartRfDiscovery(false);
  }

  result = SecureElement::GetInstance().Activate();
  if (result)
  {
    SecureElement::GetInstance().RouteToSecureElement();
  }

  sIsSecElemSelected = true;

  StartRfDiscovery(true);
  PowerSwitch::GetInstance().SetModeOn(PowerSwitch::SE_ROUTING);

  return result;
}

bool NfcManager::DisableSecureElement()
{
  NCI_DEBUG("enter");
  bool result = true;
  bool reDiscover = false;

  if (!sIsSecElemSelected)
  {
    NCI_ERROR("already deselected");
    goto TheEnd;
  }

  if (PowerSwitch::GetInstance().GetLevel() == PowerSwitch::LOW_POWER)
  {
    NCI_DEBUG("do not deselect while power is OFF");
    sIsSecElemSelected = false;
    goto TheEnd;
  }

  if (sRfEnabled)
  {
    // Stop RF Discovery if we were polling.
    StartRfDiscovery(false);
    reDiscover = true;
  }

  //result = SecureElement::GetInstance().RouteToDefault();
  sIsSecElemSelected = false;

  //if controller is not routing to sec elems AND there is no pipe connected,
  //then turn off the sec elems
  if (!SecureElement::GetInstance().IsBusy())
  {
    SecureElement::GetInstance().Deactivate();
  }

TheEnd:
  //if nothing is active after this, then tell the controller to power down
  if (!PowerSwitch::GetInstance().SetModeOff(PowerSwitch::SE_ROUTING))
  {
    PowerSwitch::GetInstance().SetLevel(PowerSwitch::LOW_POWER);
  }
  return result;
}

tNFA_STATUS NfcManager::Disable(bool aGraceful)
{
#if 0 // SLSI_MIFARE
  EXTNS_Close();
#endif
  return NFA_Disable(aGraceful);
}

/* START [J14111303] - screen or power state */
void NfcManager::SetScreenOrPowerState(int state)
{
  NCI_DEBUG("enter, screen state = %d ", state);
  doSetScreenOrPowerState (state);

  NCI_DEBUG("exit");
}
/* END [J14111303] - screen or power state */

bool NfcManager::SetmPOSDedicatedMode(bool enabled)
{
  NCI_DEBUG("enabled = %d ", enabled);
  return SLSI_mPOSDedicatedMode (enabled);
}

/* START [D18020701] - mPOS and EMV PCD test */
bool NfcManager::NfcSelfTest(int type)
{
  NCI_DEBUG("type = %d ", type);
  switch(type)
  {
    case 0:
    {
    //ANALOG_FILED_ON
      type = 0x0A;
      break;
    }
    case 1:
    {
    //ANALOG_FILED_OFF
      type = 0x0B;
      break;
    }
    case 2:
    {
    //ANALOG_FIELD_RESET
      type = 0x0C;
      break;
    }
    case 3:
    {
    //ANALOG_FIELD_POWER_OFF
      type = 0x0D;
      break;
    }
    case 4: //EVT_Transaction for Removal Procedure
    case 5: //EVT_Transaction for PowerOff Procedure 
      break;
  }
  return SLSI_mPOSConfigforTest(type);
}
/* END [D18020701] - mPOS and EMV PCD test */

/**
 * Private functions.
 */
static void HandleRfDiscoveryEvent(tNFC_RESULT_DEVT* aDiscoveredDevice)
{
  if (aDiscoveredDevice->more)
  {
    // There is more discovery notification coming.
    return;
  }

  bool isP2p = NfcTag::GetInstance().IsP2pDiscovered();
  if (isP2p)
  {
    // Select the peer that supports P2P.
    NfcTag::GetInstance().SelectP2p();
  }
  else
  {
    // Select the first of multiple tags that is discovered.
    NfcTag::GetInstance().SelectFirstTag();
  }
}

void NfaDeviceManagementCallback(uint8_t aDmEvent, tNFA_DM_CBACK_DATA* aEventData)
{
  NCI_DEBUG("enter; event=0x%X", aDmEvent);

  switch (aDmEvent)
  {
    // Result of NFA_Enable.
    case NFA_DM_ENABLE_EVT:
    {
      SyncEventGuard guard(sNfaEnableEvent);
      NCI_DEBUG("NFA_DM_ENABLE_EVT; status=0x%X", aEventData->status);
      sIsNfaEnabled = aEventData->status == NFA_STATUS_OK;
      sIsDisabling = false;
      sNfaEnableEvent.NotifyOne();
      break;
    }
    // Result of NFA_Disable.
    case NFA_DM_DISABLE_EVT:
    {
      SyncEventGuard guard(sNfaDisableEvent);
      NCI_DEBUG("NFA_DM_DISABLE_EVT");
      sIsNfaEnabled = false;
      sIsDisabling = false;
      sNfaDisableEvent.NotifyOne();
      break;
    }
    // Result of NFA_SetConfig.
    case NFA_DM_SET_CONFIG_EVT:
    {
      NCI_DEBUG("NFA_DM_SET_CONFIG_EVT");
      SyncEventGuard guard(sNfaSetConfigEvent);
      sNfaSetConfigEvent.NotifyOne();
      break;
    }
    // Result of NFA_GetConfig.
    case NFA_DM_GET_CONFIG_EVT:
    {
      NCI_DEBUG("NFA_DM_GET_CONFIG_EVT");
      SyncEventGuard guard(sNfaGetConfigEvent);
      if (aEventData->status == NFA_STATUS_OK &&
          aEventData->get_config.tlv_size <= sizeof(sConfig))
      {
        sCurrentConfigLen = aEventData->get_config.tlv_size;
        memcpy(sConfig, aEventData->get_config.param_tlvs, aEventData->get_config.tlv_size);
      }
      else
      {
        NCI_ERROR("NFA_DM_GET_CONFIG failed; status=0x%X", aEventData->status);
        sCurrentConfigLen = 0;
      }
      sNfaGetConfigEvent.NotifyOne();
      break;
    }
    case NFA_DM_RF_FIELD_EVT:
      if (sIsDisabling || !sIsNfaEnabled)
      {
        break;
      }
/* START [J14111101_Part3] - pending enable discovery during listen mode */
      if (aEventData->rf_field.rf_field_status == NFA_DM_RF_FIELD_ON)
        SecureElement::GetInstance().SetIsPeerInListenMode(true);
      else if ((aEventData->rf_field.rf_field_status == NFA_DM_RF_FIELD_OFF) &&
          (SecureElement::GetInstance().IsPeerInListenMode()))
      {
        SecureElement::GetInstance().SetIsPeerInListenMode(false);
        NCI_DEBUG("NFA_DEACTIVATED_EVT Status");
        slsiEventHandler (NFA_DEACTIVATED_EVT, NULL);
      }
/* END [J14111101_Part3] - pending enable discovery during listen mode */

/* START [J14111103] - workaround for rf_info after P2P */
      if (slsiIsFlag(SLSI_WRFLAG_SKIP_RF_NTF)) // skip just one
      {
        slsiClearFlag(SLSI_WRFLAG_SKIP_RF_NTF);
        break;
      }
/* END [J14111103] - workaround for rf_info after P2P */

      if (!sP2pActive && aEventData->rf_field.status == NFA_STATUS_OK)
      {
        SecureElement::GetInstance().NotifyRfFieldEvent(
          aEventData->rf_field.rf_field_status == NFA_DM_RF_FIELD_ON);
      }
      break;

    case NFA_DM_NFCC_TRANSPORT_ERR_EVT:
      NCI_ERROR ("NFA_DM_NFCC_TRANSPORT_ERR_EVT; abort");
      break;

    case NFA_DM_NFCC_TIMEOUT_EVT:
    {
      if (aDmEvent == NFA_DM_NFCC_TIMEOUT_EVT)
      {
        NCI_ERROR("NFA_DM_NFCC_TIMEOUT_EVT; abort all outstanding operations");
      }
      else
      {
        NCI_ERROR("NFA_DM_NFCC_TRANSPORT_ERR_EVT; abort all outstanding operations");
      }
      NfcTagManager::DoAbortWaits();
      NfcTag::GetInstance().Abort();
      // TODO : Implement LLCP.
      {
        NCI_DEBUG("aborting sNfaEnableDisablePollingEvent");
        SyncEventGuard guard(sNfaEnableDisablePollingEvent);
        sNfaEnableDisablePollingEvent.NotifyOne();
      }
      {
        NCI_DEBUG("aborting sNfaEnableEvent");
        SyncEventGuard guard(sNfaEnableEvent);
        sNfaEnableEvent.NotifyOne();
      }
      {
        NCI_DEBUG("aborting sNfaDisableEvent");
        SyncEventGuard guard(sNfaDisableEvent);
        sNfaDisableEvent.NotifyOne();
      }
      sDiscoveryEnabled = false;
      PowerSwitch::GetInstance().Abort();

      if (!sIsDisabling && sIsNfaEnabled)
      {
        NfcManager::Disable(false);
        sIsDisabling = true;
      }
      else
      {
        sIsNfaEnabled = false;
        sIsDisabling = false;
      }
      PowerSwitch::GetInstance().Initialize(PowerSwitch::UNKNOWN_LEVEL);
      NCI_ERROR("aborted all waiting events");
    }
    break;
    case NFA_DM_PWR_MODE_CHANGE_EVT:
      PowerSwitch::GetInstance().DeviceManagementCallback(aDmEvent, aEventData);
      break;
    default:
      NCI_DEBUG("unhandled event");
      break;
  }
}

static void NfaConnectionCallback(uint8_t aConnEvent, tNFA_CONN_EVT_DATA* aEventData)
{
  tNFA_STATUS status = NFA_STATUS_FAILED;
  NCI_DEBUG("enter; event=0x%X", aConnEvent);

/* START [J00000000] - SLSI callback hooker */
  slsiEventHandler (aConnEvent, aEventData);
/* END [J00000000] - SLSI callback hooker */

  switch (aConnEvent)
  {
    // Whether polling successfully started.
    case NFA_POLL_ENABLED_EVT:
    {
      NCI_DEBUG("NFA_POLL_ENABLED_EVT: status = 0x%X", aEventData->status);

      SyncEventGuard guard(sNfaEnableDisablePollingEvent);
      sNfaEnableDisablePollingEvent.NotifyOne();
      break;
    }
    // Listening/Polling stopped.
    case NFA_POLL_DISABLED_EVT:
    {
      NCI_DEBUG("NFA_POLL_DISABLED_EVT: status = 0x%X", aEventData->status);

      SyncEventGuard guard(sNfaEnableDisablePollingEvent);
      sNfaEnableDisablePollingEvent.NotifyOne();
      break;
    }
    // RF Discovery started.
    case NFA_RF_DISCOVERY_STARTED_EVT:
    {
      NCI_DEBUG("NFA_RF_DISCOVERY_STARTED_EVT: status = 0x%X", aEventData->status);

      SyncEventGuard guard(sNfaEnableDisablePollingEvent);
      sNfaEnableDisablePollingEvent.NotifyOne();
      break;
    }
    // RF Discovery stopped event.
    case NFA_RF_DISCOVERY_STOPPED_EVT:
    {
      NCI_DEBUG("NFA_RF_DISCOVERY_STOPPED_EVT: status = 0x%X", aEventData->status);

      SyncEventGuard guard(sNfaEnableDisablePollingEvent);
      sNfaEnableDisablePollingEvent.NotifyOne();
      break;
    }
    // NFC link/protocol discovery notificaiton.
    case NFA_DISC_RESULT_EVT:
      status = aEventData->disc_result.status;
      NCI_DEBUG("NFA_DISC_RESULT_EVT: status = 0x%X", status);
      if (status != NFA_STATUS_OK)
      {
        NCI_ERROR("NFA_DISC_RESULT_EVT error: status = 0x%x", status);
      }
      else
      {
        NfcTag::GetInstance().ConnectionEventHandler(aConnEvent, aEventData);
        HandleRfDiscoveryEvent(&aEventData->disc_result.discovery_ntf);
      }
      break;

    case NFA_SELECT_RESULT_EVT: // NFC link/protocol discovery select response
      NCI_DEBUG("NFA_SELECT_RESULT_EVT: status = %d, gIsSelectingRfInterface = %d, sIsDisabling=%d", aEventData->status, gIsSelectingRfInterface, sIsDisabling);

      if (sIsDisabling)
        break;

      if (aEventData->status != NFA_STATUS_OK)
      {
        if (gIsSelectingRfInterface)
        {
          NfcTagManager::DoConnectStatus(false);
        }

        NCI_ERROR("NFA_SELECT_RESULT_EVT error: status = %d", aEventData->status);
        NFA_Deactivate (FALSE);
      }
      break;

    // NFC link/protocol activated.
    case NFA_ACTIVATED_EVT:
      NCI_DEBUG("NFA_ACTIVATED_EVT: gIsSelectingRfInterface=%d, sIsDisabling=%d",
              gIsSelectingRfInterface, sIsDisabling);
      if (sIsDisabling || !sIsNfaEnabled)
      {
        break;
      }

#if 0 // SLSI_MIFARE
      if (EXTNS_GetConnectFlag())
      {
        NfcTag::GetInstance().SetActivationState();
        NfcTagManager::DoConnectStatus(true);
        break;
      }
#endif
      NfcTag::GetInstance().SetActivationState();
      if (gIsSelectingRfInterface)
      {
        NfcTagManager::DoConnectStatus(true);
        break;
      }

      NfcTagManager::DoResetPresenceCheck();
      if (IsPeerToPeer(aEventData->activated))
      {
        sP2pActive = true;
        NCI_DEBUG("NFA_ACTIVATED_EVT; is p2p");
        // Disable RF field events in case of p2p.
        uint8_t nfa_disable_rf_events[] = { 0x00 };
        NCI_DEBUG("Disabling RF field events");
        status = NFA_SetConfig(NCI_PARAM_ID_RF_FIELD_INFO, sizeof(nfa_disable_rf_events),
            &nfa_disable_rf_events[0]);
        if (status == NFA_STATUS_OK)
        {
          NCI_DEBUG ("Disabled RF field events");
        }
        else
        {
          NCI_DEBUG ("Failed to disable RF field events");
        }

        // For the SE, consider the field to be on while p2p is active.
        SecureElement::GetInstance().NotifyRfFieldEvent(true);
      }
      else if (Pn544InteropIsBusy() == false)
      {
        NfcTag::GetInstance().ConnectionEventHandler(aConnEvent, aEventData);

        // We know it is not activating for P2P.  If it activated in
        // listen mode then it is likely for an SE transaction.
        // Send the RF Event.
        if (IsListenMode(aEventData->activated))
        {
          sSeRfActive = true;
          SecureElement::GetInstance().NotifyListenModeState(true);
        }
/* START [J14111102] - KOVIO BARCODE */
        else
        {
          tNFA_ACTIVATED& activated = aEventData->activated;
          if (NfcTag::GetInstance().IsSameKovio(activated))
          {
            status = NFA_Deactivate (FALSE);
            if (status != NFA_STATUS_OK)
            {
              NCI_DEBUG("%s: deactivate failed; error=0x%X", __FUNCTION__, status);
            }
            break;
          }
        }
/* END [J14111102] - KOVIO BARCODE */
      }
      break;
    // NFC link/protocol deactivated.
    case NFA_DEACTIVATED_EVT:
      NCI_DEBUG("NFA_DEACTIVATED_EVT Type: %u, gIsTagDeactivating: %d",
                aEventData->deactivated.type,gIsTagDeactivating);
      NfcTag::GetInstance().SetDeactivationState(aEventData->deactivated);
      if (aEventData->deactivated.type != NFA_DEACTIVATE_TYPE_SLEEP)
      {
        NfcTagManager::DoResetPresenceCheck();
        NfcTag::GetInstance().ConnectionEventHandler(aConnEvent, aEventData);
        NfcTagManager::DoAbortWaits();
        NfcTag::GetInstance().Abort();
      }
      else if (gIsTagDeactivating)
      {
        NfcTagManager::DoDeactivateStatus(0);
#if 0 // SLSI_MIFARE
      }
      else if (EXTNS_GetDeactivateFlag())
      {
        NfcTagManager::DoDeactivateStatus(0);
#endif
      }

      // If RF is activated for what we think is a Secure Element transaction
      // and it is deactivated to either IDLE or DISCOVERY mode, notify w/event.
      if ((aEventData->deactivated.type == NFA_DEACTIVATE_TYPE_IDLE) ||
          (aEventData->deactivated.type == NFA_DEACTIVATE_TYPE_DISCOVERY))
      {
        if (sSeRfActive)
        {
          sSeRfActive = false;
          if (!sIsDisabling && sIsNfaEnabled)
          {
            SecureElement::GetInstance().NotifyListenModeState(false);
          }
        }
        else if (sP2pActive)
        {
          sP2pActive = false;
          // Make sure RF field events are re-enabled.
          NCI_DEBUG("NFA_DEACTIVATED_EVT; is p2p");
          // Disable RF field events in case of p2p.
          uint8_t nfa_enable_rf_events[] = { 0x01 };

          if (!sIsDisabling && sIsNfaEnabled)
          {
            status = NFA_SetConfig(NCI_PARAM_ID_RF_FIELD_INFO, sizeof(nfa_enable_rf_events),
                &nfa_enable_rf_events[0]);
            if (status == NFA_STATUS_OK)
            {
              NCI_DEBUG ("%s: Enabled RF field events", __FUNCTION__);
              /* START [J14111103] - workaround for rf_info after P2P */
              slsiSetFlag(SLSI_WRFLAG_SKIP_RF_NTF);
              /* END [J14111103] - workaround for rf_info after P2P */
            }
            else
            {
              NCI_ERROR ("%s: Failed to enable RF field events", __FUNCTION__);
            }

            // Consider the field to be off at this point
            SecureElement::GetInstance().NotifyRfFieldEvent(false);
          }
        }
      }
      break;
    case NFA_TLV_DETECT_EVT: // TLV Detection complete
      status = aEventData->tlv_detect.status;
      NCI_DEBUG("NFA_TLV_DETECT_EVT: status = %d, protocol = %d, num_tlvs = %d, num_bytes = %d",
          status, aEventData->tlv_detect.protocol,
          aEventData->tlv_detect.num_tlvs, aEventData->tlv_detect.num_bytes);
      if (status != NFA_STATUS_OK)
      {
        NCI_ERROR("NFA_TLV_DETECT_EVT error: status = %d", status);
      }
      break;
    // NDEF Detection complete.
    case NFA_NDEF_DETECT_EVT:
      // If status is failure, it means the tag does not contain any or valid NDEF data.
      // Pass the failure status to the NFC Service.
      status = aEventData->ndef_detect.status;
      NCI_DEBUG("NFA_NDEF_DETECT_EVT: status = 0x%X, protocol = %u, "
                "max_size = %lu, cur_size = %lu, flags = 0x%X",
                status,
                aEventData->ndef_detect.protocol, aEventData->ndef_detect.max_size,
                aEventData->ndef_detect.cur_size, aEventData->ndef_detect.flags);
      NfcTag::GetInstance().ConnectionEventHandler(aConnEvent, aEventData);
      NfcTagManager::DoCheckNdefResult(status,
        aEventData->ndef_detect.max_size, aEventData->ndef_detect.cur_size,
        aEventData->ndef_detect.flags);
      break;
    // Data message received (for non-NDEF reads).
    case NFA_DATA_EVT:
      NCI_DEBUG("NFA_DATA_EVT:  len = %d", aEventData->data.len);
      NfcTagManager::DoTransceiveComplete(aEventData->data.p_data,
                                          aEventData->data.len);
      break;
    case NFA_RW_INTF_ERROR_EVT:
      NCI_DEBUG("NFC_RW_INTF_ERROR_EVT");
      NfcTagManager::NotifyRfTimeout();
      break;
    // Select completed.
    case NFA_SELECT_CPLT_EVT:
      status = aEventData->status;
      NCI_DEBUG("NFA_SELECT_CPLT_EVT: status = 0x%X", status);
      if (status != NFA_STATUS_OK)
      {
        NCI_ERROR("NFA_SELECT_CPLT_EVT error: status = 0x%X", status);
      }
      break;
    // NDEF-read or tag-specific-read completed.
    case NFA_READ_CPLT_EVT:
      NCI_DEBUG("NFA_READ_CPLT_EVT: status = 0x%X", aEventData->status);
      NfcTagManager::DoReadCompleted(aEventData->status);
      NfcTag::GetInstance().ConnectionEventHandler(aConnEvent, aEventData);
      break;
    // Write completed.
    case NFA_WRITE_CPLT_EVT:
      NCI_DEBUG("NFA_WRITE_CPLT_EVT: status = 0x%X", aEventData->status);
      NfcTagManager::DoWriteStatus(aEventData->status == NFA_STATUS_OK);
      break;
    // Tag set as Read only.
    case NFA_SET_TAG_RO_EVT:
      NCI_DEBUG("NFA_SET_TAG_RO_EVT: status = 0x%X", aEventData->status);
      NfcTagManager::DoMakeReadonlyResult(aEventData->status);
      break;
    // LLCP link is activated.
    case NFA_LLCP_ACTIVATED_EVT:
      NCI_DEBUG("NFA_LLCP_ACTIVATED_EVT: is_initiator: %d  remote_wks: %d"
                ", remote_lsc: %d, remote_link_miu: %d, local_link_miu: %d",
                aEventData->llcp_activated.is_initiator,
                aEventData->llcp_activated.remote_wks,
                aEventData->llcp_activated.remote_lsc,
                aEventData->llcp_activated.remote_link_miu,
                aEventData->llcp_activated.local_link_miu);

      PeerToPeer::GetInstance().LlcpActivatedHandler(aEventData->llcp_activated);
      break;
    // LLCP link is deactivated.
    case NFA_LLCP_DEACTIVATED_EVT:
      NCI_DEBUG("NFA_LLCP_DEACTIVATED_EVT");
      PeerToPeer::GetInstance().LlcpDeactivatedHandler(aEventData->llcp_deactivated);
      break;
    // Received first packet over llcp.
    case NFA_LLCP_FIRST_PACKET_RECEIVED_EVT:
      NCI_DEBUG("NFA_LLCP_FIRST_PACKET_RECEIVED_EVT");
      PeerToPeer::GetInstance().LlcpFirstPacketHandler();
      break;
    case NFA_PRESENCE_CHECK_EVT:
      NCI_DEBUG("NFA_PRESENCE_CHECK_EVT");
      NfcTagManager::DoPresenceCheckResult(aEventData->status);
      break;
    case NFA_FORMAT_CPLT_EVT:
      NCI_DEBUG("NFA_FORMAT_CPLT_EVT: status=0x%X", aEventData->status);
      NfcTagManager::DoFormatStatus(aEventData->status == NFA_STATUS_OK);
      break;
    case NFA_CE_UICC_LISTEN_CONFIGURED_EVT :
      NCI_DEBUG("NFA_CE_UICC_LISTEN_CONFIGURED_EVT : status=0x%X", aEventData->status);
      SecureElement::GetInstance().ConnectionEventHandler(aConnEvent, aEventData);
      break;
    case NFA_SET_P2P_LISTEN_TECH_EVT:
      NCI_DEBUG("NFA_SET_P2P_LISTEN_TECH_EVT");
      PeerToPeer::GetInstance().ConnectionEventHandler(aConnEvent, aEventData);
      break;
    default:
      NCI_ERROR("unknown event ????");
      break;
  }
}

void StartRfDiscovery(bool aIsStart)
{
  tNFA_STATUS status = NFA_STATUS_FAILED;

  NCI_DEBUG ("is start=%d, current=%d", aIsStart, sRfEnabled);
  SyncEventGuard guard(sNfaEnableDisablePollingEvent);
/* START [J14127013] - check current rf state */
  if (sRfEnabled == aIsStart)
    return;
/* END [J14127013] - check current rf state */
  status = aIsStart ? NFA_StartRfDiscovery() : NFA_StopRfDiscovery();
  if (status == NFA_STATUS_OK)
  {
    sNfaEnableDisablePollingEvent.Wait(); // Wait for NFA_RF_DISCOVERY_xxxx_EVT.
    sRfEnabled = aIsStart;
  }
  else
  {
    NCI_ERROR("NFA_StartRfDiscovery/NFA_StopRfDiscovery fail; error=0x%X", status);
  }
}

void DoStartupConfig()
{
/* START [J000000002] - SLSI initialize */
/* START [P150309001] - mPOS */
  slsiInitialize();
/* END [P150309001] - mPOS */
/* END [J000000002] - SLSI initialize */
}

bool IsNfcActive()
{
  return sIsNfaEnabled;
}

bool StartStopPolling(bool aIsStartPolling)
{
  NCI_DEBUG("enter; isStart=%u", aIsStartPolling);
  tNFA_STATUS stat = NFA_STATUS_FAILED;

  StartRfDiscovery(false);
  if (aIsStartPolling)
  {
    tNFA_TECHNOLOGY_MASK tech_mask = DEFAULT_TECH_MASK;
    unsigned long num = 0;
    if (GetNumValue(NAME_POLLING_TECH_MASK, &num, sizeof(num)))
      tech_mask = num;

    SyncEventGuard guard(sNfaEnableDisablePollingEvent);
    NCI_DEBUG("enable polling");
    stat = NFA_EnablePolling(tech_mask);
    if (stat == NFA_STATUS_OK)
    {
      NCI_DEBUG("wait for enable event");
      sNfaEnableDisablePollingEvent.Wait(); // Wait for NFA_POLL_ENABLED_EVT.
    }
    else
    {
      NCI_ERROR("NFA_EnablePolling fail, error=0x%X", stat);
    }
  }
  else
  {
    SyncEventGuard guard(sNfaEnableDisablePollingEvent);
    NCI_DEBUG("disable polling");
    stat = NFA_DisablePolling();
    if (stat == NFA_STATUS_OK)
    {
      sNfaEnableDisablePollingEvent.Wait(); // Wait for NFA_POLL_DISABLED_EVT.
    }
    else
    {
      NCI_ERROR("NFA_DisablePolling fail, error=0x%X", stat);
    }
  }
  StartRfDiscovery(true);
  NCI_DEBUG("exit");
  return stat == NFA_STATUS_OK;
}


tNFA_STATUS startPolling_rfDiscoveryDisabled(tNFA_TECHNOLOGY_MASK tech_mask)
{
  tNFA_STATUS stat = NFA_STATUS_FAILED;

  unsigned long num = 0;

  if (tech_mask == 0 && GetNumValue(NAME_POLLING_TECH_MASK, &num, sizeof(num)))
    tech_mask = num;
  else if (tech_mask == 0) tech_mask = DEFAULT_TECH_MASK;
  /* START [J16101001] - Read RF register project number */
  if (isUsingPollActiveF() == false)
  {
    NCI_DEBUG ("%s: removed Active Poll F from the options", __FUNCTION__);
    tech_mask &= ~NFA_TECHNOLOGY_MASK_F_ACTIVE;
  }
  /* END [J16101001] - Read RF register project number */

  SyncEventGuard guard (sNfaEnableDisablePollingEvent);
  NCI_DEBUG ("%s: enable polling", __FUNCTION__);
  stat = NFA_EnablePolling (tech_mask);
  if (stat == NFA_STATUS_OK)
  {
    NCI_DEBUG ("%s: wait for enable event", __FUNCTION__);
    sPollingEnabled = true;
    sNfaEnableDisablePollingEvent.Wait (); //wait for NFA_POLL_ENABLED_EVT
  }
  else
  {
    NCI_ERROR ("%s: fail enable polling; error=0x%X", __FUNCTION__, stat);
  }

  return stat;
}

tNFA_STATUS stopPolling_rfDiscoveryDisabled()
{
  tNFA_STATUS stat = NFA_STATUS_FAILED;

  SyncEventGuard guard (sNfaEnableDisablePollingEvent);
  NCI_DEBUG ("%s: disable polling", __FUNCTION__);
  stat = NFA_DisablePolling ();
  if (stat == NFA_STATUS_OK)
  {
    sPollingEnabled = false;
    sNfaEnableDisablePollingEvent.Wait (); //wait for NFA_POLL_DISABLED_EVT
  }
  else
  {
    NCI_ERROR ("%s: fail disable polling; error=0x%X", __FUNCTION__, stat);
  }

  return stat;
}

static bool IsPeerToPeer(tNFA_ACTIVATED& aActivated)
{
  return aActivated.activate_ntf.protocol == NFA_PROTOCOL_NFC_DEP;
}

static bool IsListenMode(tNFA_ACTIVATED& aActivated)
{
  return ((NFC_DISCOVERY_TYPE_LISTEN_A == aActivated.activate_ntf.rf_tech_param.mode) ||
          (NFC_DISCOVERY_TYPE_LISTEN_B == aActivated.activate_ntf.rf_tech_param.mode) ||
          (NFC_DISCOVERY_TYPE_LISTEN_F == aActivated.activate_ntf.rf_tech_param.mode) ||
          (NFC_DISCOVERY_TYPE_LISTEN_A_ACTIVE == aActivated.activate_ntf.rf_tech_param.mode) ||
          (NFC_DISCOVERY_TYPE_LISTEN_F_ACTIVE == aActivated.activate_ntf.rf_tech_param.mode) ||
          (NFC_DISCOVERY_TYPE_LISTEN_ISO15693 == aActivated.activate_ntf.rf_tech_param.mode) ||
          (NFC_DISCOVERY_TYPE_LISTEN_B_PRIME == aActivated.activate_ntf.rf_tech_param.mode));
}
