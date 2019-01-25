/*
 *    Copyright (C) 2017 SAMSUNG S.LSI
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at:
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 *   Author: Heejae Kim <heejae12.kim@samsung.com>
 *
 */

#include "OverrideLog.h"
#include "NfcDebug.h"
#include "NfcAdaptation.h"
#include "SyncEvent.h"
#include "PeerToPeer.h"
#include "NfcTag.h"
#include "config.h"
#include "PowerSwitch.h"
#include "Pn544Interop.h"
#include "SecureElement.h"
#include "Mutex.h"
#include "IntervalTimer.h"
#include <ScopedPrimitiveArray.h>
#include <ScopedLocalRef.h>
#include <ScopedUtfChars.h>
#include <ScopedPrimitiveArray.h>

extern "C"
{
  #include "nfa_api.h"
  #include "nfa_p2p_api.h"
  #include "rw_api.h"
  #include "nfa_ee_api.h"
  #include "nfc_brcm_defs.h"
  #include "ce_api.h"
}

/*****************************************************************************
 ** Extern variables and functions
 *****************************************************************************/
extern void StartRfDiscovery(bool isStart);
extern void nfcManager_enableDiscoveryImpl (tNFA_TECHNOLOGY_MASK tech_mask);
extern void nfcManager_disableDiscoveryImpl (bool screenoffCE);
extern bool IsNfcActive();

extern tNFA_STATUS stopPolling_rfDiscoveryDisabled();
extern tNFA_STATUS startPolling_rfDiscoveryDisabled(tNFA_TECHNOLOGY_MASK tech_mask);

extern tNFA_INTF_TYPE getCurrentRfInterface();
extern void setCurrentRfInterface(tNFA_INTF_TYPE rfInterface);

/*****************************************************************************
 ** Public variables and functions
 *****************************************************************************/
#define VENDCFG_MASK_NONE            0x00
#define VENDCFG_MASK_NFCEE_OFF       0x01 /* TODO: remove it, it is update by L */
#define VENDCFG_NFCEE_OFF            2
#define VENDCFG_NFCEE_ON             3

#define NCI_PROP_SET_SLEEP_TIME      0x1A    /* Last updated value: 20160530 */

#define NCI_PROP_GET_RFREG_OID       0x21
#define NCI_PROP_SET_RFREG_OID       0x22
#define NCI_PROP_SET_SIMVDD2_OID     0x30    /* [J16052700] Switch SIMVDD2 power source */
#define NCI_PROP_SIM_TEST            0x31
#define NCI_PROP_PRBS_TEST           0x32
#define NCI_PROP_SCREEN_STATE_OID    0x38
#define NCI_PROP_PARTIAL_AID_OID     0x39

#define NFA_PROP_GET_RFREG_EVT       0x61
#define NFA_PROP_SET_RFREG_EVT       0x62

#define VS_NO_RESPONSE               0xFF
#define VS_ZERO_LENGTH               0xFE

#define POWER_STATE_ON               6
#define POWER_STATE_OFF              7

/* [J14120201] - Generic ESE ID */
#define SEID_HOST                    0x00
#define SEID_ESE                     0x02
#define SEID_UICC                    0x03

#define SEID_GEN_HOST                0x00
#define SEID_GEN_ESE                 0x01
#define SEID_GEN_UICC                0x02

// S.LSI Additional Patch Flag
#define SLSI_PATCHFLAG_NONE                     0x0000
#define SLSI_PATCHFLAG_WAIT_ENABLE_DISCOVERY    0x0002
#define SLSI_PATCHFLAG_WAIT_DISABLE_DISCOVERY   0x0004
#define SLSI_PATCHFLAG_WAIT_SCREEN_STATE        0x0008

// S.LSI Additional WR Flag
#define SLSI_WRFLAG_SKIP_RF_NTF                 0x0010
#define SLSI_WRFLAG_NXP_P2P                     0x0040  /* PATCHID: J14111107 */

void slsiEventHandler(uint8_t event, tNFA_CONN_EVT_DATA* data);
void slsiInitialize(void);
void slsiDeinitialize(void);
void slsiSetFlag(uint16_t flag);
bool slsiIsFlag(uint16_t flag);
void slsiClearFlag(uint16_t flag);

/* [J14120201] - Generic ESE ID */
int slsiGetSeId(jint genSeId);
int slsiGetGenSeId(int seId);
int slsiGetGenSeId(tNFA_HANDLE handle);

/* [J14111101] - pending enable discovery during listen mode */
void enableDiscoveryAfterDeactivation (tNFA_TECHNOLOGY_MASK tech_mask);
void disableDiscoveryAfterDeactivation (void);

void slsiSetNfcSleepTimeout(unsigned long sec, int option);

/* [P150309001] - mPOS */
bool SLSI_mPOSGetInfo(bool enable);
bool SLSI_mPOSDedicatedMode(bool enable);
bool SLSI_mPOSSelectUiccHost(bool enable);
bool SLSI_mPOSConfigforTest(int nMode);

/* [J14111303] - screen or power state */
void doSetScreenOrPowerState (int state);

/* [J14111100] - support secure element */
void doSelectSecureElement(int seId);
void doDeselectSecureElement(int seId);
void setSecureElementListenTechMask(int tech_mask);
int getSecureElementTechList();

/* [J14121101] - Remaining AID Size */
int getRemainingAidTableSize();

/* [J14121201] - Aid Table Size */
int getAidTableSize();

bool checkFwVersion(int m1, int m2, int b1, int b2);
bool lessFwVersion(int m1, int m2, int b1, int b2);

/* [J15052703] - FW Update Req : Get FW Version */
bool sIsNfcOffStatus;
//extern bool sIsNfcOffStatus;
extern tNFC_FW_VERSION fw_version;

/* Additon for test */
int nfcManager_getFwVersion(JNIEnv* e, jobject o);
int doSWPSelfTest(uint8_t ch);
uint8_t doPRBStest (UINT8 tech, UINT8 rate);

/* workaround */
int doWorkaround(uint16_t wrId, void *arg);
bool isUsingPollActiveF(void);

/*****************************************************************************
 ** private variables and functions
 *****************************************************************************/
static SyncEvent      sNfaEnableEvent;     //event for NFA_Enable()
static SyncEvent      sNfaEnableDisablePollingEvent;  //event for NFA_EnablePolling(), NFA_DisablePolling()
static SyncEvent      sNfaSetConfigEvent;  // event for Set_Config....
static SyncEvent      sNfaVsCmdEvent;
static bool           sP2pActive = false;  // whether p2p was last active

static uint8_t        gNfaVsCmdResult;
static Mutex          gMutex;
static IntervalTimer  gTimer;
static IntervalTimer  gTimerNxpP2p;

static bool           bIsSecElemSelected = false;  //has NFC service selected a sec elem
static bool           bIsSlsiInit = false;
static bool           bNxpP2p = false;

tNFC_ACTIVATE_DEVT    gActivationData;
uint8_t               gScreenState;
uint8_t               gCurrentScreenState;
uint16_t              gRunningPatchFlag;
uint16_t              gVendorConfig;
uint32_t              gFwVer = 0;
bool                  gUsingPollActiveF = true;

static struct enableDiscoveryData
{
  uint16_t  tech_mask;
  bool      enable_lptd;
  bool      reader_mode;
  bool      enable_host_routing;
  bool      restart;
  bool      discovery_duration;
  bool      enable_p2p;
} sEnableDiscoveryData;

static void nfcManager_doSetScreenOrPowerStateImpl();
static void slsiDoPendedImpl (union sigval);
static void setPartialAID(uint8_t *p_option, UINT8 option_length);

static void generalVsCmdCallback (uint8_t event, uint16_t param_len, UINT8* p_param);
static void setPartialAIDCallback (uint8_t event, uint16_t param_len, UINT8* p_param);
static void nfaSimTestCallback(tNFC_VS_EVT event, uint16_t data_len, uint8_t *p_data);
static void nfaPrbsTestCallback(tNFC_VS_EVT event, uint16_t data_len, uint8_t *p_data);
static void doSetScreenStateCallback (uint8_t event, uint16_t param_len, UINT8* p_param);

static bool checkUsingPollActiveF(void);
static uint32_t getFwVersion();

/* [J14111107] - NXP P2P */
static void doWrNxpP2p (union sigval);


/*******************************************************************************
 **
 ** Block description:   slsi event hooker
 **
 ** User case:           slsiEventHandler (event, data)
 **
 *******************************************************************************/
void slsiEventHandler (uint8_t event, tNFA_CONN_EVT_DATA* data)
{
  if (!bIsSlsiInit)
  {
    NCI_DEBUG("%s: slsi additional is alread deinit", __FUNCTION__);
    return;
  }

  NCI_DEBUG("%s: event= %u", __FUNCTION__, event);

  switch (event)
  {
    case NFA_ACTIVATED_EVT:
    {
      tNFA_ACTIVATED& activated = data->activated;
      gActivationData = activated.activate_ntf;    // TODO: is it copied?
      /* START [J15033101] - TECH recovery after NXP_P2P workaround */
      if(slsiIsFlag(SLSI_WRFLAG_NXP_P2P))
        gTimerNxpP2p.Kill();
      /* END [J15033101] - TECH recovery after NXP_P2P workaround */
      break;
    }
    case NFA_DEACTIVATED_EVT:
    case NFA_CE_DEACTIVATED_EVT:
      /* START [J14111107] - NXP P2P */
      gTimerNxpP2p.Kill();
      if (slsiIsFlag(SLSI_WRFLAG_NXP_P2P))
        doWorkaround(SLSI_WRFLAG_NXP_P2P, NULL);

      /* START [J14111101] - pending enable discovery during listen mode */
      if (SecureElement::GetInstance().IsPeerInListenMode() != true)
      {
        if ((slsiIsFlag(SLSI_PATCHFLAG_WAIT_ENABLE_DISCOVERY)) ||
            (slsiIsFlag(SLSI_PATCHFLAG_WAIT_DISABLE_DISCOVERY)) ||
            (slsiIsFlag(SLSI_PATCHFLAG_WAIT_SCREEN_STATE)))
        {
          gTimer.Set (100, slsiDoPendedImpl); // TODO: is it enough 100ms?
        }
      }
      /* END [J14111101] - pending enable discovery during listen mode */
      break;
    case NFA_NDEF_DETECT_EVT:
      if (data->ndef_detect.status == NFA_STATUS_FAILED)
      {
        if ((NCI_STATUS_RF_PROTOCOL_ERR == data->ndef_detect.detail_status) ||
            (NCI_STATUS_TIMEOUT == data->ndef_detect.detail_status))
        {
          if (gActivationData.protocol == NCI_PROTOCOL_ISO_DEP &&
              gActivationData.rf_tech_param.mode == NCI_DISCOVERY_TYPE_POLL_A)
          {
            bool val = true;
            doWorkaround(SLSI_WRFLAG_NXP_P2P, (void *)&val);
          }
        }
      }
      break;
  }
}

static void slsiDoPendedImpl (union sigval)
{
  NCI_DEBUG ("%s: enter", __FUNCTION__);
  gMutex.Lock();
/* START [J17070101] - Abort pendings during NFC off */
  if (sIsNfcOffStatus)
  {
    NCI_DEBUG ("%s: Abort pendings due to Disabling NFC", __FUNCTION__);
    slsiClearFlag(SLSI_PATCHFLAG_WAIT_ENABLE_DISCOVERY | SLSI_PATCHFLAG_WAIT_DISABLE_DISCOVERY |
        SLSI_PATCHFLAG_WAIT_SCREEN_STATE);
  }
/* END [J17070101] - Abort pendings during NFC off */

/* START [J14111101_Part3] - pending enable discovery during listen mode */
  else if (SecureElement::GetInstance().IsPeerInListenMode())
/* END [J14111101_Part3] - pending enable discovery during listen mode */
  {
    NCI_DEBUG ("%s: activated listen again", __FUNCTION__);
  }
  else
  {
    if (slsiIsFlag(SLSI_PATCHFLAG_WAIT_ENABLE_DISCOVERY))
    {
      slsiClearFlag(SLSI_PATCHFLAG_WAIT_ENABLE_DISCOVERY);
      nfcManager_enableDiscoveryImpl (sEnableDiscoveryData.tech_mask);
    }

    if (slsiIsFlag(SLSI_PATCHFLAG_WAIT_DISABLE_DISCOVERY))
    {
      slsiClearFlag(SLSI_PATCHFLAG_WAIT_DISABLE_DISCOVERY);
      nfcManager_disableDiscoveryImpl (true);
    }

    if (slsiIsFlag(SLSI_PATCHFLAG_WAIT_SCREEN_STATE))
    {
      slsiClearFlag(SLSI_PATCHFLAG_WAIT_SCREEN_STATE);
      nfcManager_doSetScreenOrPowerStateImpl ();
    }
  }
  gMutex.Unlock();
  NCI_DEBUG ("%s: exit", __FUNCTION__);
}

/* START [J00000001] - SLSI vendor config */
/*******************************************************************************
 **
 ** Block description:   set vendor configuration
 **
 ** User case:           be called by user
 **
 *******************************************************************************/
void setVenConfigValue(int config)
{
  uint16_t venConfig = (UINT16) config;
  NCI_DEBUG ("%s: venConfig : 0x%X", __FUNCTION__, venConfig);

  switch (venConfig)
  {
/* START [J14111201] - disable SE during NFC-OFF */
    case VENDCFG_NFCEE_OFF:
      gVendorConfig |= VENDCFG_MASK_NFCEE_OFF;
      break;

    case VENDCFG_NFCEE_ON:
      gVendorConfig &= ~VENDCFG_MASK_NFCEE_OFF;
      break;
  }
/* END [J14111201] - disable SE during NFC-OFF */
}
/* END [J00000001] */

/*******************************************************************************
 **
 ** Block description:   General Prop command response handler
 **                      This function ignores the state of response
 **
 ** User case:           slsiStartupConfig()
 **
 *******************************************************************************/
static void generalVsCmdCallback (uint8_t event, uint16_t param_len, UINT8* p_param)
{
  NCI_DEBUG ("%s: event=0x%X", __FUNCTION__, event);
  SyncEventGuard guard (sNfaVsCmdEvent);
  sNfaVsCmdEvent.NotifyOne();
}

/*******************************************************************************
 **
 ** Block description:   startup configuration
 **
 ** User case:           slsiStartupConfig()
 **
 *******************************************************************************/
void slsiInitialize()
{
  NCI_DEBUG("enter slsiInitialize");

  memset((void *)&sEnableDiscoveryData, 0, sizeof(sEnableDiscoveryData));

  bIsSlsiInit = true;
  gRunningPatchFlag = SLSI_PATCHFLAG_NONE;

  getFwVersion();

  /* START [J14121202] - skip RF INFO notify to service */
  slsiSetFlag(SLSI_WRFLAG_SKIP_RF_NTF);
  /* END [J14121202] - skip RF INFO notify to service */
  uint8_t  field_info_param[] = { 0x01 };
  NCI_DEBUG ("%s: Enabling RF field events", __FUNCTION__);
  if (NFA_SetConfig(NCI_PARAM_ID_RF_FIELD_INFO, sizeof(field_info_param), &field_info_param[0]) != NFA_STATUS_OK)
    NCI_ERROR ("%s: fail to set RF field", __FUNCTION__);

  setPartialAID(NULL, 0);
  gVendorConfig = VENDCFG_MASK_NONE;
}

/*******************************************************************************
 **
 ** Block description:   slsi deinitialize
 **
 ** User case:           slsiDeinitialize()
 **
 *******************************************************************************/
void slsiDeinitialize(void)
{
  gRunningPatchFlag = SLSI_PATCHFLAG_NONE;

/* START [J16052700] - Switch SIMVDD2 */
  {
    SyncEventGuard guard (sNfaVsCmdEvent);
    NCI_DEBUG("%s: Set SIMVDD2 to external", __FUNCTION__);
    uint8_t vdd_src = 0x01; /* 0: Internal, 1: External */
    if (NFA_STATUS_OK == NFA_SendVsCommand(NCI_PROP_SET_SIMVDD2_OID, 1, &vdd_src, generalVsCmdCallback))
      sNfaVsCmdEvent.Wait (1000);
  }
/* END [J16052700] - Switch SIMVDD2 */

/* START [J18020201] SE ModeSet */
  SecureElement::GetInstance().SetSecureElementModeset();
/* END [J18020201] SE ModeSet */

/* START [J14111201] - disable SE during NFC-OFF */
  if ((gVendorConfig & VENDCFG_MASK_NFCEE_OFF) == VENDCFG_MASK_NFCEE_OFF)
    //RoutingManager::getInstance().onNfccShutdown();
/* END [J14111201] - disable SE during NFC-OFF */

    gVendorConfig = VENDCFG_MASK_NONE;

/* START [J14111100] - support secure element */
  bIsSecElemSelected = false;
/* END [J14111100] - support secure element */
  bIsSlsiInit = false;
}

/* START [J14111101] - pending enable discovery during listen mode */
/*******************************************************************************
 **
 ** Function:        pending enable discovery during listen mode
 **
 ** Description:     Set screen state
 **
 ** Returns:         None
 **
 *****
 **************************************************************************/
void enableDiscoveryAfterDeactivation (tNFA_TECHNOLOGY_MASK tech_mask)
{
  gMutex.Lock();
  slsiClearFlag(SLSI_PATCHFLAG_WAIT_DISABLE_DISCOVERY);
  slsiSetFlag(SLSI_PATCHFLAG_WAIT_ENABLE_DISCOVERY);

  sEnableDiscoveryData.tech_mask             = tech_mask;
  gMutex.Unlock();
}

void disableDiscoveryAfterDeactivation (void)
{
  gMutex.Lock();
  slsiClearFlag(SLSI_PATCHFLAG_WAIT_ENABLE_DISCOVERY);
  slsiSetFlag(SLSI_PATCHFLAG_WAIT_DISABLE_DISCOVERY);
  gMutex.Unlock();
}
/* END [J14111101] */

/*******************************************************************************
 **
 ** Block description:   Set partial AID
 **
 ** User case:           setPartialAID(p_option, option_length)
 **                       - p_option:       partial setting
 **                       - option_length:  length of p_option
 **
 ** Returns:             void
 **
 *******************************************************************************/
static void setPartialAID(uint8_t *p_option, UINT8 option_length)
{
  tNFA_STATUS stat = NFA_STATUS_FAILED;
  uint8_t buffer[10];
  uint8_t length, *p_payload;

  NCI_DEBUG ("%s: enter", __FUNCTION__);

  memset(buffer, 0, 5);

  if (p_option == NULL || option_length < 1)
  {
    if(GetStrValue(NAME_PARTIAL_AID, (char *)buffer, sizeof(buffer)))
    {
      length = buffer[0];
      p_payload = buffer + 1;
    }
    else
      goto TheEnd;
  }
  else
  {
    length = option_length;
    p_payload = p_option;
  }

  NCI_DEBUG ("%s: length=%d", __FUNCTION__, length);

  {
    SyncEventGuard guard (sNfaVsCmdEvent);
    stat = NFA_SendVsCommand(NCI_PROP_PARTIAL_AID_OID, length, p_payload, setPartialAIDCallback);
    if (stat == NFA_STATUS_OK)
      sNfaVsCmdEvent.Wait ();
  }

TheEnd:
  NCI_DEBUG ("%s: exit; state=0x%x", __FUNCTION__, stat);
}

static void setPartialAIDCallback (uint8_t event, uint16_t param_len, UINT8* p_param)
{
  NCI_DEBUG ("%s: event=0x%X", __FUNCTION__, event);
  SyncEventGuard guard (sNfaVsCmdEvent);
  sNfaVsCmdEvent.NotifyOne();
}

/*******************************************************************************
 **
 ** Function:        doWrNxpP2p
 **
 ** Description:     Start or stop workaround to facilitate P2P connection with NXP controllers.
 **                  isStartWorkAround: true to start work-around; false to stop work-around.
 **
 ** Returns:         None.
 **
 *******************************************************************************/
#define NAME_WR_NXP_P2P  "NAME_WR_NXP_P2P"
void doWrNxpP2p (union sigval)
{
/* START [J14111107] - NXP P2P */
  NCI_DEBUG ("%s: enter; isStartWorkAround=%u, WarType:%d", __FUNCTION__, bNxpP2p, gRunningPatchFlag);
  unsigned long isNxpP2pWr = 0;

  if (!GetNumValue(NAME_WR_NXP_P2P, &isNxpP2pWr, sizeof(isNxpP2pWr)))
    isNxpP2pWr = 1;

  if (isNxpP2pWr == 0)
  {
    NCI_DEBUG ("%s: work around does not enabled", __FUNCTION__);
    return;
  }

  StartRfDiscovery (false);
  if (bNxpP2p)
  {
    stopPolling_rfDiscoveryDisabled ();

    // if you want listen only during WR, remove it
    NCI_DEBUG("%s: Enable F tech only", __FUNCTION__);
    if (startPolling_rfDiscoveryDisabled (NFA_TECHNOLOGY_MASK_F) == NFA_STATUS_OK) {
      slsiSetFlag(SLSI_WRFLAG_NXP_P2P);
/* START [J15033101] - TECH recovery after NXP_P2P workaround */
      bNxpP2p = false;
      gTimerNxpP2p.Set(100, doWrNxpP2p);
/* END [J15033101] - TECH recovery after NXP_P2P workaround */
    }
  }
  else
  {
    slsiClearFlag(SLSI_WRFLAG_NXP_P2P);

    // if you want listen only during WR, remove it
    stopPolling_rfDiscoveryDisabled ();

    startPolling_rfDiscoveryDisabled (sEnableDiscoveryData.tech_mask);
  }

  StartRfDiscovery (true);
  NCI_DEBUG ("%s: exit", __FUNCTION__);
}

/*******************************************************************************
 **
 ** Block description:   Factory test
 **
 ** User case:
 **
 ** Returns:
 **
 *******************************************************************************/
uint8_t doPRBStest (UINT8 tech, UINT8 rate)
{
  uint8_t  p[2];
  p[0] = (uint8_t)tech;
  p[1] = (uint8_t)rate;

  SyncEventGuard guard (sNfaVsCmdEvent);
  gNfaVsCmdResult = VS_NO_RESPONSE;
  if (NFA_SendVsCommand(NCI_PROP_PRBS_TEST, 2, p, nfaPrbsTestCallback) == NFA_STATUS_OK)
  {
    NCI_DEBUG("%s: waiting sNfaVsCmdEvent", __FUNCTION__);
    sNfaVsCmdEvent.Wait();
  }

  return gNfaVsCmdResult;
}

int doSWPSelfTest(uint8_t ch)
{
  tNFA_STATUS stat = NFA_STATUS_FAILED;

  SyncEventGuard guard (sNfaVsCmdEvent);
  gNfaVsCmdResult = VS_NO_RESPONSE;
  // TODO: original -> ch... ndees smart card confirm
  stat = NFA_SendVsCommand(NCI_PROP_SIM_TEST, 1, (uint8_t*)&ch, nfaSimTestCallback);
  if (stat == NFA_STATUS_OK)
  {
    NCI_DEBUG("%s: waiting sNfaVsCmdEvent", __FUNCTION__);
    sNfaVsCmdEvent.Wait();
  }

  NCI_DEBUG("%s: exit; gNfaVsCmdResult = 0x%02X", __FUNCTION__, gNfaVsCmdResult);
  return (gNfaVsCmdResult == 0) ? true : false;
}

static void nfaSimTestCallback(tNFC_VS_EVT event, uint16_t data_len, uint8_t *p_data)
{
  NCI_DEBUG("%s: enter; event=0x%02X, len=%d", __FUNCTION__, event, data_len);

  for (int i=0; i < data_len; i++)
  {
    NCI_DEBUG("[%d]%02X", i, p_data[i]);
  }

  SyncEventGuard guard (sNfaVsCmdEvent);
  if (data_len)
    gNfaVsCmdResult = p_data[data_len - 1];
  else
    gNfaVsCmdResult = VS_ZERO_LENGTH;
  sNfaVsCmdEvent.NotifyOne();
  NCI_DEBUG("%s: exit", __FUNCTION__);
}

static void nfaPrbsTestCallback(tNFC_VS_EVT event, uint16_t data_len, uint8_t *p_data)
{
  NCI_DEBUG("%s: enter; event=0x%02X, len=%d", __FUNCTION__, event, data_len);

  for (int i=0; i < data_len; i++)
  {
    NCI_DEBUG("[%d]%02X", i, p_data[i]);
  }

  SyncEventGuard guard (sNfaVsCmdEvent);
  if (data_len)
    gNfaVsCmdResult = p_data[data_len - 1];
  else
    gNfaVsCmdResult = VS_ZERO_LENGTH;
  sNfaVsCmdEvent.NotifyOne();
  NCI_DEBUG("%s: exit", __FUNCTION__);
}

/*******************************************************************************
 **
 ** Block description:   get F/W version
 **
 ** User case:           call from Service (getFWVersion)
 **
 ** Returns:             F/W version
 **
 *******************************************************************************/
int nfcManager_getFwVersion(JNIEnv*, jobject)
{
  return (jint)getFwVersion();
}

#define FW_VER(m1, m2, b1, b2)      ((m1 * 0x10000) + (m2 * 0x100) + (b2 < 10 ? b2 : b2+6))
bool checkFwVersion(int m1, int m2, int b1, int b2)
{
  return (gFwVer == (uint32_t) FW_VER(m1, m2, b1, b2));
}

bool lessFwVersion(int m1, int m2, int b1, int b2)
{
  return (gFwVer < (uint32_t) FW_VER(m1, m2, b1, b2));
}

static uint32_t getFwVersion()
{
  tNFC_FW_VERSION fwVersion;

  NCI_DEBUG("%s : Enter", __FUNCTION__);
  memset(&fwVersion, 0, sizeof(fwVersion));

  /* START [J15052703] - FW Update Req : Get FW Version*/
  if(sIsNfcOffStatus) // NFC-OFF Status
    fwVersion = NfcAdaptation::fw_version;
  /* END [J15052703] - FW Update Req : Get FW Version*/
  else // NFC-ON Status
    fwVersion = NFC_getFWVersion();

  if(fwVersion.major_version)
  {
    gFwVer =  (uint32_t)fwVersion.major_version * 0x10000;
    gFwVer += (uint16_t)fwVersion.minor_version * 0x100;
    //gFwVer += (uint16_t)fwVersion.build_info_high;		 // Can't not display high value of build info.
    gFwVer += (uint16_t)fwVersion.build_info_low;
    NCI_DEBUG("%s : F/W Version = %x.%x.%x (%lx)", __FUNCTION__, fwVersion.major_version,
        fwVersion.minor_version, fwVersion.build_info_low, gFwVer);
  }
  NCI_DEBUG("%s : Exit", __FUNCTION__);
  return gFwVer;
}

/*******************************************************************************
 **
 ** Function:        doSelectSecureElement
 **
 ** Description:     NFC controller starts routing data in listen mode.
 **
 ** Returns:         None
 **
 *******************************************************************************/
void doSelectSecureElement(int seId)
{
  NCI_DEBUG ("%s: enter", __FUNCTION__);
  bool stat = true;

  if (bIsSecElemSelected)
  {
    NCI_DEBUG ("%s: already selected", __FUNCTION__);
    goto TheEnd;
  }

  PowerSwitch::GetInstance ().SetLevel (PowerSwitch::FULL_POWER);

  StartRfDiscovery (false);

  stat = SecureElement::GetInstance().Activate ();

  bIsSecElemSelected = true;

  StartRfDiscovery (true);
  PowerSwitch::GetInstance ().SetModeOn (PowerSwitch::SE_ROUTING);

TheEnd:
  NCI_DEBUG ("%s: exit", __FUNCTION__);
}

/* START [J14120901] - set ce listen tech */
/*******************************************************************************
 **
 ** Function:        getSecureElementTechList
 **
 **
 ** CAS_Kr : Added on AR2.2.2 : [Request] Test code for eSE setting
 **  1)API for getting  Technology infor mation which is supported on eSE.
 **
 *******************************************************************************/
int getSecureElementTechList()
{
  NCI_DEBUG ("%s: enter", __FUNCTION__);

  //NCI_DEBUG ("%s: exit - ret = 0x%02lX", __FUNCTION__, listenMask);
  //return listenMask;
  return 0;
}

void setSecureElementListenTechMask(int tech_mask)
{
  NCI_DEBUG ("%s: enter - tech_mask = 0x%02X", __FUNCTION__, tech_mask);

  StartRfDiscovery (false);

  PeerToPeer::GetInstance().EnableP2pListening(false);
  //RoutingManager::getInstance().setCeListenTech(tech_mask);

  StartRfDiscovery (true);

  NCI_DEBUG ("%s: exit", __FUNCTION__);
}
/* END [J14120901] */

/*******************************************************************************
 **
 ** Function:        doDeselectSecureElement
 **
 ** Description:     NFC controller stops routing data in listen mode.
 **
 ** Returns:         None
 **
 *******************************************************************************/
void doDeselectSecureElement(int seId)
{
  NCI_DEBUG ("%s: enter", __FUNCTION__);
  bool bRestartDiscovery = false;

  if (! bIsSecElemSelected)
  {
    NCI_ERROR ("%s: already deselected", __FUNCTION__);
    goto TheEnd2;
  }

  if (PowerSwitch::GetInstance ().GetLevel() == PowerSwitch::LOW_POWER)
  {
    NCI_DEBUG ("%s: do not deselect while power is OFF", __FUNCTION__);
    bIsSecElemSelected = false;
    goto TheEnd;
  }

  // TODO: how to know this?
  if (1/*sRfEnabled*/) {
    // Stop RF Discovery if we were polling
    StartRfDiscovery (false);
    bRestartDiscovery = true;
  }

  //if controller is not routing to sec elems AND there is no pipe connected,
  //then turn off the sec elems
  if (SecureElement::GetInstance().IsBusy() == false)
    SecureElement::GetInstance().Deactivate ();

  bIsSecElemSelected = false;

TheEnd:
  if (bRestartDiscovery)
    StartRfDiscovery (true);

  //if nothing is active after this, then tell the controller to power down
  if (! PowerSwitch::GetInstance ().SetModeOff (PowerSwitch::SE_ROUTING))
    PowerSwitch::GetInstance ().SetLevel (PowerSwitch::LOW_POWER);

TheEnd2:
  NCI_DEBUG ("%s: exit", __FUNCTION__);
}

/* START [J14121101] - Remaining AID Size */
/*******************************************************************************
 **
 ** Function:        getRemainingAidTableSize
 **
 ** Description:     Calculate the free space (bytes) in listen-mode routing table.
 **
 ** Returns:         Number of bytes free.
 **
 *******************************************************************************/
int getRemainingAidTableSize()
{
  int remainingAidTableSize = 0;

  remainingAidTableSize = NFA_GetRemainingAidTableSize();
  NCI_DEBUG ("%s: Remaining AID Table Size : %d", __FUNCTION__, remainingAidTableSize);

  return remainingAidTableSize;
}
/* END [J14121101] */


/* START [J14121201] - Aid Table Size */
/*******************************************************************************
 **
 ** Function:        getAidTableSize
 **
 ** Description:     This function is called to get the AID routing table max size.
 **
 ** Returns:         NFA_EE_MAX_AID_CFG_LEN(230)
 **
 *******************************************************************************/
int getAidTableSize()
{
  return NFA_GetAidTableSize();
}
/* END [J14121201] */

/* START [J14111303] - screen or power state */
/*******************************************************************************
 **
 ** Function:        doSetScreenOrPowerState
 **
 ** Description:     Set screen state
 **
 ** Returns:         None
 **
 *****
 **************************************************************************/
void doSetScreenOrPowerState (int state)
{
  if (state == POWER_STATE_ON || state == POWER_STATE_OFF)
  {
    /* power state off means to disable NFCEE (NFC service is disabled)
     * power state on means to keep NFCEE state (NFC service is running but power is off) */
    NCI_DEBUG ("%s: enter; power state=0x%X", __FUNCTION__, state);
    setVenConfigValue(state == POWER_STATE_ON ? VENDCFG_NFCEE_ON : VENDCFG_NFCEE_OFF);
    goto TheEnd;
  }

  NCI_DEBUG ("%s: enter; screen state=0x%X", __FUNCTION__, state);

  if(!IsNfcActive())
  {
    NCI_DEBUG("%s: NFC stack is closed. skip sending screen state", __FUNCTION__);
    return;
  }

  gScreenState = state;
  gMutex.Lock();
/* START [J15033101] - TECH recovery after NXP_P2P workaround */
  slsiClearFlag(SLSI_WRFLAG_NXP_P2P);
/* END [J15033101] - TECH recovery after NXP_P2P workaround */

/* START [J14111101_Part3] - pending enable discovery during listen mode */
  if (SecureElement::GetInstance().IsPeerInListenMode() && !sP2pActive)
/* END [J14111101_Part3] - pending enable discovery during listen mode */
  {
    slsiSetFlag(SLSI_PATCHFLAG_WAIT_SCREEN_STATE);
  }
  else
    nfcManager_doSetScreenOrPowerStateImpl ();
  gMutex.Unlock();

TheEnd:
  NCI_DEBUG ("%s: exit", __FUNCTION__);
}

static void nfcManager_doSetScreenOrPowerStateImpl()
{
  uint8_t screen_state[2] = {0x00, 0x00};
  tNFA_STATUS stat = NFA_STATUS_FAILED;

  NCI_DEBUG ("%s: enter; request state=0x%X, current state=0x%X", __FUNCTION__, gScreenState, gCurrentScreenState);
  if (gScreenState == gCurrentScreenState)
    return;

  screen_state[0] = gScreenState;
  {
    SyncEventGuard guard (sNfaVsCmdEvent);
    stat = NFA_SendVsCommand(NCI_PROP_SCREEN_STATE_OID, 1, screen_state, doSetScreenStateCallback);

    if (stat == NFA_STATUS_OK)
    {
      if (sNfaVsCmdEvent.Wait (2000) == false)
        NCI_ERROR ("%s: vs command timed out", __FUNCTION__);
    }
  }
  NCI_DEBUG ("%s: exit", __FUNCTION__);
}

static void doSetScreenStateCallback (uint8_t event, uint16_t param_len, UINT8* p_param)
{
  NCI_DEBUG ("%s: Enter; event=0x%X", __FUNCTION__, event);

  SyncEventGuard guard (sNfaVsCmdEvent);
  sNfaVsCmdEvent.NotifyOne();

  // update
  if (param_len > 0 && p_param[param_len - 1] == NFA_STATUS_OK)
    gCurrentScreenState = gScreenState;

  NCI_DEBUG ("%s: Exit", __FUNCTION__);
}

/* START [J15112002] - CE Error Notification */
/* START [J17010901] - Filtering OID in Prop. Callback */
#define VSN_OID_CE_ERROR    0x3F

void nfaVSNtfCallback(uint8_t event, uint16_t param_len, UINT8 *p_param)
{
  if (NULL == p_param)
  {
    NCI_ERROR("%s: Abnormal callback. Parameter == NULL", __FUNCTION__);
    return;
  }

  int oid = p_param[1];
  if (oid == VSN_OID_CE_ERROR)
  {
    NCI_DEBUG ("%s: enter: CE Error Notification Callback", __FUNCTION__);
    if (param_len > 3)
    {
      int error = p_param[3];
      int status = (p_param[2] == 2 ? p_param[4] : 0x00);

      //RoutingManager::getInstance().CeErrorEventHandler(NULL, (RoutingManager::CeErrorEvent)error, status);
    }
    NCI_DEBUG ("%s: exit: CE Error Notification Callback", __FUNCTION__);
  }
}
/* END [J17010901] */
/* END [J15112002] - CE Error Notification */

/*******************************************************************************
 **
 ** Block description:   flag control
 **
 ** User case:
 **
 ** Returns:
 **
 *******************************************************************************/
void slsiSetFlag(uint16_t flag)
{
  gRunningPatchFlag |= flag;
}
bool slsiIsFlag(uint16_t flag)
{
  return ((gRunningPatchFlag & flag) == flag);
}
void slsiClearFlag(uint16_t flag)
{
  gRunningPatchFlag &= ~flag;
}

/* START [P150309001] System LSI - mPOS */
#define VS_MPOS_MODE_OID                0x33
#define VS_MPOS_DISABLE_MPOS_MODE       0x00
#define VS_MPOS_ENABLE_MPOS_MODE        0x01
#define VS_MPOS_DISABLE_RF_DISCOVERY    0x02
#define VS_MPOS_ENABLE_RF_DISCOVERY     0x03
#define VS_MPOS_STATUS_OK               0x00
#define VS_MPOS_STATUS_FAILED           0x01

void nfamPOSDedicatedModeCallback(tNFC_VS_EVT mPosEvent, uint16_t length, uint8_t *p_data)
{
  int i;
  char respBuf[256];

  NCI_DEBUG("%s: enter; event=0x%02X, len=%d", __FUNCTION__, mPosEvent, length);
  memset(respBuf, 0, sizeof(respBuf));

  SyncEventGuard guard (sNfaVsCmdEvent);
  for(i = 0; i < length; ++i)
    sprintf(&respBuf[i*3], "%02X ", p_data[i]);
  NCI_DEBUG("%s: Data = %s", __FUNCTION__, respBuf);

  gNfaVsCmdResult = p_data[length-1];
  sNfaVsCmdEvent.NotifyOne();

  NCI_DEBUG("%s: exit", __FUNCTION__);
}


void nfamPOSRFDiscoveryCallback(tNFC_VS_EVT mPosEvent, uint16_t length, uint8_t *p_data)
{
  int i;
  char respBuf[256];

  NCI_DEBUG("%s: enter; event=0x%02X, len=%d", __FUNCTION__, mPosEvent, length);
  memset(respBuf, 0, sizeof(respBuf));

  for(i = 0; i < length; ++i)
    sprintf(&respBuf[i*3], "%02X ", p_data[i]);
  NCI_DEBUG("%s: Data = %s", __FUNCTION__, respBuf);

  gNfaVsCmdResult = p_data[length-1];

  NCI_DEBUG("%s: exit", __FUNCTION__);
}

void nfamPOSSelectUiccHostCallback(tNFC_VS_EVT mPosEvent, uint16_t length, uint8_t *p_data)
{
  int i;
  char respBuf[256];

  NCI_DEBUG("%s: enter; event=0x%02X, len=%d", __FUNCTION__, mPosEvent, length);
  memset(respBuf, 0, sizeof(respBuf));

  SyncEventGuard guard (sNfaVsCmdEvent);
  for(i = 0; i < length; ++i)
    sprintf(&respBuf[i*3], "%02X ", p_data[i]);
  NCI_DEBUG("%s: Data = %s", __FUNCTION__, respBuf);

  gNfaVsCmdResult = p_data[length-1];
  sNfaVsCmdEvent.NotifyOne();

  NCI_DEBUG("%s: exit", __FUNCTION__);
}

/* START [J17010901] - Filtering OID in Prop. Callback */
void nfaVSCNtfCallback(uint8_t event, uint16_t param_len, UINT8 *p_param)
{
  if(NULL == p_param)
  {
    NCI_ERROR("%s: Abnormal callback. Parameter == NULL", __FUNCTION__);
    return;
  }

  int oid = p_param[1];
  if (oid == VS_MPOS_MODE_OID)
  {
    uint8_t   mPOSRFEnable;
    uint8_t   stat = NFA_STATUS_FAILED;
    (void)event;

    NCI_DEBUG ("%s: enter: mPOS Callback", __FUNCTION__);

    if (param_len > 3)
    {
      int status = p_param[3];
      if(status == 0x00) // EVT_READER_REQ_NTF
      {
        NCI_DEBUG("%s: ******* Enable mPOS RF Discvoery *******", __FUNCTION__);
        mPOSRFEnable = VS_MPOS_ENABLE_RF_DISCOVERY;
        {
          stat = NFA_SendVsCommand(VS_MPOS_MODE_OID, 1, &mPOSRFEnable, nfamPOSRFDiscoveryCallback);
          if (stat == NFA_STATUS_OK)
          {
            NCI_DEBUG("%s: waiting sNfaVsCmdEvent", __FUNCTION__);
          }
        }
      }
      else if((status > 0x01) && (status < 0x05))  // Notify mPOS Error to NFC Service.
      {
        NCI_ERROR("%s: ******* Received ERROR Notification about mPOS Operation *******", __FUNCTION__);
      }
    }
    NCI_DEBUG ("%s: exit: mPOS Callback", __FUNCTION__);
  }
}
/* END [J17010901] */

void nfamPOSConfigforTestCallback(tNFC_VS_EVT mPosEvent, uint16_t length, uint8_t *p_data)
{
  int i;
  char respBuf[256];

  NCI_DEBUG("%s: enter; event=0x%02X, len=%d", __FUNCTION__, mPosEvent, length);
  memset(respBuf, 0, sizeof(respBuf));

  SyncEventGuard guard (sNfaVsCmdEvent);
  for(i = 0; i < length; ++i)
    sprintf(&respBuf[i*3], "%02X ", p_data[i]);
  NCI_DEBUG("%s: Data = %s", __FUNCTION__, respBuf);

  gNfaVsCmdResult = p_data[length-1];
  sNfaVsCmdEvent.NotifyOne();

  NCI_DEBUG("%s: exit", __FUNCTION__);
}

bool SLSI_mPOSGetInfo(bool enable)
{
  bool ret = JNI_FALSE;
  unsigned long num = 0;

  if (GetNumValue("MPOS_MODE_ENABLE", &num, sizeof(num)))
  {
    if(num == JNI_TRUE)
      ret = num;
    else
      ret = JNI_FALSE;
  }
  else
    ret = JNI_FALSE;

  return ret;
}

/*******************************************************************************
 **
 ** Function:        SLSI_mPOSDedicatedMode
 **
 ** Description:     RF Discovery Stop (State IDLE)
 **                  mPOS Dedicated Mode Start / Stop
 **
 **                    jobject: Java object.
 **                    enable: Dedicated mode Config Start/Stop.
 **
 ** Returns:         True if Dedicated mode config ok.
 *******************************************************************************/
bool SLSI_mPOSDedicatedMode(bool enable)
{
  uint8_t   mPOSEnable;
  uint8_t   stat = NFA_STATUS_FAILED;
  bool    ret = JNI_FALSE;

  NCI_DEBUG("%s: Enter", __FUNCTION__);

  mPOSEnable = (enable == true) ? VS_MPOS_ENABLE_MPOS_MODE : VS_MPOS_DISABLE_MPOS_MODE;
  {
    if(mPOSEnable == VS_MPOS_ENABLE_MPOS_MODE)  // Enter mPOS Dedicate Mode..!!
    {
      NCI_DEBUG("%s: ******* RF Deactivation - IDLE *******", __FUNCTION__);
      StartRfDiscovery(false);
      NCI_DEBUG("%s: ******* Enter mPOS Dedicated Mode. *******", __FUNCTION__);

      {
        SyncEventGuard guard (sNfaVsCmdEvent);
        stat = NFA_SendVsCommand(VS_MPOS_MODE_OID, 1, &mPOSEnable, nfamPOSDedicatedModeCallback);
        if (stat == NFA_STATUS_OK)
        {
          NCI_DEBUG("%s: waiting sNfaVsCmdEvent", __FUNCTION__);
          sNfaVsCmdEvent.Wait();
        }
        ret = (gNfaVsCmdResult == VS_MPOS_STATUS_OK) ? JNI_TRUE : JNI_FALSE;
      }

      {
        stat = NFA_RegVSCback (true,nfaVSCNtfCallback); //CallBack for VS NTF
        if (stat == NFA_STATUS_OK)
          NCI_DEBUG("%s: Registered EVT_READER_REQ_NTF for mPOS..!!", __FUNCTION__);
      }
    }
    else // Exit mPOS Dedicate Mode..!!
    {
      stat = NFA_RegVSCback (false,nfaVSCNtfCallback); //CallBack for VS NTF
      if(stat == NFA_STATUS_OK)
        NCI_DEBUG("Deregistered mPOS callback.");

      SyncEventGuard guard (sNfaVsCmdEvent);
      stat = NFA_SendVsCommand(VS_MPOS_MODE_OID, 1, &mPOSEnable, nfamPOSDedicatedModeCallback);
      if (stat == NFA_STATUS_OK)
      {
        NCI_DEBUG("%s: waiting sNfaVsCmdEvent", __FUNCTION__);
        sNfaVsCmdEvent.Wait();
      }
      ret = (gNfaVsCmdResult == VS_MPOS_STATUS_OK) ? JNI_TRUE : JNI_FALSE;

      if(mPOSEnable == VS_MPOS_DISABLE_MPOS_MODE)
      {
        NCI_DEBUG("%s: ******* Exit mPOS Dedicated Mode. *******", __FUNCTION__);
        StartRfDiscovery(true);
        NCI_DEBUG("%s: ******* Start RF Discovery *******", __FUNCTION__);
      }
    }
  }

  NCI_DEBUG("%s: Exit", __FUNCTION__);
  return ret;
}

bool SLSI_mPOSSelectUiccHost(bool enable)
{
  uint8_t   mPOSUiccHost;
  uint8_t   stat = NFA_STATUS_FAILED;
  bool    ret = JNI_FALSE;

  NCI_DEBUG("%s: Enter", __FUNCTION__);
  StartRfDiscovery(false);

  mPOSUiccHost = (enable == true) ? VS_MPOS_ENABLE_MPOS_MODE : VS_MPOS_DISABLE_MPOS_MODE;
  {
    SyncEventGuard guard (sNfaVsCmdEvent);
    stat = NFA_SendVsCommand(0x34, 1, &mPOSUiccHost, nfamPOSSelectUiccHostCallback);
    if (stat == NFA_STATUS_OK)
    {
      NCI_DEBUG("%s: waiting sNfaVsCmdEvent", __FUNCTION__);
      sNfaVsCmdEvent.Wait();
    }
    ret = (gNfaVsCmdResult == VS_MPOS_STATUS_OK) ? JNI_TRUE : JNI_FALSE;
  }

  NCI_DEBUG("%s: Exit", __FUNCTION__);
  return ret;
}

bool SLSI_mPOSConfigforTest(int nTestMode)
{
  uint8_t   mPOSTestmode;
  uint8_t   stat = NFA_STATUS_FAILED;
  bool    ret = JNI_FALSE;

  NCI_DEBUG("%s: Enter", __FUNCTION__);
  mPOSTestmode = (uint8_t)nTestMode;

  SyncEventGuard guard (sNfaVsCmdEvent);
  stat = NFA_SendVsCommand(0x34, 1, &mPOSTestmode, nfamPOSConfigforTestCallback);
  if (stat == NFA_STATUS_OK)
  {
    NCI_DEBUG("%s: waiting sNfaVsCmdEvent", __FUNCTION__);
    sNfaVsCmdEvent.Wait();
  }
  ret = (gNfaVsCmdResult == VS_MPOS_STATUS_OK) ? JNI_TRUE : JNI_FALSE;

  NCI_DEBUG("%s: Exit", __FUNCTION__);
  return ret;
}
/* END [P150309001] System LSI - mPOS   */

/* START [J15091601] System LSI - GCF Reader Test */
#define VS_GCF_READER_TEST_OID      0x36
#define VS_GCF_DISABLE              0x00
#define VS_GCF_ENABLE               0x01
#define VS_GCF_STATUS_OK            0x00
#define VS_GCF_STATUS_REJECT        0x01

void nfaGcfReaderTestModeCallback(tNFC_VS_EVT mGCFEvent, uint16_t length, uint8_t *p_data)
{
  int i;
  char respBuf[256];

  NCI_DEBUG("%s: enter; event=0x%02X, len=%d", __FUNCTION__, mGCFEvent, length);
  memset(respBuf, 0, sizeof(respBuf));

  SyncEventGuard guard (sNfaVsCmdEvent);
  for(i = 0; i < length; ++i)
    sprintf(&respBuf[i*3], "%02X ", p_data[i]);
  NCI_DEBUG("%s: Data = %s", __FUNCTION__, respBuf);

  gNfaVsCmdResult = p_data[length-1];
  sNfaVsCmdEvent.NotifyOne();

  NCI_DEBUG("%s: exit", __FUNCTION__);
}

void slsiSetNfcSleepTimeout(unsigned long sec, int option)
{
  uint8_t value[3];

  NCI_DEBUG("%s: enter;", __FUNCTION__);

  if (option > 0)
  {
    value[0] = sec % 60;
    value[1] = sec / 60;
  }
  value[2] = option;

  tNFA_STATUS stat = NFA_SendVsCommand(NCI_PROP_SET_SLEEP_TIME, 3, value, NULL);
  if (stat != NFA_STATUS_OK)
    NCI_ERROR("%s: failed to set DTA sleep timeout", __FUNCTION__);

  NCI_DEBUG("%s: exit;", __FUNCTION__);
}

bool nfcManager_SLSI_GcfReaderTestMode(bool enable)
{
  uint8_t   mGCFEnable;
  uint8_t   stat = NFA_STATUS_FAILED;
  bool    ret = JNI_FALSE;

  NCI_DEBUG("%s: Enter, enable = %d", __FUNCTION__, enable);

  mGCFEnable = (enable == true) ? VS_GCF_ENABLE : VS_GCF_DISABLE;

  {
    if(mGCFEnable == VS_GCF_ENABLE)  // Enter GCF Reader Test ..!!
    {
      NCI_DEBUG("%s: ******* [GCF] RF Deactivation - IDLE *******", __FUNCTION__);
      StartRfDiscovery(false);
      NCI_DEBUG("%s: ******* [GCF] Enter mPOS Dedicated Mode. *******", __FUNCTION__);

      {
        SyncEventGuard guard (sNfaVsCmdEvent);
        stat = NFA_SendVsCommand(VS_GCF_READER_TEST_OID, 1, &mGCFEnable, nfaGcfReaderTestModeCallback);
        if (stat == NFA_STATUS_OK)
        {
          NCI_DEBUG("%s: waiting sNfaVsCmdEvent", __FUNCTION__);
          sNfaVsCmdEvent.Wait();
        }
      }

      {
        SyncEventGuard guard (sNfaVsCmdEvent);
        stat = NFA_SendVsCommand(0x34, 1, &mGCFEnable, nfamPOSSelectUiccHostCallback);
        if (stat == NFA_STATUS_OK)
        {
          NCI_DEBUG("%s: waiting sNfaVsCmdEvent", __FUNCTION__);
          sNfaVsCmdEvent.Wait();
        }
      }

      {
        SyncEventGuard guard (sNfaVsCmdEvent);
        stat = NFA_SendVsCommand(VS_MPOS_MODE_OID, 1, &mGCFEnable, nfamPOSDedicatedModeCallback);
        if (stat == NFA_STATUS_OK)
        {
          NCI_DEBUG("%s: waiting sNfaVsCmdEvent", __FUNCTION__);
          sNfaVsCmdEvent.Wait();
        }
      }
    }

    else // Exit GCF Reader Test Mode..!!
    {
      {
        SyncEventGuard guard (sNfaVsCmdEvent);
        stat = NFA_SendVsCommand(VS_GCF_READER_TEST_OID, 1, &mGCFEnable, nfaGcfReaderTestModeCallback);
        if (stat == NFA_STATUS_OK)
        {
          NCI_DEBUG("%s: waiting sNfaVsCmdEvent", __FUNCTION__);
          sNfaVsCmdEvent.Wait();
        }
      }

      {
        SyncEventGuard guard (sNfaVsCmdEvent);
        stat = NFA_SendVsCommand(VS_MPOS_MODE_OID, 1, &mGCFEnable, nfamPOSDedicatedModeCallback);
        if (stat == NFA_STATUS_OK)
        {
          NCI_DEBUG("%s: waiting sNfaVsCmdEvent", __FUNCTION__);
          sNfaVsCmdEvent.Wait();
        }
      }

      if(mGCFEnable == VS_GCF_DISABLE)
      {
        NCI_DEBUG("%s: ******* [GCF] Exit mPOS Dedicated Mode. *******", __FUNCTION__);
        StartRfDiscovery(true);
        NCI_DEBUG("%s: ******* [GCF] Start RF Discovery *******", __FUNCTION__);
      }
    }

    ret = (gNfaVsCmdResult == VS_GCF_STATUS_OK) ? JNI_TRUE : JNI_FALSE;

  }

  NCI_DEBUG("%s: Exit", __FUNCTION__);
  return ret;
}
/* END [J15091601] System LSI - GCF Reader Test */

/* START [J14120201] - Generic ESE ID */
int slsiGetSeId(jint genSeId)
{
  switch (genSeId)
  {
    case SEID_GEN_ESE:
      return SEID_ESE;
    case SEID_GEN_UICC:
      return SEID_UICC;
    case SEID_GEN_HOST:
      return SEID_HOST;
  }
  return 0xFF;
}

int slsiGetGenSeId(int seId)
{
  switch (seId)
  {
    case SEID_ESE:
      return SEID_GEN_ESE;
    case SEID_UICC:
      return SEID_GEN_UICC;
    case SEID_HOST:
      return SEID_GEN_HOST;
  }
  return 0xFF;
}

int slsiGetGenSeId(tNFA_HANDLE handle)
{
  int seId = handle & ~NFA_HANDLE_GROUP_EE;
  return slsiGetGenSeId(seId);
}
/* END [J14120201] */

static void slientReset (union sigval)
{
  NCI_ERROR ("%s: silent reset", __FUNCTION__);
  exit(0);
}

int doWorkaround(uint16_t wrId, void *arg)
{
  switch (wrId)
  {
    case SLSI_WRFLAG_NXP_P2P:
      if (arg != NULL)
        bNxpP2p = *((bool *)arg);
      else
        bNxpP2p = false;

      gTimerNxpP2p.Set(10, doWrNxpP2p);
      break;
  }
  return 0;
}

/*******************************************************************************
 **
 ** Function:        isUsingPollActiveF
 **
 ** Description:     check the system can use Active F for poll.
 **
 ** Returns:         false if the option set to block active F.
 **
 *******************************************************************************/
bool isUsingPollActiveF(void)
{
  return gUsingPollActiveF;
}
/*******************************************************************************
 **
 ** Function:        checkUsingPollActiveF
 **
 ** Description:     get the using active f option from RF register file.
 **
 ** Returns:         false if the option set to block active F.
 **
 *******************************************************************************/
#define FILE_NAME_SIZE                  256
static int getFieldFromCfg(const char *field, char *storage, int storageSize)
{
  static char halCfgFile[] = "/etc/libnfc-sec-hal.conf";
  char buffer[FILE_NAME_SIZE], *pb = NULL, *pt = NULL;
  FILE* fd = NULL;

  if ((fd = fopen(halCfgFile, "r")) == NULL)
  {
    NCI_DEBUG("%s: failed to open file: %s", __FUNCTION__, halCfgFile);
    return 0;
  }

  while (!feof(fd) && fgets(buffer, sizeof(buffer)-1, fd))
  {
    if (!strncmp(buffer, field, strlen(field)))
    {
      pb = buffer;
      pt = storage;
      while ( *pb++ != '"' && (pb - buffer) < FILE_NAME_SIZE);
      while ( *pb != '"' && (pt - storage) < storageSize)
        *pt++ = *pb++;
      *pt = '\0';

      break;
    }
  }

  fclose(fd);
  if (pt == NULL)
  {
    NCI_DEBUG("%s: failed to get %s field from %s", __FUNCTION__, field, halCfgFile);
    return 0;
  }
  return (pt - storage);
}

/*******************************************************************************
 **
 ** Function:        checkUsingPollActiveF
 **
 ** Description:     get the using active f option from RF register file.
 **
 ** Returns:         false if the option set to block active F.
 **
 *******************************************************************************/
#define HAL_CFG_RF_REG_PATH             "RF_DIR_PATH"
#define HAL_CFG_RF_REG_FILE             "RF_FILE_NAME"
#define RFREG_META_DATA_LENGTH          16
#define RFREG_META_MW_OPTION_LENGTH     1
#define RFREG_META_MW_OPTION            10
#define RFREG_MW_OPTION_BLOCK_PAF       0x80    // Using Poll Active F
static bool checkUsingPollActiveF(void)
{
  char rfRegFile[FILE_NAME_SIZE];
  FILE* fd = NULL;
  uint8_t option;
  int len;

  NCI_DEBUG ("%s: enter", __FUNCTION__);

  // Get RFREG_FILE field from hal conf.  (This patch does not use Single SKU and uses only default field.)
  len = getFieldFromCfg (HAL_CFG_RF_REG_PATH, rfRegFile, sizeof(rfRegFile));
  strcat(rfRegFile, "/");
  if (len <= 0 || getFieldFromCfg (HAL_CFG_RF_REG_FILE, rfRegFile + len + 1, sizeof(rfRegFile) - len - 1) <= 0)
    return true;

  // Open (default) RF register file.
  NCI_DEBUG("%s: rfRegFile=%s", __FUNCTION__, rfRegFile);

  if ((fd = fopen(rfRegFile, "rb")) == NULL)
  {
    NCI_DEBUG("%s: failed to open file", __FUNCTION__);
    return true;
  }

  fseek(fd, 0, SEEK_SET);
  if (fseek(fd, -RFREG_META_DATA_LENGTH + RFREG_META_MW_OPTION, SEEK_END) < 0)
  {
    NCI_DEBUG("%s: file size error", __FUNCTION__);
    fclose(fd);
    return true;
  }

  len = fread(&option, 1, RFREG_META_MW_OPTION_LENGTH, fd);
  fclose(fd);

  if (len <= 0)
    return true;

  // if the bit is set, then active poll F is not used.
  NCI_DEBUG("%s: option value: %x", __FUNCTION__, option);
  if (option & RFREG_MW_OPTION_BLOCK_PAF)
    return false;
  return true;
}
