/*
 * This file is part of the libCEC(R) library.
 *
 * libCEC(R) is Copyright (C) 2011-2012 Pulse-Eight Limited.  All rights reserved.
 * libCEC(R) is an original work, containing original code.
 *
 * libCEC(R) is a trademark of Pulse-Eight Limited.
 *
 * This program is dual-licensed; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *
 * Alternatively, you can license this library under a commercial license,
 * please contact Pulse-Eight Licensing for more information.
 *
 * For more information contact:
 * Pulse-Eight Licensing       <license@pulse-eight.com>
 *     http://www.pulse-eight.com/
 *     http://www.pulse-eight.net/
 */

#include "ioctlCECAdapterCommunication.h"
#include "../platform/sockets/ioctlport.h"
#include "../platform/util/timeutils.h"
#include "../LibCEC.h"
#include "../CECProcessor.h"

using namespace std;
using namespace CEC;
using namespace PLATFORM;

#define CEC_ADAPTER_PING_TIMEOUT 15000

CioctlCECAdapterCommunication::CioctlCECAdapterCommunication(CCECProcessor *processor, const char *strPort, uint16_t iBaudRate /* = 38400 */) :
    m_port(NULL),
    m_processor(processor),
    m_bHasData(false),
    m_iLineTimeout(0),
    m_iFirmwareVersion(CEC_FW_VERSION_UNKNOWN),
    m_lastDestination(CECDEVICE_UNKNOWN),
    m_bNextIsEscaped(false),
    m_bGotStart(false),
    m_bInitialised(false),
    m_pingThread(NULL)
{
  for (unsigned int iPtr = 0; iPtr < 15; iPtr++)
    m_bWaitingForAck[iPtr] = false;
  fprintf(stderr, "** IOCTL new port\r\n");
  m_port = new CioctlPort(strPort);
}

typedef enum
{
   CEC_VERSION_Reserved = 0x00, /*!< CEC Reserved */
   CEC_VERSION_Reserved1 = 0x01, /*!< CEC Reserved */
   CEC_VERSION_Reserved2 = 0x02, /*!< CEC Reserved */
   CEC_VERSION_Reserved3 = 0x03, /*!< CEC Reserved */
   CEC_VERSION_1_3a     = 0x04, /*!< CEC Version 1.3a */
   CEC_VERSION_1_4      = 0x05  /*!< CEC Version 1.4  */
} tmdlHdmiCECVersion_t;

typedef tmdlHdmiCECVersion_t cec_version;
typedef struct
{
   unsigned char count;
   unsigned char service;
   unsigned char addr;
   unsigned char data[15];
} cec_frame;

#define CEC_IOCTL_BASE 0x40

enum {
   /* driver specific */
   CEC_VERBOSE_ON_CMD = 0,
   CEC_VERBOSE_OFF_CMD,
   CEC_BYEBYE_CMD,

   /* CEC */
   CEC_IOCTL_RX_ADDR_CMD, /* receiver logical address selector */
   CEC_IOCTL_PHY_ADDR_CMD, /* physical address selector */
   CEC_IOCTL_WAIT_FRAME_CMD,
   CEC_IOCTL_ABORT_MSG_CMD,
   CEC_IOCTL_ACTIVE_SRC_CMD,
   CEC_IOCTL_VERSION_CMD,
   CEC_IOCTL_CLEAR_ANALOGUE_TIMER_CMD,
   CEC_IOCTL_CLEAR_DIGITAL_TIMER_CMD,
   CEC_IOCTL_CLEAR_EXT_TIMER_WITH_EXT_PLUG_CMD,
   CEC_IOCTL_CLEAR_EXT_TIMER_WITH_PHY_ADDR_CMD,
   CEC_IOCTL_DECK_CTRL_CMD,
   CEC_IOCTL_DECK_STATUS_CMD,
   CEC_IOCTL_DEVICE_VENDOR_ID_CMD,
   CEC_IOCTL_FEATURE_ABORT_CMD,
   CEC_IOCTL_GET_CEC_VERSION_CMD,
   CEC_IOCTL_GET_MENU_LANGUAGE_CMD,
   CEC_IOCTL_GIVE_AUDIO_STATUS_CMD,
   CEC_IOCTL_GIVE_DECK_STATUS_CMD,
   CEC_IOCTL_GIVE_DEVICE_POWER_STATUS_CMD,
   CEC_IOCTL_GIVE_DEVICE_VENDOR_ID_CMD,
   CEC_IOCTL_GIVE_OSD_NAME_CMD,
   CEC_IOCTL_GIVE_PHY_ADDR_CMD,
   CEC_IOCTL_GIVE_SYS_AUDIO_MODE_STATUS_CMD,
   CEC_IOCTL_GIVE_TUNER_DEVICE_STATUS_CMD,
   CEC_IOCTL_IMAGE_VIEW_ON_CMD,
   CEC_IOCTL_INACTIVE_SRC_CMD,
   CEC_IOCTL_MENU_REQUEST_CMD,
   CEC_IOCTL_MENU_STATUS_CMD,
   CEC_IOCTL_PLAY_CMD,
   CEC_IOCTL_POLLING_MSG_CMD,
   CEC_IOCTL_REC_OFF_CMD,
   CEC_IOCTL_REC_ON_ANALOGUE_SERVICE_CMD,
   CEC_IOCTL_REC_ON_DIGITAL_SERVICE_CMD,
   CEC_IOCTL_REC_ON_EXT_PHY_ADDR_CMD,
   CEC_IOCTL_REC_ON_EXT_PLUG_CMD,
   CEC_IOCTL_REC_ON_OWN_SRC_CMD,
   CEC_IOCTL_REC_STATUS_CMD,
   CEC_IOCTL_REC_TV_SCREEN_CMD,
   CEC_IOCTL_REPORT_AUDIO_STATUS_CMD,
   CEC_IOCTL_REPORT_PHY_ADDR_CMD,
   CEC_IOCTL_REPORT_POWER_STATUS_CMD,
   CEC_IOCTL_REQUEST_ACTIVE_SRC_CMD,
   CEC_IOCTL_ROUTING_CHANGE_CMD,
   CEC_IOCTL_ROUTING_INFORMATION_CMD,
   CEC_IOCTL_SELECT_ANALOGUE_SERVICE_CMD,
   CEC_IOCTL_SELECT_DIGITAL_SERVICE_CMD,
   CEC_IOCTL_SET_ANALOGUE_TIMER_CMD,
   CEC_IOCTL_SET_AUDIO_RATE_CMD,
   CEC_IOCTL_SET_DIGITAL_TIMER_CMD,
   CEC_IOCTL_SET_EXT_TIMER_WITH_EXT_PLUG_CMD,
   CEC_IOCTL_SET_EXT_TIMER_WITH_PHY_ADDR_CMD,
   CEC_IOCTL_SET_MENU_LANGUAGE_CMD,
   CEC_IOCTL_SET_OSD_NAME_CMD,
   CEC_IOCTL_SET_OSD_STRING_CMD,
   CEC_IOCTL_SET_STREAM_PATH_CMD,
   CEC_IOCTL_SET_SYS_AUDIO_MODE_CMD,
   CEC_IOCTL_SET_TIMER_PROGRAM_TITLE_CMD,
   CEC_IOCTL_STANDBY_CMD,
   CEC_IOCTL_SYS_AUDIO_MODE_REQUEST_CMD,
   CEC_IOCTL_SYS_AUDIO_MODE_STATUS_CMD,
   CEC_IOCTL_TEXT_VIEW_ON_CMD,
   CEC_IOCTL_TIMER_CLEARED_STATUS_CMD,
   CEC_IOCTL_TIMER_STATUS_CMD,
   CEC_IOCTL_TUNER_DEVICE_STATUS_ANALOGUE_CMD,
   CEC_IOCTL_TUNER_DEVICE_STATUS_DIGITAL_CMD,
   CEC_IOCTL_TUNER_STEP_DECREMENT_CMD,
   CEC_IOCTL_TUNER_STEP_INCREMENT_CMD,
   CEC_IOCTL_USER_CTRL_CMD,
   CEC_IOCTL_USER_CTRL_PLAY_CMD,
   CEC_IOCTL_USER_CTRL_SELECT_AUDIOINPUT_CMD,
   CEC_IOCTL_USER_CTRL_SELECT_AVINPUT_CMD,
   CEC_IOCTL_USER_CTRL_SELECT_MEDIA_CMD,
   CEC_IOCTL_USER_CTRL_TUNE_CMD,
   CEC_IOCTL_USER_CTRL_RELEASED_CMD,
   CEC_IOCTL_VENDOR_COMMAND_CMD,
   CEC_IOCTL_VENDOR_COMMAND_WITH_ID_CMD,
   CEC_IOCTL_VENDOR_REMOTE_BUTTON_DOWN_CMD,
   CEC_IOCTL_VENDOR_REMOTE_BUTTON_UP_CMD,
   CEC_IOCTL_GET_SW_VERSION_CMD,
   CEC_IOCTL_SET_POWER_STATE_CMD,
   CEC_IOCTL_GET_POWER_STATE_CMD,
   CEC_IOCTL_INSTANCE_CONFIG_CMD,
   CEC_IOCTL_INSTANCE_SETUP_CMD,
   CEC_IOCTL_GET_INSTANCE_SETUP_CMD,
   CEC_IOCTL_ENABLE_EVENT_CMD,
   CEC_IOCTL_DISABLE_EVENT_CMD,
   CEC_IOCTL_ENABLE_CALIBRATION_CMD,
   CEC_IOCTL_DISABLE_CALIBRATION_CMD,
   CEC_IOCTL_SEND_MSG_CMD,
   CEC_IOCTL_SET_REGISTER_CMD
};

#define TDA_IOCTL_BASE 0x40
#define RELEASE 0xFF


enum {
   /* driver specific */
   TDA_VERBOSE_ON_CMD = 0,
   TDA_VERBOSE_OFF_CMD,
   TDA_BYEBYE_CMD,
   /* HDMI Tx */
   TDA_GET_SW_VERSION_CMD,
   TDA_SET_POWER_CMD,
};

#define TDA_IOCTL_SET_POWER _IOWR(TDA_IOCTL_BASE, TDA_SET_POWER_CMD,tda_power)


typedef enum tmPowerState
{
    tmPowerOn,                          // Device powered on      (D0 state)
    tmPowerStandby,                     // Device power standby   (D1 state)
    tmPowerSuspend,                     // Device power suspended (D2 state)
    tmPowerOff                          // Device powered off     (D3 state)

}   tmPowerState_t, *ptmPowerState_t;

typedef tmPowerState_t cec_power;
typedef tmPowerState_t tda_power;

typedef struct tmSWVersion
{
    unsigned int      compatibilityNr;        // Interface compatibility number
    unsigned int  majorVersionNr;         // Interface major version number
    unsigned int   minorVersionNr;         // Interface minor version number

}   tmSWVersion_t, *ptmSWVersion_t;

typedef tmSWVersion_t cec_sw_version;

typedef enum
{
   CEC_LOGICAL_ADDRESS_TV                     = 0,  /*!< TV                    */
   CEC_LOGICAL_ADDRESS_RECORDING_DEVICE_1     = 1,  /*!< Recording Device 1    */
   CEC_LOGICAL_ADDRESS_RECORDING_DEVICE_2     = 2,  /*!< Recording Device 1    */
   CEC_LOGICAL_ADDRESS_TUNER_1                = 3,  /*!< Tuner 1               */
   CEC_LOGICAL_ADDRESS_PLAYBACK_DEVICE_1      = 4,  /*!< Playback Device 1     */
} tmdlHdmiCECLogicalAddress_t;

 typedef enum
 {
     TMDL_HDMICEC_CLOCK_XTAL,
     TMDL_HDMICEC_CLOCK_FRO,
     TMDL_HDMICEC_CLOCK_PCLK
 } tmdlHdmiCecClockSource_t;

typedef struct _tmdlHdmiCecInstanceSetup_t
{
    tmdlHdmiCECLogicalAddress_t DeviceLogicalAddress;
    tmdlHdmiCecClockSource_t    cecClockSource;
//  tmdlHdmiCECDeviceState_t    DeviceState;
} tmdlHdmiCecInstanceSetup_t, *ptmdlHdmiCecInstanceSetup_t;
      typedef tmdlHdmiCecInstanceSetup_t cec_setup;


#define CEC_IOCTL_VERBOSE_ON _IO(CEC_IOCTL_BASE, CEC_VERBOSE_ON_CMD)
#define CEC_IOCTL_VERBOSE_OFF _IO(CEC_IOCTL_BASE, CEC_VERBOSE_OFF_CMD)
#define CEC_IOCTL_BYEBYE _IO(CEC_IOCTL_BASE, CEC_BYEBYE_CMD)

#define CEC_IOCTL_RX_ADDR      _IOWR(CEC_IOCTL_BASE,CEC_IOCTL_RX_ADDR_CMD,unsigned long)

#define CEC_IOCTL_WAIT_FRAME      _IOWR(CEC_IOCTL_BASE,CEC_IOCTL_WAIT_FRAME_CMD,cec_frame)
#define CEC_IOCTL_VERSION      _IOWR(CEC_IOCTL_BASE,CEC_IOCTL_VERSION_CMD,cec_version)
#define CEC_IOCTL_POLLING_MSG      _IO(CEC_IOCTL_BASE,CEC_IOCTL_POLLING_MSG_CMD)
#define CEC_IOCTL_DEVICE_VENDOR_ID      _IOWR(CEC_IOCTL_BASE,CEC_IOCTL_DEVICE_VENDOR_ID_CMD,unsigned long)
#define CEC_IOCTL_GIVE_OSD_NAME      _IO(CEC_IOCTL_BASE,CEC_IOCTL_GIVE_OSD_NAME_CMD)
#define CEC_IOCTL_SET_POWER_STATE      _IOWR(CEC_IOCTL_BASE,CEC_IOCTL_SET_POWER_STATE_CMD,cec_power)

#define CEC_IOCTL_ACTIVE_SRC      _IO(CEC_IOCTL_BASE,CEC_IOCTL_ACTIVE_SRC_CMD)
#define CEC_IOCTL_GET_SW_VERSION      _IOWR(CEC_IOCTL_BASE,CEC_IOCTL_GET_SW_VERSION_CMD,cec_sw_version)
#define CEC_IOCTL_GIVE_PHY_ADDR      _IO(CEC_IOCTL_BASE,CEC_IOCTL_GIVE_PHY_ADDR_CMD)
#define CEC_IOCTL_INSTANCE_SETUP      _IOWR(CEC_IOCTL_BASE,CEC_IOCTL_INSTANCE_SETUP_CMD,cec_setup)

bool CioctlCECAdapterCommunication::CheckAdapter(uint32_t iTimeoutMs /* = 10000 */)
{
  bool bReturn(false);
  uint64_t iNow = GetTimeMs();
  uint64_t iTarget = iTimeoutMs > 0 ? iNow + iTimeoutMs : iNow + CEC_DEFAULT_TRANSMIT_WAIT;


  /* try to ping the adapter */
  bool bPinged(false);
  unsigned iPingTry(0);
  while (iNow < iTarget && (bPinged = PingAdapter()) == false)
  {
    CLibCEC::AddLog(CEC_LOG_ERROR, "the adapter did not respond correctly to a ping (try %d)", ++iPingTry);
    CEvent::Sleep(500);
    iNow = GetTimeMs();
  }

  /* try to read the firmware version */
  m_iFirmwareVersion = CEC_FW_VERSION_UNKNOWN;
  unsigned iFwVersionTry(0);
  while (bPinged && iNow < iTarget && (m_iFirmwareVersion = GetFirmwareVersion()) == CEC_FW_VERSION_UNKNOWN && iFwVersionTry < 3)
  {
    CLibCEC::AddLog(CEC_LOG_WARNING, "the adapter did not respond with a correct firmware version (try %d)", ++iFwVersionTry);
    CEvent::Sleep(500);
    iNow = GetTimeMs();
  }

  if (m_iFirmwareVersion == CEC_FW_VERSION_UNKNOWN)
  {
    CLibCEC::AddLog(CEC_LOG_DEBUG, "defaulting to firmware version 1");
    m_iFirmwareVersion = 1;
  }

  if (m_iFirmwareVersion >= 2)
  {
    /* try to set controlled mode */
    unsigned iControlledTry(0);
    bool bControlled(false);
    while (iNow < iTarget && (bControlled = SetControlledMode(true)) == false)
    {
      CLibCEC::AddLog(CEC_LOG_ERROR, "the adapter did not respond correctly to setting controlled mode (try %d)", ++iControlledTry);
      CEvent::Sleep(500);
      iNow = GetTimeMs();
    }
    bReturn = bControlled;
  }
  else
    bReturn = true;

  {
    CLockObject lock(m_mutex);
    m_bInitialised = bReturn;
  }

  return bReturn;
}

bool CioctlCECAdapterCommunication::Open(IAdapterCommunicationCallback *cb, uint32_t iTimeoutMs /* = 10000 */, bool bSkipChecks /* = false */, bool bStartListening /* = true */)
{
  uint64_t iNow = GetTimeMs();
  uint64_t iTimeout = iNow + iTimeoutMs;

  fprintf(stderr, "*** IOCTL Open\r\n");

  {
    CLockObject lock(m_mutex);

    if (!m_port)
    {
      CLibCEC::AddLog(CEC_LOG_ERROR, "port is NULL");
      return false;
    }

    if (IsOpen())
    {
      CLibCEC::AddLog(CEC_LOG_ERROR, "port is already open");
      return true;
    }

    {
      int fd;
      tda_power power;
      fd = open("/dev/hdmitx", O_RDWR);
      if (fd >=0) {
        power = tmPowerOn;
        fprintf(stderr, "Call TDA_IOCTL_SET_POWER (On) : %d\r\n",
                ioctl(fd, TDA_IOCTL_SET_POWER, &power));
      }
    }



    m_callback = cb;
    CStdString strError;
    bool bConnected(false);
    while (!bConnected && iNow < iTimeout)
    {
      if ((bConnected = m_port->Open(iTimeout)) == false)
      {
        strError.Format("error opening device node '%s': %s", m_port->GetName().c_str(), m_port->GetError().c_str());
        Sleep(250);
        iNow = GetTimeMs();
      }
    }

    if (!bConnected)
    {
      CLibCEC::AddLog(CEC_LOG_ERROR, strError);
      return false;
    }

    CLibCEC::AddLog(CEC_LOG_DEBUG, "connection opened, clearing any previous input and waiting for active transmissions to end before starting");

    if (0) {
      fprintf(stderr, "Setting verbose: %d\r\n",
              m_port->Ioctl(CEC_IOCTL_VERBOSE_OFF,NULL));
    }

  //unsigned long player = CEC_LOGICAL_ADDRESS_PLAYBACK_DEVICE_1;

    // m_port->Ioctl(CEC_IOCTL_RX_ADDR, (void *)&player);


    //Lund
    {
      int i;
      cec_power power;
      cec_sw_version vers;
      unsigned long rx_addr;
      int fd;
      memset(&vers, 0xAA, sizeof(vers));

      power = tmPowerOn;

      // Apparently we need to call instance_setup first, or other calls fail.
      cec_setup setup;
      setup.DeviceLogicalAddress = CEC_LOGICAL_ADDRESS_PLAYBACK_DEVICE_1;
      // What clock to use? XTAL, FRO or PCLK
      setup.cecClockSource = TMDL_HDMICEC_CLOCK_XTAL;
      fprintf(stderr, "Call CEC_IOCTL_INSTANCE_SETUP : %d\r\n",
              m_port->Ioctl(CEC_IOCTL_INSTANCE_SETUP,&setup));

      fprintf(stderr, "Call CEC_IOCTL_GET_SW_VERSION_CMD : %d\r\n",
              m_port->Ioctl(CEC_IOCTL_GET_SW_VERSION,&vers));
      fprintf(stderr, "Version is comp: %08X, major: %08X, minor: %08X\r\n",
              vers.compatibilityNr, vers.majorVersionNr, vers.minorVersionNr);

      //fprintf(stderr, "Get CEC_IOCTL_BYEBYE: %d\r\n",
      //m_port->Ioctl(CEC_IOCTL_BYEBYE,NULL));
      //fprintf(stderr, "Call CEC_IOCTL_ACTIVE_SOURCE: %d\r\n",
      //       m_port->Ioctl(CEC_IOCTL_ACTIVE_SRC,NULL));
      //rx_addr = CEC_LOGICAL_ADDRESS_PLAYBACK_DEVICE_1;
      //fprintf(stderr, "Call CEC_IOCTL_RX_ADDR: %d\r\n",
      //        m_port->Ioctl(CEC_IOCTL_RX_ADDR,
      //                     (void *)rx_addr));
      power = tmPowerOn;
      fprintf(stderr, "Call CEC_IOCTL_SET_POWER_STATE (On) : %d\r\n",
              m_port->Ioctl(CEC_IOCTL_SET_POWER_STATE,&power));

      //fprintf(stderr, "Call CEC_IOCTL_GIVE_PHY_ADDR: %d\r\n",
      //       m_port->Ioctl(CEC_IOCTL_GIVE_PHY_ADDR, NULL));

    }

    // The drivers/video/display/nxp/comps/tmdlHdmiCEC/src/tmdlHdmiCEC.c
    // file say:
    // \brief This message is used by any device for device discovery - similar to
    // ping in other protocols
    if (!m_port->Ioctl(CEC_IOCTL_POLLING_MSG,NULL))
      fprintf(stderr, "** IOCTL enabled polling\r\n");



    bSkipChecks = true;
    if (!bSkipChecks)
    {
      //clear any input bytes
      uint8_t buff[1024];
      ssize_t iBytesRead(0);
      bool bGotMsgStart(false), bGotMsgEnd(false);
      while ((iBytesRead = m_port->Read(buff, 1024, 100)) > 0 || (bGotMsgStart && !bGotMsgEnd))
      {
        if (!bGotMsgStart)
          CLibCEC::AddLog(CEC_LOG_DEBUG, "data received, clearing it");
        // if something was received, wait for MSGEND
        for (ssize_t iPtr = 0; iPtr < iBytesRead; iPtr++)
        {
          if (buff[iPtr] == MSGSTART)
            bGotMsgStart = true;
          else if (buff[iPtr] == MSGEND)
            bGotMsgEnd = true;
        }
        Sleep(250);
      }
    }
  }

  bSkipChecks = false;
  if (!bSkipChecks && !CheckAdapter())
  {
    CLibCEC::AddLog(CEC_LOG_ERROR, "the adapter failed to pass basic checks");
    delete m_port;
    m_port = NULL;
    return false;
  }
  else if (bStartListening)
  {
    m_pingThread = new CioctlAdapterPingThread(this, CEC_ADAPTER_PING_TIMEOUT);
    if (CreateThread() && m_pingThread->CreateThread())
    {
      CLibCEC::AddLog(CEC_LOG_DEBUG, "communication thread started");
      return true;
    }
    else
    {
      delete m_port;
      m_port = NULL;
      CLibCEC::AddLog(CEC_LOG_ERROR, "could not create a communication thread");
      return false;
    }
  }
  else
  {
    delete m_port;
    m_port = NULL;
  }

  fprintf(stderr, "** IOCTL port is still %p\r\n", m_port);
  return true;
}

void CioctlCECAdapterCommunication::Close(void)
{
  if (m_pingThread)
    m_pingThread->StopThread(0);
  delete m_pingThread;
  m_pingThread = NULL;
  StopThread(0);
}

void *CioctlCECAdapterCommunication::Process(void)
{
  cec_command command;
  command.Clear();
  bool bCommandReceived(false);
  while (!IsStopped())
  {
    {
      CLockObject lock(m_mutex);
      bCommandReceived = m_callback &&
        Read(command, 0)
        && m_bInitialised;
    }

    fprintf(stderr, "*** IOCTL yay,. received frame! \r\n");

    /* push the next command to the callback method if there is one */
    if (!IsStopped() && bCommandReceived)
      m_callback->OnCommandReceived(command);

    Sleep(5);
  }


  fprintf(stderr, "** IOCTL reading aborting\r\n");
  /* set the ackmask to 0 before closing the connection */
  SetAckMaskInternal(0, true);

  if (m_iFirmwareVersion >= 2)
    SetControlledMode(false);

  if (m_port)
  {
    delete m_port;
    m_port = NULL;
  }

  m_rcvCondition.Broadcast();
  return NULL;
}

cec_adapter_message_state CioctlCECAdapterCommunication::Write(const cec_command &data, uint8_t iMaxTries, uint8_t iLineTimeout /* = 3 */, uint8_t iRetryLineTimeout /* = 3 */)
{
  cec_adapter_message_state retVal(ADAPTER_MESSAGE_STATE_UNKNOWN);
  if (!IsRunning())
    return retVal;

  CCECAdapterMessage *output = new CCECAdapterMessage(data);

  /* set the number of retries */
  if (data.opcode == CEC_OPCODE_NONE) //TODO
    output->maxTries = 1;
  else if (data.initiator != CECDEVICE_BROADCAST)
    output->maxTries = iMaxTries;

  output->lineTimeout = iLineTimeout;
  output->retryTimeout = iRetryLineTimeout;
  output->tries = 0;

  if (data.destination < 15)
  {
    CLockObject lock(m_mutex);
    m_bWaitingForAck[data.destination] = true;
  }

  bool bRetry(true);
  while (bRetry && ++output->tries < output->maxTries)
  {
    bRetry = (!Write(output) || output->NeedsRetry()) && output->transmit_timeout > 0;
    if (bRetry)
      Sleep(CEC_DEFAULT_TRANSMIT_RETRY_WAIT);
  }
  retVal = output->state;

  delete output;
  return retVal;
}

bool CioctlCECAdapterCommunication::Write(CCECAdapterMessage *data)
{
  data->state = ADAPTER_MESSAGE_STATE_WAITING_TO_BE_SENT;
  SendMessageToAdapter(data);

  if ((data->expectControllerAck && data->state != ADAPTER_MESSAGE_STATE_SENT_ACKED) ||
      (!data->expectControllerAck && data->state != ADAPTER_MESSAGE_STATE_SENT))
  {
    CLibCEC::AddLog(CEC_LOG_DEBUG, "command was not %s", data->state == ADAPTER_MESSAGE_STATE_SENT_NOT_ACKED ? "acked" : "sent");
    return false;
  }

  return true;
}

bool CioctlCECAdapterCommunication::Read(cec_command &command, uint32_t iTimeout)
{
  if (!IsRunning())
    return false;

  CCECAdapterMessage msg;
  if (Read(msg, iTimeout))
  {
    if (ParseMessage(msg))
    {
      command = m_currentframe;
      m_currentframe.Clear();
      return true;
    }
  }
  return false;
}

bool CioctlCECAdapterCommunication::Read(CCECAdapterMessage &msg, uint32_t iTimeout, size_t iLen)
{
  CLockObject lock(m_mutex);
  ReadFromDevice(iTimeout, iLen);

  msg.Clear();
  CCECAdapterMessage *buf(NULL);

  if (!m_inBuffer.Pop(buf))
  {
    if (iTimeout == 0 || !m_rcvCondition.Wait(m_mutex, m_bHasData, iTimeout))
      return false;
    m_inBuffer.Pop(buf);
    m_bHasData = !m_inBuffer.IsEmpty();
  }

  if (buf)
  {
    msg.packet = buf->packet;
    msg.state = ADAPTER_MESSAGE_STATE_INCOMING;
    delete buf;
    return true;
  }
  return false;
}

CStdString CioctlCECAdapterCommunication::GetError(void) const
{
  CStdString strError;
  strError = m_port->GetError();
  return strError;
}

bool CioctlCECAdapterCommunication::StartBootloader(void)
{
  bool bReturn(false);
  if (!IsRunning())
    return bReturn;

  CLibCEC::AddLog(CEC_LOG_DEBUG, "starting the bootloader");

  CCECAdapterMessage params;
  return SendCommand(MSGCODE_START_BOOTLOADER, params, false);
}

bool CioctlCECAdapterCommunication::PingAdapter(void)
{
  CLibCEC::AddLog(CEC_LOG_DEBUG, "sending ping");

  CCECAdapterMessage params;

  cec_version version;
  /* Basic ioctl test */
  if (m_port->Ioctl(CEC_IOCTL_VERSION, &version) < 0)
  {
    CLibCEC::AddLog(CEC_LOG_ERROR, "the adapter did not respond correctly to ioctl ");
  }
  fprintf(stderr, "CEC version returned is %X\r\n", version);
  return true;

  //return SendCommand(MSGCODE_PING, params);
}

bool CioctlCECAdapterCommunication::ParseMessage(const CCECAdapterMessage &msg)
{
  bool bEom(false);
  bool bIsError(msg.IsError());

  if (msg.IsEmpty())
    return bEom;

  CLockObject adapterLock(m_mutex);
  switch(msg.Message())
  {
  case MSGCODE_FRAME_START:
    {
      m_currentframe.Clear();
      if (msg.Size() >= 2)
      {
        m_currentframe.initiator   = msg.Initiator();
        m_currentframe.destination = msg.Destination();
        m_currentframe.ack         = msg.IsACK();
        m_currentframe.eom         = msg.IsEOM();
      }
      if (m_currentframe.ack == 0x1)
      {
        m_lastDestination    = m_currentframe.destination;
        if (m_currentframe.destination < 15)
        {
          if (!m_bWaitingForAck[m_currentframe.destination])
            m_processor->HandlePoll(m_currentframe.initiator, m_currentframe.destination);
          else
            m_bWaitingForAck[m_currentframe.destination] = false;
        }
      }
    }
    break;
  case MSGCODE_RECEIVE_FAILED:
    {
      m_currentframe.Clear();
      if (m_lastDestination != CECDEVICE_UNKNOWN)
        bIsError = m_processor->HandleReceiveFailed(m_lastDestination);
    }
    break;
  case MSGCODE_FRAME_DATA:
    {
      if (msg.Size() >= 2)
      {
        m_currentframe.PushBack(msg[1]);
        m_currentframe.eom = msg.IsEOM();
      }
    }
    break;
  default:
    break;
  }

  CLibCEC::AddLog(bIsError ? CEC_LOG_WARNING : CEC_LOG_DEBUG, msg.ToString());
  return msg.IsEOM();
}

uint16_t CioctlCECAdapterCommunication::GetFirmwareVersion(void)
{
  uint16_t iReturn(m_iFirmwareVersion);

  if (iReturn == CEC_FW_VERSION_UNKNOWN)
  {
    CLockObject lock(m_mutex);
    CLibCEC::AddLog(CEC_LOG_DEBUG, "requesting the firmware version.");
    cec_datapacket response = GetSetting(MSGCODE_FIRMWARE_VERSION, 2);
    if (response.size == 2)
    {
      m_iFirmwareVersion = (response[0] << 8 | response[1]);
      iReturn = m_iFirmwareVersion;
      CLibCEC::AddLog(CEC_LOG_DEBUG, "firmware version %d", m_iFirmwareVersion);
    }
  }

  return iReturn;
}

bool CioctlCECAdapterCommunication::SetLineTimeout(uint8_t iTimeout)
{
  bool bReturn(true);

  if (m_iLineTimeout != iTimeout)
  {
    CLibCEC::AddLog(CEC_LOG_DEBUG, "setting the line timeout to %d", iTimeout);
    CCECAdapterMessage params;
    params.PushEscaped(iTimeout);
    bReturn = SendCommand(MSGCODE_TRANSMIT_IDLETIME, params);
    if (bReturn)
      m_iLineTimeout = iTimeout;
  }

  return bReturn;
}

bool CioctlCECAdapterCommunication::SetAckMask(uint16_t iMask)
{
  return SetAckMaskInternal(iMask, IsRunning());
}

bool CioctlCECAdapterCommunication::SetAckMaskInternal(uint16_t iMask, bool bWriteDirectly /* = false */)
{
  CLibCEC::AddLog(CEC_LOG_DEBUG, "setting ackmask to %2x", iMask);

  CCECAdapterMessage params;
  params.PushEscaped(iMask >> 8);
  params.PushEscaped((uint8_t)iMask);
  return SendCommand(MSGCODE_SET_ACK_MASK, params, true, false, bWriteDirectly);
}

bool CioctlCECAdapterCommunication::PersistConfiguration(libcec_configuration *configuration)
{
  if (m_iFirmwareVersion < 2)
    return false;

  bool bReturn(true);
  bReturn &= SetSettingAutoEnabled(true);
  bReturn &= SetSettingDeviceType(CLibCEC::GetType(configuration->logicalAddresses.primary));
  bReturn &= SetSettingDefaultLogicalAddress(configuration->logicalAddresses.primary);
  bReturn &= SetSettingLogicalAddressMask(CLibCEC::GetMaskForType(configuration->logicalAddresses.primary));
  bReturn &= SetSettingPhysicalAddress(configuration->iPhysicalAddress);
  bReturn &= SetSettingCECVersion(CEC_VERSION_1_3A);
  bReturn &= SetSettingOSDName(configuration->strDeviceName);
  if (bReturn)
    bReturn = WriteEEPROM();
  return bReturn;
}

bool CioctlCECAdapterCommunication::GetConfiguration(libcec_configuration *configuration)
{
  configuration->iFirmwareVersion = m_iFirmwareVersion;
  if (m_iFirmwareVersion < 2)
    return false;

  bool bReturn(true);
  cec_device_type type;
  if (GetSettingDeviceType(type))
  {
    CLibCEC::AddLog(CEC_LOG_DEBUG, "using persisted device type setting %s", m_processor->ToString(type));
    configuration->deviceTypes.Clear();
    configuration->deviceTypes.Add(type);
  }
  else
  {
    CLibCEC::AddLog(CEC_LOG_DEBUG, "no persisted device type setting");
    bReturn = false;
  }

  if (GetSettingPhysicalAddress(configuration->iPhysicalAddress))
  {
    CLibCEC::AddLog(CEC_LOG_DEBUG, "using persisted physical address setting %4x", configuration->iPhysicalAddress);
  }
  else
  {
    CLibCEC::AddLog(CEC_LOG_DEBUG, "no persisted physical address setting");
    bReturn = false;
  }

  CStdString strDeviceName;
  if (GetSettingOSDName(strDeviceName))
  {
    snprintf(configuration->strDeviceName, 13, "%s", strDeviceName.c_str());
    CLibCEC::AddLog(CEC_LOG_DEBUG, "using persisted device name setting %s", configuration->strDeviceName);
  }
  else
  {
    CLibCEC::AddLog(CEC_LOG_DEBUG, "no persisted device name setting");
    bReturn = false;
  }

  // don't read the following settings:
  // - auto enabled (always enabled)
  // - default logical address (autodetected)
  // - logical address mask (autodetected)
  // - CEC version (1.3a)

  // TODO to be added to the firmware:
  // - base device (4 bits)
  // - HDMI port number (4 bits)
  // - TV vendor id (12 bits)
  // - wake devices (8 bits)
  // - standby devices (8 bits)
  // - use TV menu language (1 bit)
  // - activate source (1 bit)
  // - power off screensaver (1 bit)
  // - power off on standby (1 bit)
  // - send inactive source (1 bit)
  return bReturn;
}

bool CioctlCECAdapterCommunication::SetControlledMode(bool controlled)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "turning controlled mode %s", controlled ? "on" : "off");

  CCECAdapterMessage params;
  params.PushEscaped(controlled ? 1 : 0);
  return SendCommand(MSGCODE_SET_CONTROLLED, params);
}

bool CioctlCECAdapterCommunication::SetSettingAutoEnabled(bool enabled)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "turning autonomous mode %s", enabled ? "on" : "off");

  CCECAdapterMessage params;
  params.PushEscaped(enabled ? 1 : 0);
  return SendCommand(MSGCODE_SET_AUTO_ENABLED, params);
}

bool CioctlCECAdapterCommunication::GetSettingAutoEnabled(bool &enabled)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "requesting autonomous mode setting");

  cec_datapacket response = GetSetting(MSGCODE_GET_AUTO_ENABLED, 1);
  if (response.size == 1)
  {
    enabled = response[0] == 1;
    return true;
  }
  return false;
}

bool CioctlCECAdapterCommunication::SetSettingDeviceType(cec_device_type type)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "setting the device type to %1X", (uint8_t)type);

  CCECAdapterMessage params;
  params.PushEscaped((uint8_t)type);
  return SendCommand(MSGCODE_SET_DEVICE_TYPE, params);
}

bool CioctlCECAdapterCommunication::GetSettingDeviceType(cec_device_type &value)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "requesting device type setting");

  cec_datapacket response = GetSetting(MSGCODE_GET_DEVICE_TYPE, 1);
  if (response.size == 1)
  {
    value = (cec_device_type)response[0];
    return true;
  }
  return false;
}

bool CioctlCECAdapterCommunication::SetSettingDefaultLogicalAddress(cec_logical_address address)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "setting the default logical address to %1X", address);

  CCECAdapterMessage params;
  params.PushEscaped((uint8_t)address);
  return SendCommand(MSGCODE_SET_DEFAULT_LOGICAL_ADDRESS, params);
}

bool CioctlCECAdapterCommunication::GetSettingDefaultLogicalAddress(cec_logical_address &address)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "requesting default logical address setting");

  cec_datapacket response = GetSetting(MSGCODE_GET_DEFAULT_LOGICAL_ADDRESS, 1);
  if (response.size == 1)
  {
    address = (cec_logical_address)response[0];
    return true;
  }
  return false;
}

bool CioctlCECAdapterCommunication::SetSettingLogicalAddressMask(uint16_t iMask)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "setting the logical address mask to %2X", iMask);

  CCECAdapterMessage params;
  params.PushEscaped(iMask >> 8);
  params.PushEscaped((uint8_t)iMask);
  return SendCommand(MSGCODE_SET_LOGICAL_ADDRESS_MASK, params);
}

bool CioctlCECAdapterCommunication::GetSettingLogicalAddressMask(uint16_t &iMask)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "requesting logical address mask setting");

  cec_datapacket response = GetSetting(MSGCODE_GET_LOGICAL_ADDRESS_MASK, 2);
  if (response.size == 2)
  {
    iMask = ((uint16_t)response[0] << 8) | ((uint16_t)response[1]);
    return true;
  }
  return false;
}

bool CioctlCECAdapterCommunication::SetSettingPhysicalAddress(uint16_t iPhysicalAddress)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "setting the physical address to %04X", iPhysicalAddress);

  CCECAdapterMessage params;
  params.PushEscaped(iPhysicalAddress >> 8);
  params.PushEscaped((uint8_t)iPhysicalAddress);
  return SendCommand(MSGCODE_SET_PHYSICAL_ADDRESS, params);
}

bool CioctlCECAdapterCommunication::GetSettingPhysicalAddress(uint16_t &iPhysicalAddress)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "requesting physical address setting");

  cec_datapacket response = GetSetting(MSGCODE_GET_PHYSICAL_ADDRESS, 2);
  if (response.size == 2)
  {
    iPhysicalAddress = ((uint16_t)response[0] << 8) | ((uint16_t)response[1]);
    return true;
  }
  return false;
}

bool CioctlCECAdapterCommunication::SetSettingCECVersion(cec_version version)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "setting the CEC version to %s", CLibCEC::GetInstance()->ToString(version));

  CCECAdapterMessage params;
  params.PushEscaped((uint8_t)version);
  return SendCommand(MSGCODE_SET_HDMI_VERSION, params);
}

bool CioctlCECAdapterCommunication::GetSettingCECVersion(cec_version &version)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "requesting CEC version setting");

  cec_datapacket response = GetSetting(MSGCODE_GET_HDMI_VERSION, 1);
  if (response.size == 1)
  {
    version = (cec_version)response[0];
    return true;
  }
  return false;
}

bool CioctlCECAdapterCommunication::SetSettingOSDName(const char *strOSDName)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "setting the OSD name to %s", strOSDName);

  CCECAdapterMessage params;
  for (size_t iPtr = 0; iPtr < strlen(strOSDName); iPtr++)
    params.PushEscaped(strOSDName[iPtr]);
  return SendCommand(MSGCODE_SET_OSD_NAME, params);
}

bool CioctlCECAdapterCommunication::GetSettingOSDName(CStdString &strOSDName)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "requesting OSD name setting");

  cec_datapacket response = GetSetting(MSGCODE_GET_OSD_NAME, 13);
  if (response.size == 0)
    return false;

  char buf[14];
  for (uint8_t iPtr = 0; iPtr < response.size && iPtr < 13; iPtr++)
    buf[iPtr] = (char)response[iPtr];
  buf[response.size] = 0;

  strOSDName.Format("%s", buf);
  return true;
}

bool CioctlCECAdapterCommunication::WriteEEPROM(void)
{
  CLockObject lock(m_mutex);
  CLibCEC::AddLog(CEC_LOG_DEBUG, "writing settings in the EEPROM");

  CCECAdapterMessage params;
  return SendCommand(MSGCODE_WRITE_EEPROM, params);
}

bool CioctlCECAdapterCommunication::IsOpen(void)
{
  return !IsStopped() && m_port->IsOpen() && IsRunning();
}

bool CioctlCECAdapterCommunication::WaitForAck(CCECAdapterMessage &message)
{
  bool bError(false);
  bool bTransmitSucceeded(false);
  uint8_t iPacketsLeft(message.isTransmission ? message.Size() / 4 : 1);

  int64_t iNow = GetTimeMs();
  int64_t iTargetTime = iNow + (message.transmit_timeout <= 5 ? CEC_DEFAULT_TRANSMIT_WAIT : message.transmit_timeout);

  while (!bTransmitSucceeded && !bError && iNow < iTargetTime)
  {
    CCECAdapterMessage msg;
    if (!Read(msg, 50))
    {
      iNow = GetTimeMs();
      continue;
    }

    if (msg.Message() == MSGCODE_FRAME_START && msg.IsACK())
    {
      if (msg.Initiator() < 15 && m_bWaitingForAck[msg.Initiator()])
        m_bWaitingForAck[msg.Initiator()] = false;
      else if (msg.Initiator() < 15)
      {
        m_processor->HandlePoll(msg.Initiator(), msg.Destination());
        m_lastDestination = msg.Initiator();
      }
      iNow = GetTimeMs();
      continue;
    }

    if (msg.Message() == MSGCODE_RECEIVE_FAILED &&
        m_lastDestination != CECDEVICE_UNKNOWN &&
        m_processor->HandleReceiveFailed(m_lastDestination))
    {
      iNow = GetTimeMs();
      continue;
    }

    bError = msg.IsError();
    if (bError)
    {
      message.reply = msg.Message();
      CLibCEC::AddLog(CEC_LOG_DEBUG, msg.ToString());
    }
    else
    {
      switch(msg.Message())
      {
      case MSGCODE_COMMAND_ACCEPTED:
        if (iPacketsLeft > 0)
          iPacketsLeft--;
        if (!message.isTransmission && iPacketsLeft == 0)
          bTransmitSucceeded = true;
        CLibCEC::AddLog(CEC_LOG_DEBUG, "%s - waiting for %d more", msg.ToString().c_str(), iPacketsLeft);
        break;
      case MSGCODE_TRANSMIT_SUCCEEDED:
        CLibCEC::AddLog(CEC_LOG_DEBUG, msg.ToString());
        bTransmitSucceeded = (iPacketsLeft == 0);
        bError = !bTransmitSucceeded;
        message.reply = MSGCODE_TRANSMIT_SUCCEEDED;
        break;
      default:
        // ignore other data while waiting
        break;
      }

      iNow = GetTimeMs();
    }
  }

  message.state = bTransmitSucceeded && !bError ?
      ADAPTER_MESSAGE_STATE_SENT_ACKED :
      ADAPTER_MESSAGE_STATE_SENT_NOT_ACKED;

  return bTransmitSucceeded && !bError;
}

void CioctlCECAdapterCommunication::AddData(uint8_t *data, size_t iLen)
{
  CLockObject lock(m_mutex);
  for (size_t iPtr = 0; iPtr < iLen; iPtr++)
  {
    if (!m_bGotStart)
    {
      if (data[iPtr] == MSGSTART)
        m_bGotStart = true;
    }
    else if (data[iPtr] == MSGSTART) //we found a msgstart before msgend, this is not right, remove
    {
      if (m_currentAdapterMessage.Size() > 0)
        CLibCEC::AddLog(CEC_LOG_WARNING, "received MSGSTART before MSGEND, removing previous buffer contents");
      m_currentAdapterMessage.Clear();
      m_bGotStart = true;
    }
    else if (data[iPtr] == MSGEND)
    {
      CCECAdapterMessage *newMessage = new CCECAdapterMessage;
      newMessage->packet = m_currentAdapterMessage.packet;
      m_inBuffer.Push(newMessage);
      m_currentAdapterMessage.Clear();
      m_bGotStart = false;
      m_bNextIsEscaped = false;
      m_bHasData = true;
      m_rcvCondition.Broadcast();
    }
    else if (m_bNextIsEscaped)
    {
      m_currentAdapterMessage.PushBack(data[iPtr] + (uint8_t)ESCOFFSET);
      m_bNextIsEscaped = false;
    }
    else if (data[iPtr] == MSGESC)
    {
      m_bNextIsEscaped = true;
    }
    else
    {
      m_currentAdapterMessage.PushBack(data[iPtr]);
    }
  }
}

bool CioctlCECAdapterCommunication::ReadFromDevice(uint32_t iTimeout, size_t iSize /* = 256 */)
{
  ssize_t iBytesRead;
  uint8_t buff[256];
  if (!m_port)
    return false;
  if (iSize > 256)
    iSize = 256;

  CLockObject lock(m_mutex);

  iBytesRead = m_port->Read(buff, sizeof(uint8_t) * iSize, iTimeout);

  if (iBytesRead < 0 || iBytesRead > 256)
  {
    CLibCEC::AddLog(CEC_LOG_ERROR, "error reading from serial port: %s", m_port->GetError().c_str());
    StopThread(false);
    return false;
  }
  else if (iBytesRead > 0)
  {
    AddData(buff, iBytesRead);
  }

  return iBytesRead > 0;
}

void CioctlCECAdapterCommunication::SendMessageToAdapter(CCECAdapterMessage *msg)
{
  CLockObject adapterLock(m_mutex);
  if (!m_port->IsOpen())
  {
    CLibCEC::AddLog(CEC_LOG_ERROR, "error writing to serial port: the connection is closed");
    msg->state = ADAPTER_MESSAGE_STATE_ERROR;
    return;
  }

  if (msg->isTransmission && (msg->Size() < 2 || msg->At(1) != MSGCODE_TRANSMIT_IDLETIME))
  {
    if (msg->tries == 1)
      SetLineTimeout(msg->lineTimeout);
    else
      SetLineTimeout(msg->retryTimeout);
  }

  if (m_port->Write(msg->packet.data, msg->Size()) != (ssize_t) msg->Size())
  {
    CLibCEC::AddLog(CEC_LOG_ERROR, "error writing to serial port: %s", m_port->GetError().c_str());
    msg->state = ADAPTER_MESSAGE_STATE_ERROR;
  }
  else
  {
    CLibCEC::AddLog(CEC_LOG_DEBUG, "command sent");
    msg->state = ADAPTER_MESSAGE_STATE_SENT;

    if (msg->expectControllerAck)
    {
      if (!WaitForAck(*msg))
        CLibCEC::AddLog(CEC_LOG_DEBUG, "did not receive ack");
    }
  }
}

CStdString CioctlCECAdapterCommunication::GetPortName(void)
{
  CStdString strName;
  strName = m_port->GetName();
  return strName;
}

bool CioctlCECAdapterCommunication::SendCommand(cec_adapter_messagecode msgCode, CCECAdapterMessage &params, bool bExpectAck /* = true */, bool bIsTransmission /* = false */, bool bSendDirectly /* = true */, bool bIsRetry /* = false */)
{
  CLockObject lock(m_mutex);

  CCECAdapterMessage *output = new CCECAdapterMessage;

  fprintf(stderr, "** IOCTL SendCommand %02X change me to ioctl\r\n",
          msgCode);

  output->PushBack(MSGSTART);
  output->PushEscaped((uint8_t)msgCode);
  output->Append(params);
  output->PushBack(MSGEND);
  output->isTransmission = bIsTransmission;
  output->expectControllerAck = bExpectAck;

  if (bSendDirectly)
    SendMessageToAdapter(output);
  else
    Write(output);

  bool bWriteOk = output->state == (output->expectControllerAck ? ADAPTER_MESSAGE_STATE_SENT_ACKED : ADAPTER_MESSAGE_STATE_SENT);
  cec_adapter_messagecode reply = output->reply;
  delete output;

  if (!bWriteOk)
  {
    CLibCEC::AddLog(CEC_LOG_ERROR, "'%s' failed", CCECAdapterMessage::ToString(msgCode));

    if (!bIsRetry && reply == MSGCODE_COMMAND_REJECTED && msgCode != MSGCODE_SET_CONTROLLED)
    {
      CLibCEC::AddLog(CEC_LOG_DEBUG, "setting controlled mode and retrying");
      if (SetControlledMode(true))
        return SendCommand(msgCode, params, bExpectAck, bIsTransmission, bSendDirectly, true);
    }
    return false;
  }

  return true;
}

cec_datapacket CioctlCECAdapterCommunication::GetSetting(cec_adapter_messagecode msgCode, uint8_t iResponseLength)
{
  cec_datapacket retVal;
  retVal.Clear();

  CCECAdapterMessage params;

#if 0
  if (!SendCommand(msgCode, params, false))
  {
    CLibCEC::AddLog(CEC_LOG_ERROR, "%s failed", CCECAdapterMessage::ToString(msgCode));
    return retVal;
  }
#endif

  // Lund
  switch(msgCode) {
  case MSGCODE_FIRMWARE_VERSION:
    {
      unsigned long vendor = 0x01234567;
      m_port->Ioctl(CEC_IOCTL_DEVICE_VENDOR_ID, &vendor);
      fprintf(stderr, "*** IOCTL vendor say %08X\r\n", vendor);
    }
    break;
  default:
    break;
  }


  CCECAdapterMessage input;
  if (Read(input, CEC_DEFAULT_TRANSMIT_WAIT, iResponseLength + 3 /* start + msgcode + iResponseLength + end */))
  {
    if (input.Message() != msgCode)
      CLibCEC::AddLog(CEC_LOG_ERROR, "invalid response to %s received (%s)", CCECAdapterMessage::ToString(msgCode), CCECAdapterMessage::ToString(input.Message()));
    else
    {
      for (uint8_t iPtr = 1; iPtr < input.Size(); iPtr++)
        retVal.PushBack(input[iPtr]);
    }
  }
  else
  {
    CLibCEC::AddLog(CEC_LOG_ERROR, "no response to %s received", CCECAdapterMessage::ToString(msgCode));
  }

  return retVal;
}

void *CioctlAdapterPingThread::Process(void)
{
  while (!IsStopped())
  {
    if (m_timeout.TimeLeft() == 0)
    {
      m_timeout.Init(CEC_ADAPTER_PING_TIMEOUT);
      m_com->PingAdapter();
    }

    Sleep(500);
  }
  return NULL;
}
