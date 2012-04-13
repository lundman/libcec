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

#include "ioctlCECAdapterDetection.h"
#include "../platform/util/StdString.h"

#if defined(__APPLE__)
#include <dirent.h>
#include <sys/param.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/IOMessage.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <CoreFoundation/CoreFoundation.h>
#elif defined(__WINDOWS__)
#pragma comment(lib, "advapi32.lib")
#pragma comment(lib, "setupapi.lib")
#include <setupapi.h>

// the virtual COM port only shows up when requesting devices with the raw device guid!
static GUID USB_RAW_GUID =  { 0xA5DCBF10, 0x6530, 0x11D2, { 0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED } };
#elif defined(HAVE_LIBUDEV)
#include <dirent.h>
#include <poll.h>
extern "C" {
#include <libudev.h>
}
#elif defined(__FreeBSD__)
#include <stdio.h>
#include <unistd.h>
#endif

#define CEC_VID 0x2548
#define CEC_PID 0x1001

using namespace CEC;
using namespace std;


uint8_t CioctlCECAdapterDetection::FindAdapters(cec_adapter *deviceList, uint8_t iBufSize, const char *strDevicePath /* = NULL */)
{
  uint8_t iFound(0);

  fprintf(stderr, "*** IOCTL Adapter running\r\n");

  /* NXP HDMI uses /dev/hdmicec and ioctl() communication */
#define NXP_DEV_PATH "/dev/hdmicec"
  if (!access(NXP_DEV_PATH, 0))
  {
    deviceList[iFound].device_type = DEVICE_TYPE_NXP;
    snprintf(deviceList[iFound  ].path, sizeof(deviceList[iFound].path), NXP_DEV_PATH);
    snprintf(deviceList[iFound++].comm, sizeof(deviceList[iFound].path), NXP_DEV_PATH);
    fprintf(stderr, "Detected NXP %s device\r\n", NXP_DEV_PATH);
  }

  iBufSize = 0; /* silence "unused" warning on linux/osx */

  return iFound;
}
