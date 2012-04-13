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

#include "../os.h"
#include <stdio.h>
#include <fcntl.h>
#include "../sockets/ioctlport.h"
#include "../posix/os-socket.h"

#if defined(__APPLE__) || defined(__FreeBSD__)
#ifndef XCASE
#define XCASE	0
#endif
#ifndef OLCUC
#define OLCUC	0
#endif
#ifndef IUCLC
#define IUCLC	0
#endif
#endif
using namespace std;
using namespace PLATFORM;

void CioctlSocket::Close(void)
{
  if (IsOpen())
    SocketClose(m_socket);
}

void CioctlSocket::Shutdown(void)
{
  if (IsOpen())
    SocketClose(m_socket);
}

int CioctlSocket::Ioctl(int request, void* data)
{
  return IsOpen() ? SocketIoctl(m_socket, &m_iError, request, data) : -1;
}

ssize_t CioctlSocket::Write(void* data, size_t len)
{
  return IsOpen() ? SocketWrite(m_socket, &m_iError, data, len) : -1;
}


#define CEC_IOCTL_BASE 0x40
#define CEC_IOCTL_WAIT_FRAME_CMD 5
#define CEC_IOCTL_WAIT_FRAME      _IOWR(CEC_IOCTL_BASE,CEC_IOCTL_WAIT_FRAME_CMD,cec_frame)

typedef struct
{
   unsigned char count;
   unsigned char service;
   unsigned char addr;
   unsigned char data[15];
} cec_frame;
ssize_t CioctlSocket::Read(void* data, size_t len, uint64_t iTimeoutMs /* = 0 */)
{
  cec_frame frame;
  if (!IsOpen()) return -1;

  if (!Ioctl(CEC_IOCTL_WAIT_FRAME, data))
    return sizeof(frame);

  return 0;
}

//setting all this stuff up is a pain in the ass
bool CioctlSocket::Open(uint64_t iTimeoutMs /* = 0 */)
{
  iTimeoutMs = 0;
  if (IsOpen())
    return false;

  //m_socket = open(m_strName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  m_socket = open(m_strName.c_str(), O_RDWR );

  if (m_socket == INVALID_SERIAL_SOCKET_VALUE)
  {
    m_strError = strerror(errno);
    return false;
  }

  //  SocketSetBlocking(m_socket, true);
  m_bIsOpen = true;

  return true;
}

