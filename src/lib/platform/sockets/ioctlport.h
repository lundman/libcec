#pragma once
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
#include "../util/buffer.h"

#include <string>
#include <stdint.h>

#if !defined(__WINDOWS__)
#include <termios.h>
#endif

#include "socket.h"

namespace PLATFORM
{
  class CioctlSocket : public CCommonSocket<ioctl_socket_t>
  {
    public:
      CioctlSocket(const CStdString &strName ) :
          CCommonSocket<ioctl_socket_t>(INVALID_SERIAL_SOCKET_VALUE, strName),
          #ifdef __WINDOWS__
          m_iCurrentReadTimeout(MAXDWORD),
          #endif
          m_bIsOpen(false) {}

      virtual ~CioctlSocket(void) { Close(); }

      virtual bool Open(uint64_t iTimeoutMs = 0);
      virtual void Close(void);
      virtual void Shutdown(void);
      virtual int Ioctl(int request, void* data);
      virtual ssize_t Write(void* data, size_t len);
      virtual ssize_t Read(void* data, size_t len, uint64_t iTimeoutMs = 0);

      virtual bool IsOpen(void)
      {
        return m_socket != INVALID_SERIAL_SOCKET_VALUE &&
            m_bIsOpen;
      }

    protected:
  #ifndef __WINDOWS__
      struct termios  m_options;
  #else
      bool SetTimeouts(serial_socket_t socket, int* iError, DWORD iTimeoutMs);
      DWORD           m_iCurrentReadTimeout;
  #endif

      bool            m_bIsOpen;
  };

  class CioctlPort : public CProtectedSocket<CioctlSocket>
  {
  public:
    CioctlPort(const CStdString &strName) :
      CProtectedSocket<CioctlSocket> (new CioctlSocket(strName)) {}
    virtual ~CioctlPort(void) {}
  };
};
