// coding: utf-8
// ----------------------------------------------------------------------------
/* Copyright (c) 2009, Roboterclub Aachen e.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Roboterclub Aachen e.V. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ROBOTERCLUB AACHEN E.V. ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOTERCLUB AACHEN E.V. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */
// ----------------------------------------------------------------------------

#ifndef XPCC__CAN_USB_HPP
#define XPCC__CAN_USB_HPP

#include <asio.hpp>

namespace xpcc
{
	/**
	 * \brief	Driver for a CAN232 or CANUSB adapter
	 * 
	 * \see		http://www.canusb.com/
	 * \see		http://www.can232.com/
	 * \ingroup	can
	 */
	class CanUsb
	{
	public:
		CanUsb();
		
		~CanUsb();
		
		static bool
		open();
		
		static bool
		close();
		
		static inline bool
		isMessageAvailable();
		
		/*
		static bool
		getMessage(Can::Message& message);
		*/
		
		/*
		 * The CAN controller has a free slot to send a new message.
		 *
		 * \return true if a slot is available, false otherwise
		 */

		static inline bool
		isReadyToSend()
		{
			return true;
		}
		
		/*
		 * Send a message over the CAN.
		 *
		 * \return true if the message was send, false otherwise
		 */

		/*
		static bool
		sendMessage(const Can::Message& message);
		*/


	private:

		/*
		boost::asio::io_service*  io_service;
		boost::asio::serial_port_base::baud_rate baud_rate;
		boost::asio::serialPort serialPort;
		*/

	};
}

#endif // XPCC__CAN_USB_HPP
