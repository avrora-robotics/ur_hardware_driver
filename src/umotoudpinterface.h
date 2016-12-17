/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, AVRORA ROBOTICS LLC
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AVRORA ROBOTICS LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Vladimir Leushkin
   Desc:
*/

#ifndef UMOTOUDPINTERFACE_H
#define UMOTOUDPINTERFACE_H

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/system/error_code.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>

#include <ros/console.h>

#include <mutex>

#include "umotopackets.h"

using boost::asio::ip::udp;

namespace umoto_interface
{

class UDPSocketConnection
{
public:
  UDPSocketConnection();
  virtual ~UDPSocketConnection();

  void connectToIP(const std::string& ip, uint16_t port);

  void disconnect();

  size_t sendRequest(const std::vector<uint8_t> &request, std::vector<uint8_t> &response);

private:
  void startReading();

  void handleReceive(const boost::system::error_code& error,
                     size_t bytes_transferred);

  void handleWrite(const boost::system::error_code& error,
                   std::size_t bytes_transferred);

  void handleTimeout(const boost::system::error_code &error);

  const int UDP_TIMEOUT_MSEC = 30;

  boost::asio::io_service io_service_;
  udp::socket socket_;
  udp::endpoint endpoint_receiver_;
  udp::endpoint endpoint_sender_;
  boost::asio::deadline_timer timer_;
  boost::system::error_code error_code_;
  size_t bytes_rcv_ = 0;

  std::mutex connection_mutex_;
};

class UMotoUDPInterface
{
public:
  UMotoUDPInterface();
  void connectToIP(const std::string& ip, uint16_t port);

  UMotoData sendControl(const UMotoControl& control);
  UMotoVersion requestUMotoVersion();
private:

  template<typename Request, typename Response>
  void sendRequest(const Request& request, Response& response);

  UDPSocketConnection connection;
};

}
#endif // UMOTOUDPINTERFACE_H
