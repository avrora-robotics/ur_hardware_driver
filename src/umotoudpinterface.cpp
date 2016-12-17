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

#include "umotoudpinterface.h"

namespace umoto_interface
{

UMotoUDPInterface::UMotoUDPInterface()
{

}

void UMotoUDPInterface::connectToIP(const std::string &ip, uint16_t port)
{
  connection.connectToIP(ip, port);
}

UMotoData UMotoUDPInterface::sendControl(const UMotoControl &control)
{
  UMotoControlPacket control_packet(control);
  UMotoDataPacket data_packet;

  sendRequest(control_packet, data_packet);

  return data_packet.data_field;
}

UMotoVersion UMotoUDPInterface::requestUMotoVersion()
{
  UMotoVersionRequestPacket req_packet;
  req_packet.CRC = PacketHandler::calcCRC(&req_packet);
  UMotoVersionRequestPacket resp_packet;

  sendRequest(req_packet, resp_packet);

  return resp_packet.data_field;
}

UDPSocketConnection::UDPSocketConnection()
  : socket_(io_service_), timer_(io_service_)
{

}

UDPSocketConnection::~UDPSocketConnection()
{
  disconnect();
}

void UDPSocketConnection::connectToIP(const std::string &ip, uint16_t port)
{
  endpoint_receiver_ = udp::endpoint(boost::asio::ip::address_v4::from_string(ip), port);
  endpoint_sender_ = udp::endpoint();

  socket_.open(udp::v4());
}

void UDPSocketConnection::disconnect()
{
  io_service_.stop();

  if (socket_.is_open()) {
    socket_.cancel();
    socket_.close();
  }
}

size_t UDPSocketConnection::sendRequest(const std::vector<uint8_t>& request, std::vector<uint8_t>& response)
{
  std::lock_guard<std::mutex> mutex_lock(connection_mutex_);

  socket_.send_to(boost::asio::buffer(request), endpoint_receiver_);

  timer_.expires_from_now(boost::posix_time::milliseconds(UDP_TIMEOUT_MSEC));

  timer_.async_wait(boost::bind(&UDPSocketConnection::handleTimeout, this,
                                boost::asio::placeholders::error));

  bytes_rcv_ = 0;
  error_code_ = boost::asio::error::would_block;

  socket_.async_receive_from(boost::asio::buffer(response), endpoint_receiver_,
                             boost::bind(&UDPSocketConnection::handleReceive, this,
                                         boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));

  do
  {
    io_service_.poll_one();
  }
  while (error_code_ == boost::asio::error::would_block);

  io_service_.reset();

  if (error_code_ || !socket_.is_open())
    throw std::runtime_error(error_code_.message());

  return bytes_rcv_;
}

void UDPSocketConnection::handleReceive(const boost::system::error_code &error, size_t bytes_transferred)
{
  if (error!= boost::system::errc::operation_canceled)
  {
    bytes_rcv_ = bytes_transferred;
    error_code_ = error;
  }
}

void UDPSocketConnection::handleWrite(const boost::system::error_code &error, std::size_t bytes_transferred)
{

}

void UDPSocketConnection::handleTimeout(const boost::system::error_code& error)
{
  if (error != boost::system::errc::operation_canceled)
    error_code_ = boost::asio::error::timed_out;
}

template<typename Request, typename Response>
void UMotoUDPInterface::sendRequest(const Request &request,
                                    Response &response)
{
  std::vector<uint8_t> request_buffer = PacketHandler::serialize(request);
  std::vector<uint8_t> response_buffer(sizeof(response));

  size_t response_length = connection.sendRequest(request_buffer, response_buffer);

  if (response_length!=sizeof(response))
  {
    throw std::range_error("Wrong response length");
  }

  response = PacketHandler::deserialize<Response>(response_buffer);
}

}
