//
// client.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include "simplepacket.h"

using boost::asio::ip::tcp;

int main(int argc, char* argv[])
{
  try
  {
    //if (argc != 2)
    //{
      //std::cerr << "Usage: client <host>" << std::endl;
      //return 1;
    //}
    uint16_t port = 4321;
    const char* host="134.100.13.175";
    Packet response;

    boost::asio::io_service io_service;

    tcp::socket socket(io_service);
    tcp::endpoint endpoint
      (
       boost::asio::ip::address::from_string(host),
       port
      );

    socket.connect(endpoint);

    while(true)
    {
      //boost::asio::const_buffer buf2;
      boost::array<uint8_t, 2048> buf;
      boost::system::error_code error;

      size_t len = socket.read_some(boost::asio::buffer(buf), error);

      if (error == boost::asio::error::eof)
        break; // Connection closed cleanly by peer.
      else if (error)
        throw boost::system::system_error(error); // Some other error.

      response.setData((const uint8_t*) buf.data(), len);

      std::cout << "Read " << len << " bytes" << std::endl;
      std::cout << "Command received " << response.getCommand() << std::endl;
      int32_t left = response.popS32();
      int32_t right = response.popS32();
      std::cout << "Got left " << left << " and right " << right << std::endl;
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
