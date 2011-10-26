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

    // synchronous call
    if ( true == response.receive(&socket) )
    {
      std::cout << "Command received 0x" << response.getCommand() << std::endl;
      int32_t left = response.popS32();
      int32_t right = response.popS32();
      std::cout << "Got left " << left << "um and right " << right << "um"<< std::endl;
    }
    else
    {
      std::cout << "Error during packet receive!" << std::endl;
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
