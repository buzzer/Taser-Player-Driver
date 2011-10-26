#include <iostream>
#include <string>
#include <ctime>
#include "simplepacket.h"
#include "protocol_can.h"
#include <boost/asio.hpp>

std::string time_string()
{
   using namespace std;
   time_t now = time(0);
   return ctime(&now);
}
int main ()
{
  unsigned int port = 4321;
	Packet request(CAN_REQUEST | CAN_SET_WHEELSPEEDS);
	request.pushS32(100000);
	request.pushS32(100000);

  try
  {
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::acceptor acceptor
      (
       io_service,
       boost::asio::ip::tcp::endpoint
       (
        boost::asio::ip::tcp::v4(),
        port
       )
      );

    while(true)
    {
      std::cout<<"Listening to localhost on port " << port <<std::endl;
      boost::asio::ip::tcp::socket socket(io_service);
      acceptor.accept(socket);

      if (false == request.send(&socket) )
      {
        std::cout << "Error during packet send!" << std::endl;
      }
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return 0;

}
