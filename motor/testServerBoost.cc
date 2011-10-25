#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <ctime>
#include "simplepacket.h"
#include "protocol_can.h"

std::string time_string()
{
   using namespace std;
   time_t now = time(0);
   return ctime(&now);
}
int main () {
  unsigned int port = 4321;
	Packet request(CAN_REQUEST | CAN_SET_WHEELSPEEDS);
	request.pushS32(100000);
	request.pushS32(100000);

   try {
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

         std::string message = time_string();
         //const unsigned char* pointer= request.getData();
         //signed char* p1 = boost::asio::buffer_cast<signed char*>(pointer);
         //std::string message(boost::asio::buffer_cast<const signed char*>(pointer));
         boost::system::error_code ignored_error;
         boost::asio::write
           (
            socket,
            boost::asio::buffer(message),
            boost::asio::transfer_all(),
            ignored_error
           );
      }
   }
   catch (std::exception& e)
   {
      std::cerr << e.what() << std::endl;
   }
   return 0;

}
