/*
 * 2011-10-07 Sebastian Rockel
 */
#include "TcpClient.h"

//TcpClient::TcpClient(QString hostname, uint16_t portnr) : QThread()
TcpClient::TcpClient(QString hostname, uint16_t portnr) : QObject()
{
  this->hostName=hostname;
  this->portNumber=portnr;
}
//void TcpClient::run()
//{
  //socket = new QTcpSocket(this);
  ////TODO signals
  //socket->connectToHost(hostName, portNumber);
  //// Start QT thread
  //exec();
//}
TcpClient::~TcpClient()
{
  socket->close();
  delete socket;
}

#ifdef CLIENTTEST
int main(int argc, char** argv)
{
  QString hostname="localhost";
  uint16_t portnr=4321;
  TcpClient tcpclient = new TcpClient(hostname,portnr);
  tcpclient.start();
  tcpclient.wait();
  return 0;
}
#endif
