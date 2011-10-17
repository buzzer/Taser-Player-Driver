/*
 * 2011-10-07 Sebastian Rockel
 */
#include "TcpClient.h"
#include <QDebug>
#include "protocol_can.h"

//TcpClient::TcpClient(QString hostname, uint16_t portnr) : QThread()
TcpClient::TcpClient(QString hostname, uint16_t portnr) : QObject()
{
  hostName=hostname;
  portNumber=portnr;
  listeners = new std::vector<TcpClientInterface*>();

  socket = new QTcpSocket(this);
  ////TODO signals
  socket->connectToHost(hostName, portNumber);
  // TODO connect Signals
  // register QT signals and slots
  connect(socket, SIGNAL(readyRead()), this, SLOT(slotReadData()));

  connect(socket, SIGNAL(error(QAbstractSocket::SocketError)), this,
      SLOT(slotSocketError(QAbstractSocket::SocketError)));

  connect(socket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this,
      SLOT(slotStateChanged(QAbstractSocket::SocketState)));

  connect(socket, SIGNAL(connected()), this, SLOT(slotConnected()));

  // Threading
  thread= new QThread();

  this->moveToThread(thread);
  thread->start();
}

TcpClient::~TcpClient()
{
  socket->close();
  delete socket;
  thread->terminate();
  thread->wait();
  delete thread;
  listeners->clear();
  delete listeners;
}

// QT slots
void
TcpClient::slotConnected(void)
{
  qDebug() << "Socket in connected state!";
}
void
TcpClient::slotReadData(void)
{
  Packet response;
  const uint64_t datalength = socket->bytesAvailable();

  response.setData(
      (const uint8_t*)socket->readAll().constData(),
      datalength
      );

  const uint32_t command = response.getCommand();
  qDebug() << "bytes received with: " << command;

  switch (command)
  {
    case (CAN_REPLY | CAN_BATTERYVOLTAGE) : handleBatteryVoltage(response);
      break;
    case (CAN_REPLY | CAN_WHEELADVANCES) : handleWheelAdvances(response);
      break;
    case (CAN_REPLY | CAN_MOTORTEMPS) : handleMotorTemps(response);
      break;
    case (CAN_REPLY | CAN_BRAKES_ENABLE) : handleBrakesEnable(response);
      break;
    case (CAN_REPLY | CAN_BRAKES_DISABLE) : handleBrakesDisable(response);
      break;
    case (CAN_REPLY | CAN_EMERGENCY_STOP_ENABLE) : handleEmergStopEnable(response);
      break;
    case (CAN_REPLY | CAN_EMERGENCY_STOP_DISABLE) : handleEmergStopDisable(response);
      break;
    default : handleUnknownMsg(response);
      break;
  }
}
void TcpClient::handleBatteryVoltage(Packet msg)
{
  float batVoltage = msg.popF32();
  qDebug() << "Received battery voltage " << batVoltage;

  // TODO call listener callback
  //TODO accurate timestamp
  //double: seconds since epoch
}
void TcpClient::handleWheelAdvances(Packet msg)
{
  int32_t advances[2];

  advances[0] = msg.popS32();
  advances[1] = msg.popS32();
  qDebug() << "Received wheel advances " << advances[0] << "\t" << advances[1];
  // TODO call listener callback
}
void TcpClient::handleMotorTemps(Packet msg)
{
  float motTemp[2];

  motTemp[0] = msg.popF32();
  motTemp[1] = msg.popF32();
  qDebug() << "Received motor temps " << motTemp[0] << "\t" << motTemp[1];
  // TODO call listener callback
}
void TcpClient::handleBrakesEnable(Packet msg)
{
  bool brakesEnable;

  msg.popS32() == 0 ? brakesEnable=true : brakesEnable=false;
  qDebug() << "Received brakes enabled " << brakesEnable;
  // TODO call listener callback
}
void TcpClient::handleBrakesDisable(Packet msg)
{
  bool brakesEnable;
  msg.popS32() == 0 ? brakesEnable=false : brakesEnable=true;
  qDebug() << "Received brakes enabled " << brakesEnable;
  // TODO call listener callback
}
void TcpClient::handleEmergStopEnable(Packet msg)
{
  bool emergencyStopEnable;

  msg.popS32() == 0 ? emergencyStopEnable=true : emergencyStopEnable=false;
  qDebug() << "Received emergency stop enabled " << emergencyStopEnable;
  // TODO call listener callback
}
void TcpClient::handleEmergStopDisable(Packet msg)
{
  bool emergencyStopEnable;

  msg.popS32() == 0 ? emergencyStopEnable=false : emergencyStopEnable=true;
  qDebug() << "Received emergency stop enabled " << emergencyStopEnable;
  // TODO call listener callback
}
void TcpClient::handleUnknownMsg(Packet msg)
{
    qDebug() << "Received unkown message, ignoring: " << msg.getCommand();
  // TODO call listener callback
}

void TcpClient::slotStateChanged(QAbstractSocket::SocketState state)
{
  qDebug() << "Socket state: " << state << socket->errorString();

  switch (state)
  {
    case QAbstractSocket::ConnectedState : qDebug() << "Socket in connected state!";
      break;
    case QAbstractSocket::UnconnectedState : qDebug() << "Socket in unconnected state!";
      break;
    case QAbstractSocket::HostLookupState : qDebug() << "Socket in host lookup state!";
      break;
    case QAbstractSocket::ConnectingState : qDebug() << "Socket in connecting state!";
      break;
    case QAbstractSocket::BoundState : qDebug() << "Socket in bound state!";
      break;
    case QAbstractSocket::ClosingState : qDebug() << "Socket in closing state!";
      break;
    case QAbstractSocket::ListeningState : qDebug() << "Socket in closing state!";
      break;
    default: qDebug() << "Unknown socket state!";
      break;
  }
}
void TcpClient::slotSocketError(QAbstractSocket::SocketError error)
{
  qDebug() << "Socket error: " << error;
}

/*
 * Adds a client listener
 * @param listener A reference to the listener object
 * @return the iterator to the object in the vector past to this one (!)
 */
std::vector<TcpClientInterface*>::iterator TcpClient::addListener(TcpClientInterface* listener)
{
  listeners->push_back(listener);
  return listeners->end();
}
/*
 * Removes a client listener
 * @param listenerPose The listener iterator which points to the listener to be
 * removed
 */
void TcpClient::removeListener(std::vector<TcpClientInterface*>::iterator listenerPose)
{
  listeners->erase(listenerPose);
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
