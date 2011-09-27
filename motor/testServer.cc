/*
 * 2011-09-26 Sebastian Rockel
 * Server example
 */
#include "testServer.h"
#include <iostream>
#include <stdint.h>
#include <QHostAddress>
using namespace std;

Server::Server(QObject* parent): QObject(parent)
{
  connect(&server, SIGNAL(newConnection()), this, SLOT(acceptConnection()));
  QHostAddress hostName = QHostAddress::LocalHost;
  uint16_t port = 4321;

  //server.listen(QHostAddress::Any, 4321);
  server.listen(hostName, port);
  cout << "Listening on " << hostName.toString().toStdString() << ":" << port << endl;
}

Server::~Server()
{
  server.close();
  cout << "Closed socket!" << endl;
}

void Server::acceptConnection()
{
  client = server.nextPendingConnection();

  connect(client, SIGNAL(readyRead()), this, SLOT(startRead()));
}

void Server::startRead()
{
  cout << "Start reading.." << endl;
  char buffer[1024] = {0};
  client->read(buffer, client->bytesAvailable());
  cout << buffer << endl;
  client->close();
  cout << "Closed socket!" << endl;
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  Server server;
  return app.exec();
}
