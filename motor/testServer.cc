/*
 * 2011-09-26 Sebastian Rockel
 * Server example
 */
#include "testServer.h"
#include <iostream>
using namespace std;

Server::Server(QObject* parent): QObject(parent)
{
  connect(&server, SIGNAL(newConnection()), this, SLOT(acceptConnection()));

  //server.listen(QHostAddress::Any, 4321);
  server.listen(QHostAddress::LocalHost, 4321);
}

Server::~Server()
{
  server.close();
}

void Server::acceptConnection()
{
  client = server.nextPendingConnection();

  connect(client, SIGNAL(readyRead()), this, SLOT(startRead()));
}

void Server::startRead()
{
  char buffer[1024] = {0};
  client->read(buffer, client->bytesAvailable());
  cout << buffer << endl;
  client->close();
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  Server server;
  return app.exec();
}
