#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <QThread>
#include <QTcpSocket>
#include <stdint.h>
#include "packet.h"
#include "TcpClientInterface.h"

//class TcpClient : public QThread
class TcpClient : public QObject
{
  Q_OBJECT
  private:
    QString hostName;
    uint16_t portNumber;
    QTcpSocket* socket;

	private slots:
		void slotReadData();
    void slotConnected();
		void slotStateChanged(QAbstractSocket::SocketState state);
    void slotSocketError(QAbstractSocket::SocketError);

  public:
    void run();
    TcpClient(QString, uint16_t);
    ~TcpClient();
    void send(Packet *packet);
		bool receive(Packet *packet);

    void addListener(TcpClientInterface* );
    void removeListener(TcpClientInterface* );
};

// Declare QT meta types
//Q_DECLARE_METATYPE(QAbstractSocket::SocketState)
//Q_DECLARE_METATYPE(QAbstractSocket::SocketError)
#endif
