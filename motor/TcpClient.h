#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <QThread>
#include <QTcpSocket>
#include <stdint.h>
#include "packet.h"
#include "TcpClientInterface.h"
#include <vector>

//class TcpClient : public QThread
class TcpClient : public QObject
{
  Q_OBJECT
  private:
    QString hostName;
    uint16_t portNumber;
    QTcpSocket* socket;
    QThread* thread;
    // TODO use concurrent safe list
    std::vector<TcpClientInterface*>* listeners;

	private slots:
		void slotReadData();
    void slotConnected();
		void slotStateChanged(QAbstractSocket::SocketState);
    void slotSocketError(QAbstractSocket::SocketError);
    void handleBatteryVoltage(Packet);
    void handleWheelAdvances(Packet);
    void handleMotorTemps(Packet);
    void handleBrakesEnable(Packet);
    void handleBrakesDisable(Packet);
    void handleEmergStopEnable(Packet);
    void handleEmergStopDisable(Packet);
    void handleUnknownMsg(Packet);

  public:
    void run();
    TcpClient(QString, uint16_t);
    ~TcpClient();
    void send(Packet *packet);
		bool receive(Packet *packet);

    std::vector<TcpClientInterface*>::iterator addListener(TcpClientInterface* );
    void removeListener(std::vector<TcpClientInterface*>::iterator);
};

// Declare QT meta types
//Q_DECLARE_METATYPE(QAbstractSocket::SocketState)
//Q_DECLARE_METATYPE(QAbstractSocket::SocketError)
#endif
