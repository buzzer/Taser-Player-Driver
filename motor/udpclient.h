#ifndef UDPCLIENT_H
#define UDPCLIENT_H

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/param.h>		// for htons() and friends
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <QCoreApplication>

#include <QTcpSocket>
#include <QDebug>
#include <QTimer>


#include "packet.h"
#include "logger.h"
#include "protocol_can.h"

class UdpClient : public QCoreApplication
{
	Q_OBJECT
	private:
		QTimer* timer;
 		QTcpSocket *socket;
		Logger* logger;
		//int sock, fromlen, n;
		//struct sockaddr_in server;
		//struct sockaddr_in from;
		//unsigned char buf[1024];

	private slots:
		void slotStateChanged(QAbstractSocket::SocketState state);
		//void slotStateChanged();
		void slotSendWheelspeed();
		void slotReadData();

	public:
		UdpClient(int & argc, char** argv);
		~UdpClient();
		
		//void send(Packet *packet);
		bool receive(Packet *packet);
		
		void run(void);
};

#endif


