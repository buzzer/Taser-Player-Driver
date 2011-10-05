#include <QThread>
#include <QTcpSocket>
#include <stdint.h>

class TcpClient : public QThread
{
  Q_OBJECT
  private:
    QString hostName;
    uint16_t portNumber;
    QTcpSocket socket;

  public:
    void run();
    TcpClient(QString, uint16_t);
};

TcpClient(QString hostname, uint16_t portnr) : QObject()
{
  this->hostName=hostname;
  this->portNumber=portnr;
}
void TcpClient::run()
{
  socket = new QtcpSocket(this);
  //TODO signals
  socket.connectToHost(hostName, portNumber);
  exec();
}

int main(int argc, char** argv)
{
  QString hostname="localhost";
  uint16_t portnr=4321;
  TcpClient tcpclient = new TcpClient(hostname,portnr);
  tcpclient.start();
  tcpclient.wait();
  return 0;
}
