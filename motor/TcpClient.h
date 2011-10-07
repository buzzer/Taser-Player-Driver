#include <QThread>
#include <QTcpSocket>
#include <stdint.h>

class TcpClient : public QThread
{
  Q_OBJECT
  private:
    QString hostName;
    uint16_t portNumber;
    QTcpSocket* socket;

  public:
    void run();
    TcpClient(QString, uint16_t);
    ~TcpClient();
};
