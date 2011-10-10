#ifndef TCPCLIENTINTERFACE_H
#define TCPCLIENTINTERFACE_H
/*
 * An interface to interact with the TcpClient class
 * 2011-10-10 Sebastian Rockel
 */
#include <string>

/*
 * Gives the minimal methods that a subclass has to override
 */
class TcpClientInterface
{
  public:
    virtual void onError(std::string* ) = 0;
    virtual void onWrite(bool ) = 0;
    virtual void onRead(Packet* ) = 0;
};

#endif
