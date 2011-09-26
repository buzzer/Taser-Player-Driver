/*
 * 2011-09-07 Sebastian Rockel
 */
#ifndef _TASER_H
#define _TASER_H

// Default max speeds
#define MOTOR_DEF_MAX_SPEED 0.5
#define MOTOR_DEF_MAX_TURNSPEED DTOR(45)
//#include "drive.h"
//#include "battery.h"
//#include <QtCore/QObject>
#include <QObject>
//#include <QtNetwork/QTcpSocket>
//#include <QCoreApplication>
#include <QTcpSocket>
#include "logger.h"
#include <libplayercore/playercore.h>

typedef struct player_taser_data
{
  player_position2d_data_t position;
  player_power_data_t power;
} __attribute__ ((packed)) player_taser_data_t;

////////////////////////////////////////////////////////////////////////////////
// The class for the driver
//class TaserDriver : public ThreadedDriver, public QObject
class TaserDriver : public QObject, public ThreadedDriver
//class TaserDriver : public ThreadedDriver
{
  Q_OBJECT
  private:
    Logger* logger;
    QTcpSocket *socket;

    player_taser_data_t taser_data;

    player_devaddr_t position_id;
    player_devaddr_t power_id;

    int position_subscriptions;

    // Max motor speeds (mm/sec,deg/sec)
    int motor_max_speed;
    int motor_max_turnspeed;
    int direct_wheel_vel_control; // false -> separate trans and rot vel

		//Battery battery;		///< a battery attached to CAN
		//RemoteControl remoteControl;	///< a remoteControl attached to CAN
    //Drive drive;			///< the motors attached to CAN
    //
  private slots:
    void slotStateChanged(QAbstractSocket::SocketState state);
    void slotSendWheelspeed();
    void slotReadData();
    void slotSocketError(QAbstractSocket::SocketError error);

  public:
    // Constructor; need that
    TaserDriver(ConfigFile* cf, int section);
    ~TaserDriver(void);

    virtual int Subscribe(player_devaddr_t id);
    virtual int Unsubscribe(player_devaddr_t id);

    // This method will be invoked on each incoming message
    virtual int ProcessMessage(QueuePointer &resp_queue, 
                               player_msghdr * hdr,
                               void * data);

    int HandleConfig(QueuePointer & resp_queue,
                     player_msghdr * hdr,
                     void* data);
    int HandleCommand(player_msghdr * hdr, void * data);
    void ToggleMotorPower(unsigned char val);
    void HandlePositionCommand(player_position2d_cmd_vel_t position_cmd);

  private:

    // Main function for device thread.
    virtual void Main();
    virtual int MainSetup();
    virtual void MainQuit();

    int foop;
};

#endif
