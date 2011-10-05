/*
 * 2011-09-07 Sebastian Rockel
 */
#ifndef _TASER_H
#define _TASER_H

// Default max speeds
#define MOTOR_DEF_MAX_SPEED 0.5
#define MOTOR_DEF_MAX_TURNSPEED DTOR(45)
#include <QObject>
#include <QTcpSocket>
#include <QCoreApplication>
#include "logger.h"
#include <libplayercore/playercore.h>
#include "packet.h"

typedef struct player_taser_data
{
  player_position2d_data_t position;
  player_power_data_t power;
  player_opaque_data_t motorTemps;
  player_opaque_data_t brakes;
} __attribute__ ((packed)) player_taser_data_t;

////////////////////////////////////////////////////////////////////////////////
// The class for the driver
//class TaserDriver : public QObject, public ThreadedDriver
class TaserDriver : public QCoreApplication, public ThreadedDriver
{
  Q_OBJECT
  private:
    Logger* logger;
    QTcpSocket *socket;
    QString hostName;
    uint16_t port;

    player_taser_data_t taser_data;

    player_devaddr_t position_id;
    player_devaddr_t power_id;

    int position_subscriptions;
    int power_subscriptions;

    // Max motor speeds (mm/sec,deg/sec)
    float motor_max_speed;
    float motor_max_turnspeed;
    uint32_t direct_wheel_vel_control; // false -> separate trans and rot vel
    float curSpeed[2];
    float batVoltage;
    float motTemp[2];
    int advances[2];
    bool brakesEnable;
    bool emergencyStopEnable;

  private slots:
    //void slotSendWheelspeed();
    void slotReadData();
    void slotConnected();
    void slotStateChanged(QAbstractSocket::SocketState);
    void slotSocketError(QAbstractSocket::SocketError);

  public:
    // Constructor; need that
    //TaserDriver(ConfigFile* cf, int section);
    TaserDriver(ConfigFile* cf, int section, int & argc, char** argv);
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
    void handleBatteryVoltage(Packet);
    void handleWheelAdvances(Packet);
    void handleMotorTemps(Packet);
    void handleBrakesEnable(Packet);
    void handleBrakesDisable(Packet);
    void handleEmergStopEnable(Packet);
    void handleEmergStopDisable(Packet);
    void handleUnknownMsg(Packet);

    // Main function for device thread.
    virtual void Main();
    virtual int MainSetup();
    virtual void MainQuit();

    int foop;
};
// Declare QT meta types
Q_DECLARE_METATYPE(QAbstractSocket::SocketState)
Q_DECLARE_METATYPE(QAbstractSocket::SocketError)

#endif
