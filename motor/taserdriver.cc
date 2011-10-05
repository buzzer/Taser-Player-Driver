/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2003
 *     Brian Gerkey
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * A simple example of how to write a driver that will be built as a
 * shared object.
 */

// ONLY if you need something that was #define'd as a result of configure
// (e.g., HAVE_CFMAKERAW), then #include <config.h>, like so:
/*
#include <config.h>
*/
// TODO power interface
// TODO read wheel speed

#if !defined (WIN32)
  #include <unistd.h>
#endif
#include <string>


#include "taserdriver.h"
#include <cmath>
#include <iostream>
//#include "packet.h"
#include "protocol_can.h"

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver*
TaserDriver_Init(ConfigFile* cf, int section)
{
  // Create and return a new instance of this driver
  //return((Driver*)(new TaserDriver(cf, section)));
  return((Driver*)(new TaserDriver(cf, section)));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void TaserDriver_Register(DriverTable* table)
{
  table->AddDriver("taserdriver", TaserDriver_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
TaserDriver::TaserDriver(ConfigFile* cf, int section) :
  //QObject(),
  //QCoreApplication(argc,argv),
  ThreadedDriver(cf, section, false, PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
  // zero ids, so that we'll know later which interfaces were requested
  memset(&this->position_id, 0, sizeof(player_devaddr_t));
  memset(&this->power_id, 0, sizeof(player_devaddr_t));

  this->position_subscriptions = 0;
  this->power_subscriptions = 0;

  // Do we create a robot position interface?
  if(cf->ReadDeviceAddr(&(this->position_id), section, "provides",
        PLAYER_POSITION2D_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->position_id) != 0)
    {
      this->SetError(-1);
      return;
    }
  }

  // Do we create a power interface?
  if(cf->ReadDeviceAddr(&(this->power_id), section, "provides",
                      PLAYER_POWER_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->power_id) != 0)
    {
      this->SetError(-1);
      return;
    }
  }

  // Read options from the configuration file
  this->hostName = cf->ReadString(section, "host", "localhost");
  this->port = (uint16_t)cf->ReadInt(section, "port", 1111);
  this->direct_wheel_vel_control = (uint32_t)cf->ReadInt(section, "direct_wheel_vel_control", 1);
  this->motor_max_speed = (float)fabs(cf->ReadLength(section, "max_xspeed", MOTOR_DEF_MAX_SPEED));
  this->motor_max_turnspeed = (float)fabs(cf->ReadAngle(section, "max_yawspeed", MOTOR_DEF_MAX_TURNSPEED));

  return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
int TaserDriver::MainSetup()
{
  puts("Taser driver initialising");

  // Here you do whatever is necessary to setup the device, like open and
  // configure a serial port.

  socket = new QTcpSocket();
  PLAYER_MSG2(0,"Connecting to %s:%d..", this->hostName.toStdString().data(), this->port);

  socket->connectToHost(this->hostName, this->port);
  if (true == socket->waitForConnected(5000))
  {
    PLAYER_MSG0(0,"Connected!");
  } else {
    PLAYER_ERROR("Error connecting!");
    return (-1);
  }
  PLAYER_MSG1(0,"Socket state: %d",socket->state());
  if (socket->state() != QTcpSocket::ConnectedState)
  {
    PLAYER_ERROR("Error: socket not in connected state");
  }

  // to avoid qt warnings
  //qRegisterMetaType<QAbstractSocket::SocketError>("SocketError");
  //qRegisterMetaType<QAbstractSocket::SocketState>("SocketState");

  //// register QT signals and slots
  ////connect(socket, SIGNAL(readyRead()), SLOT(slotReadData()), Qt::DirectConnection);
  //connect(socket, SIGNAL(readyRead()), SLOT(slotReadData()));

  ////connect(socket, SIGNAL(error(QAbstractSocket::SocketError)),
      ////SLOT(slotSocketError(QAbstractSocket::SocketError)), Qt::DirectConnection);
  //connect(socket, SIGNAL(error(QAbstractSocket::SocketError)),
      //SLOT(slotSocketError(QAbstractSocket::SocketError)));

  ////connect(socket, SIGNAL(stateChanged(QAbstractSocket::SocketState)),
      ////SLOT(slotStateChanged(QAbstractSocket::SocketState)), Qt::DirectConnection);
  //connect(socket, SIGNAL(stateChanged(QAbstractSocket::SocketState)),
      //SLOT(slotStateChanged(QAbstractSocket::SocketState)));

  ////connect(socket, SIGNAL(connected()), SLOT(slotConnected()), Qt::DirectConnection);
  //connect(socket, SIGNAL(connected()), SLOT(slotConnected()));

  puts("Taser driver ready");

  return(0);
}

//void
//TaserDriver::slotConnected(void)
//{
  //PLAYER_MSG0(0,"Socket in connected state!");
//}
//void
//TaserDriver::slotReadData(void)
//{
  //Packet response;
  //const uint64_t datalength = socket->bytesAvailable();

  //response.setData(
      //(const uint8_t*)socket->readAll().constData(),
      //datalength
      //);

  //const uint32_t command = response.getCommand();
  //PLAYER_MSG1(0,"%d bytes received with: %d", command);

  //switch (command)
  //{
    //case (CAN_REPLY | CAN_BATTERYVOLTAGE) : handleBatteryVoltage(response);
      //break;
    //case (CAN_REPLY | CAN_WHEELADVANCES) : handleWheelAdvances(response);
      //break;
    //case (CAN_REPLY | CAN_MOTORTEMPS) : handleMotorTemps(response);
      //break;
    //case (CAN_REPLY | CAN_BRAKES_ENABLE) : handleBrakesEnable(response);
      //break;
    //case (CAN_REPLY | CAN_BRAKES_DISABLE) : handleBrakesDisable(response);
      //break;
    //case (CAN_REPLY | CAN_EMERGENCY_STOP_ENABLE) : handleEmergStopEnable(response);
      //break;
    //case (CAN_REPLY | CAN_EMERGENCY_STOP_DISABLE) : handleEmergStopDisable(response);
      //break;
    //default : handleUnknownMsg(response);
      //break;
  //}
//}
void TaserDriver::handleBatteryVoltage(Packet msg)
{
  batVoltage = msg.popF32();
  PLAYER_MSG1(0,"Received battery voltage %f", batVoltage);

  // publish power data
  this->Publish(this->power_id,
                PLAYER_MSGTYPE_DATA,
                PLAYER_POWER_DATA_STATE,
                (void*)&(this->taser_data.power),
                sizeof(player_power_data_t),
                //&timestampStandardSIP);
                NULL);
  //
  //TODO accurate timestamp
  //double: seconds since epoch
}
void TaserDriver::handleWheelAdvances(Packet msg)
{
  advances[0] = msg.popS32();
  advances[1] = msg.popS32();
  PLAYER_MSG2(0,"Received wheel advances %d, %d", advances[0], advances[1]);
  // TODO convert from wheel advances to position (x,y,a)
  // put odometry data
  this->Publish(this->position_id,
                PLAYER_MSGTYPE_DATA,
                PLAYER_POSITION2D_DATA_STATE,
                (void*)&(this->taser_data.position),
                sizeof(player_position2d_data_t),
                //&timestampStandardSIP);
                NULL);
}
void TaserDriver::handleMotorTemps(Packet msg)
{
  motTemp[0] = msg.popF32();
  motTemp[1] = msg.popF32();
  PLAYER_MSG2(0,"Received motor temps %f, %f", motTemp[0], motTemp[1]);
  //TODO this->Publish()
}
void TaserDriver::handleBrakesEnable(Packet msg)
{
  msg.popS32() == 0 ? brakesEnable=true : brakesEnable=false;
  PLAYER_MSG1(0,"Received brakes enabled %s", brakesEnable==true?"true":"false");
  //TODO this->Publish()
}
void TaserDriver::handleBrakesDisable(Packet msg)
{
  msg.popS32() == 0 ? brakesEnable=false : brakesEnable=true;
  PLAYER_MSG1(0,"Received brakes enabled %s", brakesEnable==true?"true":"false");
  //TODO this->Publish()
}
void TaserDriver::handleEmergStopEnable(Packet msg)
{
  msg.popS32() == 0 ? emergencyStopEnable=true : emergencyStopEnable=false;
  PLAYER_MSG1(0,"Received emergency stop enabled %s", emergencyStopEnable==true?"true":"false");
  //TODO this->Publish()
}
void TaserDriver::handleEmergStopDisable(Packet msg)
{
  msg.popS32() == 0 ? emergencyStopEnable=false : emergencyStopEnable=true;
  PLAYER_MSG1(0,"Received emergency stop enabled %s", emergencyStopEnable==true?"true":"false");
  //TODO this->Publish()
}
void TaserDriver::handleUnknownMsg(Packet msg)
{
    PLAYER_WARN1("Received unkown message %d, ignoring.",msg.getCommand());
}

//void
//TaserDriver::slotSendWheelspeed(void)
//{
  //PLAYER_MSG0(0,"::slotSendWheelspeed");

  //Packet request(CAN_REQUEST | CAN_SET_WHEELSPEEDS);
  //request.pushS32((int)(curSpeed[0]*1e6));
  //request.pushS32((int)(curSpeed[1]*1e6));
  //request.send(socket);

  ////TODO check if needed
  //socket->flush();
//}
//void TaserDriver::slotStateChanged(QAbstractSocket::SocketState state)
//{
  //qDebug() << "Socket state: " << state << socket->errorString();

  //switch (state)
  //{
    //case QAbstractSocket::ConnectedState : PLAYER_MSG0(0,"Socket in connected state!");
      //break;
    //case QAbstractSocket::UnconnectedState : PLAYER_MSG0(0,"Socket in unconnected state!");
      //break;
    //case QAbstractSocket::HostLookupState : PLAYER_MSG0(0,"Socket in host lookup state!");
      //break;
    //case QAbstractSocket::ConnectingState : PLAYER_MSG0(0,"Socket in connecting state!");
      //break;
    //case QAbstractSocket::BoundState : PLAYER_MSG0(0,"Socket in bound state!");
      //break;
    //case QAbstractSocket::ClosingState : PLAYER_MSG0(0,"Socket in closing state!");
      //break;
    //case QAbstractSocket::ListeningState : PLAYER_MSG0(0,"Socket in closing state!");
      //break;
    //default: PLAYER_ERROR("Unknown socket state!");
      //break;
  //}
//}
//void slotSocketError(QAbstractSocket::SocketError error)
//{
  //PLAYER_ERROR("Socket error: ");
  //qDebug() << "  " << error;
//}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
void TaserDriver::MainQuit()
{
  puts("Shutting taser driver down");

  // Here you would shut the device down by, for example, closing a
  // serial port.
	//drive.setEmergencyStop(true);
  // TODO check for output
  PLAYER_MSG0(0,"Disconnecting socket..");
  socket->disconnectFromHost();
  if (socket->state() == QAbstractSocket::UnconnectedState ||
      socket->waitForDisconnected(1000))
  {
    PLAYER_MSG0(0,"Disconnected!");
  }
  else
  {
    PLAYER_WARN("Error on disconnecting!");
  }

  puts("Taser driver has been shutdown");
}

TaserDriver::~TaserDriver (void)
{
  player_position2d_data_t_cleanup(&taser_data.position);
  player_power_data_t_cleanup (&taser_data.power);
}

int
TaserDriver::Subscribe(player_devaddr_t id)
{
  int setupResult;

  // do the subscription
  if((setupResult = Driver::Subscribe(id)) == 0)
  {
    // also increment the appropriate subscription counter
    if(Device::MatchDeviceAddress(id, this->position_id))
    {
      this->position_subscriptions++;
    }
    else
    {
      if(Device::MatchDeviceAddress(id, this->power_id))
      {
        this->power_subscriptions++;
      }
    }
  }

  return(setupResult);
}

int
TaserDriver::Unsubscribe(player_devaddr_t id)
{
  int shutdownResult;

  // do the unsubscription
  if((shutdownResult = Driver::Unsubscribe(id)) == 0)
  {
    // also decrement the appropriate subscription counter
    if(Device::MatchDeviceAddress(id, this->position_id))
    {
      this->position_subscriptions--;
      assert(this->position_subscriptions >= 0);
    }
    else
    {
      if(Device::MatchDeviceAddress(id, this->power_id))
      {
        this->power_subscriptions--;
        assert(this->power_subscriptions >= 0);
      }
    }
  }

  return(shutdownResult);
}

/*
 * Dispatch incoming messages
 */
int TaserDriver::ProcessMessage(QueuePointer & resp_queue,
                                  player_msghdr * hdr,
                                  void * data)
{
  // Process messages here.  Send a response if necessary, using Publish().
  // If you handle the message successfully, return 0.  Otherwise,
  // return -1, and a NACK will be sent for you, if a response is required.

  // following copied from p2os driver
  // Check for capabilities requests first
  HANDLE_CAPABILITY_REQUEST (position_id, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_CAPABILITIES_REQ);
  // Position2d caps
  HANDLE_CAPABILITY_REQUEST (position_id, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL);

  // Process other messages
  if(hdr->type == PLAYER_MSGTYPE_REQ)
  {
    return(this->HandleConfig(resp_queue,hdr,data));
  }
  else
  {
    if(hdr->type == PLAYER_MSGTYPE_CMD)
    {
      return(this->HandleCommand(hdr,data));
    }
    else
    {
      return(-1);
    }
  }
}

int
TaserDriver::HandleConfig(QueuePointer & resp_queue,
                          player_msghdr * hdr,
                          void * data)
{
  //int joint = 0;
  //double newSpeed = 0.0f;

  // check for position config requests
  if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                           PLAYER_POSITION2D_REQ_SET_ODOM,
                           this->position_id))
  {
    if(hdr->size != sizeof(player_position2d_set_odom_req_t))
    {
      PLAYER_WARN("Arg to odometry set requests wrong size; ignoring");
      return(-1);
    }
    player_position2d_set_odom_req_t* set_odom_req =
            (player_position2d_set_odom_req_t*)data;

    PLAYER_MSG3(0,"Setting odometry data: %f, %f, %f",
    //qDebug() << "Sending odometry data"
      set_odom_req->pose.px,
      set_odom_req->pose.py,
      set_odom_req->pose.pa
      );

    //TODO store odometry data
    //Packet odomRequest(CAN_REQUEST | CAN_WHEELADVANCES);
    //odomRequest.pushS32(0);
    //odomRequest.send(socket);

    //socket->flush();

    this->Publish(this->position_id, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_SET_ODOM);
    return(0);
  }
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_POSITION2D_REQ_MOTOR_POWER,
                                this->position_id))
  {
    /* motor state change request
     *   1 = enable motors
     *   0 = disable motors (default)
     */
    if(hdr->size != sizeof(player_position2d_power_config_t))
    {
      PLAYER_WARN("Arg to motor state change request wrong size; ignoring");
      return(-1);
    }
    player_position2d_power_config_t* power_config =
            (player_position2d_power_config_t*)data;

    this->ToggleMotorPower(power_config->state);

    this->Publish(this->position_id, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_MOTOR_POWER);
    return(0);
  }
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_POSITION2D_REQ_RESET_ODOM,
                                this->position_id))
  {
    /* reset position to 0,0,0: no args */
    if(hdr->size != 0)
    {
      PLAYER_WARN("Arg to reset position request is wrong size; ignoring");
      return(-1);
    }
    //ResetRawPositions();
    //TODO reset odometer

    this->Publish(this->position_id, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_RESET_ODOM);
    return(0);
  }
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_POSITION2D_REQ_GET_GEOM,
                                this->position_id))
  {
    /* Return the robot geometry. */
    if(hdr->size != 0)
    {
      PLAYER_WARN("Arg get robot geom is wrong size; ignoring");
      return(-1);
    }
    player_position2d_geom_t geom;
    // TODO: Figure out this rotation offset somehow; it's not
    //       given in the Saphira parameters.  For now, -0.1 is
    //       about right for a Pioneer 2DX.
    // TODO fix to suite Taser
    geom.pose.px   = 0.0;
    geom.pose.py   = 0.0;
    geom.pose.pyaw = 0.0;
    // get dimensions from the parameter table
    // TODO put in seperate configuration
    //geom.size.sl = PlayerRobotParams[param_idx].RobotLength / 1e3;
    //geom.size.sw = PlayerRobotParams[param_idx].RobotWidth / 1e3;
    geom.size.sl = 0.35;
    geom.size.sw = 0.35;

    this->Publish(this->position_id, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_POSITION2D_REQ_GET_GEOM,
                  (void*)&geom, sizeof(geom), NULL);
    return(0);
  }
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_POSITION2D_REQ_VELOCITY_MODE,
                                this->position_id))
  {
    /* velocity control mode:
     *   0 = direct wheel velocity control (default)
     *   1 = separate translational and rotational control
     */
    //TODO currently only direct whell control implemented
    if(hdr->size != sizeof(player_position2d_velocity_mode_config_t))
    {
      PLAYER_WARN("Arg to velocity control mode change request is wrong "
                  "size; ignoring");
      return(-1);
    }
    player_position2d_velocity_mode_config_t* velmode_config =
            (player_position2d_velocity_mode_config_t*)data;

    if(velmode_config->value)
      direct_wheel_vel_control = false;
    else
      direct_wheel_vel_control = true;

    this->Publish(this->position_id, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_VELOCITY_MODE);
    return(0);
  }
  else
  {
    PLAYER_WARN("unknown config request to taser driver");
    return(-1);
  }

  return 0;
}

void
TaserDriver::HandlePositionCommand(player_position2d_cmd_vel_t position_cmd)
{
  double speedDemand, turnRateDemand;
  double leftvel, rightvel;
  double rotational_term;
  //unsigned short absspeedDemand, absturnRateDemand;
  //unsigned char motorcommand[4];

  //speedDemand = (int)rint(position_cmd.vel.px * 1e3);
  speedDemand = position_cmd.vel.px;
  //turnRateDemand = (int)rint(RTOD(position_cmd.vel.pa));
  turnRateDemand = RTOD(position_cmd.vel.pa);
  PLAYER_MSG1(0,"Speed demand: %f",speedDemand);
  PLAYER_MSG1(0,"Turnrate demand: %f", turnRateDemand);

  if(this->direct_wheel_vel_control)
  {
    // convert xspeed and yawspeed into wheelspeeds
    PLAYER_MSG0(0,"Direct wheel control enabled");

    // convert xspeed and yawspeed into wheelspeeds
    rotational_term = (M_PI/180.0) * turnRateDemand /
      0.0056; //PlayerRobotParams[param_idx].DiffConvFactor;
    leftvel = (speedDemand - rotational_term);
    rightvel = (speedDemand + rotational_term);

    // Apply wheel speed bounds
    if(fabs(leftvel) > this->motor_max_speed)
    {
      if(leftvel > 0)
      {
        rightvel *= this->motor_max_speed/leftvel;
        leftvel = this->motor_max_speed;
        puts("Left wheel velocity threshholded!");
      }
      else
      {
        rightvel *= -this->motor_max_speed/leftvel;
        leftvel = -this->motor_max_speed;
      }
    }
    if(fabs(rightvel) > this->motor_max_speed)
    {
      if(rightvel > 0)
      {
        leftvel *= this->motor_max_speed/rightvel;
        rightvel = this->motor_max_speed;
        puts("Right wheel velocity threshholded!");
      }
      else
      {
        leftvel *= -this->motor_max_speed/rightvel;
        rightvel = -this->motor_max_speed;
      }
    }

    // Convert from Player speed (m/s) to Taser speed (um/s)
    int speedL = (int)(leftvel * 1e6);
    int speedR = (int)(rightvel * 1e6);
    //int32_t speedL = (int32_t)((leftvel + speedDemand));
    //int32_t speedR = (int32_t)((rightvel + speedDemand));
    //int maximumWheelSpeeds = this->motor_max_speed;
    PLAYER_MSG1(0,"Setting left velocity to %d", speedL);
    PLAYER_MSG1(0,"Setting right velocity to %d", speedR);

    Packet request(CAN_REQUEST | CAN_SET_WHEELSPEEDS);
    request.pushS32(speedL);
    request.pushS32(speedR);
    request.send(socket);

    socket->flush();
  }
  else
  {
    // do separate trans and rot vels
    PLAYER_MSG0(0,"Direct wheel control disabled");
    // TODO non direct control
    PLAYER_ERROR("Separate trans and rot vels not yet supported");
  }
}

int
TaserDriver::HandleCommand(player_msghdr * hdr, void* data)
{
  int retVal = -1;
  //struct timeval timeVal;

  if(Message::MatchMessage(hdr,
                           PLAYER_MSGTYPE_CMD,
                           PLAYER_POSITION2D_CMD_VEL,
                           this->position_id))
  {
    // get and send the latest motor command
    player_position2d_cmd_vel_t position_cmd;
    position_cmd = *(player_position2d_cmd_vel_t*)data;
    this->HandlePositionCommand(position_cmd);
    retVal = 0;
  }
  return retVal;
}

/*
 * toggle motors on/off, according to val
 * 0 - disables motors, enables brakes
 * 1 - enables motors, disables brakes
 */
void
TaserDriver::ToggleMotorPower(uint8_t val)
{
  Packet brakes;
  uint32_t command;

  if (0 == val)
  {
    command = CAN_REQUEST | CAN_BRAKES_ENABLE;
  }
  else
  {
    command = CAN_REQUEST | CAN_BRAKES_DISABLE;
  }

  brakes.setCommand(command);
  brakes.send(socket);
  PLAYER_MSG1(0,"Set motors enable to: %d (0-disabled|1-enabled)",val);
}

////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void TaserDriver::Main()
{
  int last_position_subscrcount=0;
  Packet odomRequest(CAN_REQUEST | CAN_WHEELADVANCES);
  //Packet speedRequest(CAN_REQUEST | CAN_GET_WHEELSPEEDS);
  Packet batRequest(CAN_REQUEST  | CAN_BATTERYVOLTAGE);
  Packet tempRequest(CAN_REQUEST | CAN_MOTORTEMPS);

  //Start QCoreApplication event loop
  //this->exec();

  // The main loop; interact with the device here
  for(;;)
  {
    // test if we are supposed to cancel
    pthread_testcancel();

    // we want to reset the odometry and enable the motors if the first
    // client just subscribed to the position device, and we want to stop
    // and disable the motors if the last client unsubscribed.
    if(!last_position_subscrcount && this->position_subscriptions)
    {
      this->ToggleMotorPower(0);
      //this->ResetRawPositions();
      //TODO reset odometer
    }
    else if(last_position_subscrcount && !(this->position_subscriptions))
    {
      // enable motor power
      this->ToggleMotorPower(1);
    }
    last_position_subscrcount = this->position_subscriptions;


    // Process incoming messages.  TaserDriver::ProcessMessage() is
    // called on each message.
    ProcessMessages();

    // Interact with the device, and push out the resulting data, using
    // Driver::Publish()

    // read Odometer
    //odomRequest.send(socket);
    //socket->flush();
    //speedRequest.send(socket);
    //socket->flush();
    if (this->power_subscriptions > 0)
    {
      batRequest.send(socket);
      // Force to send the packet now, otherwise it most prabably gets delayed
      // until some buffer is full
      socket->flush();
    }
    // read motor temperatures
    //tempRequest.send(socket);


    // TODO read laser
    //socket->waitForReadyRead(50);
    //Process QCoreApplication events manually, since we didn't start its own
    //event loop!
    //QCoreApplication::processEvents();
    //this->processEvents();

    //socket->readAll();
    //socket->waitForReadyRead();
    // Sleep (you might, for example, block on a read() instead)
    usleep(100000);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.

/* need the extern to avoid C++ name-mangling  */
extern "C" {
  int player_driver_init(DriverTable* table)
  {
    puts("Taser driver initializing");
    TaserDriver_Register(table);
    puts("Taser driver done");
    return(0);
  }
}
