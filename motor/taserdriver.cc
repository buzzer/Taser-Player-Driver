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
#include <string.h>

#include <libplayercore/playercore.h>

#include "taserdriver.h"
#include <math.h>

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* 
TaserDriver_Init(ConfigFile* cf, int section)
{
  // Create and return a new instance of this driver
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
TaserDriver::TaserDriver(ConfigFile* cf, int section)
    : ThreadedDriver(cf, section, false, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, 
             PLAYER_POSITION2D_CODE)
{
  // Read an option from the configuration file
  this->foop = cf->ReadInt(section, "foo", 0);

  this->motor_max_speed = (int)rint(1e3 * cf->ReadLength(section,
        "max_xspeed",
        MOTOR_DEF_MAX_SPEED));
  this->motor_max_turnspeed = (int)rint(RTOD(cf->ReadAngle(section,
        "max_yawspeed",
        MOTOR_DEF_MAX_TURNSPEED)));

  return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
int TaserDriver::MainSetup()
{   
  puts("Taser driver initialising");

  // Here you do whatever is necessary to setup the device, like open and
  // configure a serial port.

  //printf("Was foo option given in config file? %d\n", this->foop);

	// startup motors first, even if the incoming commands don't require them. Otherwise, as soon
	// as we DO have a command that requires the motors, the server hangs during startMotors();
	drive.startMotors();

	// Instead of applying brakes (releasing them takes forever), we set Emergency Stop, which also
	// applies the brakes, but can be undone much faster.
	drive.setEmergencyStop(true);

	puts("CanServer::CanServer(): brakes applied, ready and waiting.");

  puts("Taser driver ready");

  return(0);
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
void TaserDriver::MainQuit()
{
  puts("Shutting taser driver down");

  // Here you would shut the device down by, for example, closing a
  // serial port.
	drive.setEmergencyStop(true);

  puts("Taser driver has been shutdown");
}

TaserDriver::~TaserDriver (void)
{
  player_position2d_data_t_cleanup(&taser_data.position);
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
      this->position_subscriptions++;
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
  }

  return(shutdownResult);
}

int TaserDriver::ProcessMessage(QueuePointer & resp_queue,
                                  player_msghdr * hdr,
                                  void * data)
{
  // Process messages here.  Send a response if necessary, using Publish().
  // If you handle the message successfully, return 0.  Otherwise,
  // return -1, and a NACK will be sent for you, if a response is required.

  // following copied from p2os driver
  // Position2d caps
  HANDLE_CAPABILITY_REQUEST (position_id, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL);

  // Process other messages
  if(hdr->type == PLAYER_MSGTYPE_REQ)
    return(this->HandleConfig(resp_queue,hdr,data));
  else if(hdr->type == PLAYER_MSGTYPE_CMD)
    return(this->HandleCommand(hdr,data));
  else
    return(-1);
}

int
TaserDriver::HandleConfig(QueuePointer & resp_queue,
                          player_msghdr * hdr,
                          void * data)
{
  int joint = 0;
  double newSpeed = 0.0f;

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

    // TODO set odometry
    //this->sippacket->x_offset = ((int)rint(set_odom_req->pose.px*1e3)) -
            //this->sippacket->xpos;
    //this->sippacket->y_offset = ((int)rint(set_odom_req->pose.py*1e3)) -
            //this->sippacket->ypos;
    //this->sippacket->angle_offset = ((int)rint(RTOD(set_odom_req->pose.pa))) -
            //this->sippacket->angle;

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
    // TODO
    //geom.size.sl = PlayerRobotParams[param_idx].RobotLength / 1e3;
    //geom.size.sw = PlayerRobotParams[param_idx].RobotWidth / 1e3;

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
  int speedDemand, turnRateDemand;
  double leftvel, rightvel;
  double rotational_term;
  unsigned short absspeedDemand, absturnRateDemand;
  unsigned char motorcommand[4];

  speedDemand = (int)rint(position_cmd.vel.px * 1e3);
  turnRateDemand = (int)rint(RTOD(position_cmd.vel.pa));

  if(this->direct_wheel_vel_control)
  {
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

    // Apply byte range bounds
    //if (leftvel / 20 > 126)
      //leftvel = 126 * 20;
    //if (leftvel / 20 < -126)
      //leftvel = -126 * 20;
    //if (rightvel / 20 > 126)
      //rightvel = 126 * 20;
    //if (rightvel / 20 < -126)
      //rightvel = -126 * 20;

    // send the speed command
    //motorcommand[0] = VEL2;
    //motorcommand[1] = ARGINT;
    //motorcommand[2] = (char)(rightvel /
                             //PlayerRobotParams[param_idx].Vel2Divisor);
    //motorcommand[3] = (char)(leftvel /
                             //PlayerRobotParams[param_idx].Vel2Divisor);
    //motorcommand[2] = (char)(rightvel /
                             //20);
    //motorcommand[3] = (char)(leftvel /
                             //20);

    //motorpacket.Build(motorcommand, 4);
    //this->SendReceive(&motorpacket);
    puts("Set right velocity to ");
    cout << rightvel << endl;
    puts("Set left velocity to ");
    cout << leftvel << endl;
		// SET MOTOR SPEEDS
    // We shouldn't get packets requesting a wheelspeed higher than this...
    //int maximumWheelSpeeds = config->getMaximumWheelSpeed() * 1000000 * 1.1;

    assert(abs(speedL) < maximumWheelSpeeds);
    assert(abs(speedR) < maximumWheelSpeeds);

    static bool driveInitialized = false;

    if(!driveInitialized)
    {
      // we start the motors in run() already.
      //drive.startMotors();
      usleep(20000);
      drive.setEmergencyStop(false);
      usleep(20000);
      driveInitialized = true;
    }

    drive.setMotorSpeeds(speedL, speedR);
  }
  // TODO non direct control
}

int
TaserDriver::HandleCommand(player_msghdr * hdr, void* data)
{
  int retVal = -1;
  struct timeval timeVal;

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

/* toggle motors on/off, according to val */
void
TaserDriver::ToggleMotorPower(unsigned char val)
{
  if (val == 0)
  {

    // TOGGLE BRAKES ON/OFF
    puts("enabling brakes.");
    drive.setEmergencyStop(true);

  } else {

    puts("disabling brakes.");
    drive.setEmergencyStop(false);

  }
}

////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void TaserDriver::Main() 
{
  int last_position_subscrcount=0;

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

		// speed should be between -46656 and 46656 (-36 to 36 ^ 3)
		// maximum Speed in m/s.
		float maximumTranslationSpeed = 1.0;

		float speedLeft, speedRight;

		speedLeft = maximumTranslationSpeed / (double)46656 * (double)speed;
		speedRight= maximumTranslationSpeed / (double)46656 * (double)speed;

		puts("before steering: speedLeft %2.2f, speedRight %2.2f", speedLeft, speedRight);

		float maximumRotationSpeed = maximumTranslationSpeed / 2.5;

		// steering should be between -36 and 36 (-36 to 36 ^ 3)
		float steeringFactor = (maximumRotationSpeed / (double)36.0 * (double)steering);
		puts("steeringFactor is %2.2f", steeringFactor);
		// steeringfactor is -1 to +1

		speedLeft = speedLeft + steeringFactor;
		speedRight = speedRight - steeringFactor;

		puts("setting speed to %2.2f left, %2.2f right", speedLeft, speedRight);

		Packet requestSpeed(CAN_REQUEST | CAN_SET_WHEELSPEEDS);
		requestSpeed.pushF32(speedLeft);
		requestSpeed.pushF32(speedRight);
		send(&requestSpeed);

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
