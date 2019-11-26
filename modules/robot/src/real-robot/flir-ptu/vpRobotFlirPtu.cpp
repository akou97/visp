/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Interface for Flir Ptu Cpi robot.
 *
 * Authors:
 * AKOURIM Youness (ENS Rennes)
 *
 *****************************************************************************/
/*!
les méthodes définies dans la classe FlirPtu sont : (Get+Set)
- set_cMe
- get_cMe
- set_eMc (New)
- get_eMc (New)
- setBlockUntilPositioned
- getBlockUntilPositioned
- setPositioningVelocity
- setMaxRotationVelocity
- getMaxRotationVelocity
- getDisplacement
- get_eJe 
- get_fJe 
- setVerbose

Pour l'initialisation
- init
- finishInit
- initContinuous
- initValues

Useful commands
- posToRadian
- radianToPos
- Reset
- goHome
- syncAndLock

SERVOING COMMANDS
- halt
- isRelativePositioning
- setRelativePositioning 
- setContinuous 
- getPosition 
- setPosition 
- getVelocity 
- setVelocity

*/



#include <visp3/core/vpConfig.h>


#ifdef VISP_HAVE_FLIRPTUCPI

#include <visp3/robot/vpRobotException.h>


/*!
  \file vpRobotFlirPtu.cpp
  \brief Interface for Flir Ptu Cpi robot.
*/

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobotFlirPtu.h>

/**
Basic constructor for SERIAL connection
\param port Serial port number
*/
vpRobotFlirPtu::vpRobotFlirPtu(int port)
{
  initMode = INIT_SERIAL;
  this->port = port;
  init();
}

/**
Basic constructor for IP connection
\param ip Ip address
*/
vpRobotFlirPtu::vpRobotFlirPtu(std::string ip)
{
  initMode = INIT_IP;
  this->ip = ip;
  init();
}

/**
Basic destructor, close the connection to the robot and free the memory
*/
vpRobotFlirPtu::~vpRobotFlirPtu()
{
  vpColVector qdot(2);
  qdot = 0;
  setVelocity(ARTICULAR_FRAME, qdot);
  if (verbose)
    std::cout << "Closing connection and destroying vpRobotFlir instance" << std::endl;
  cerclose(cer);
  free(cer);
  
}

////////
////
////	GETTERS AND SETTERS
////
////////

/**
Set the effector to camera matrix
\param m the matrix
*/
void vpRobotFlirPtu::set_cMe(vpHomogeneousMatrix m)
{
  if (verbose)
    std::cout << "Setting cMe" << std::endl;
  cMe = m;
}

/**
Get the effector to camera matrix
\return the matrix
*/
vpHomogeneousMatrix vpRobotFlirPtu::get_cMe() { 
  return cMe;
}


/**
Set the camera to effector matrix
\param m the matrix
*/
void vpRobotFlirPtu::set_eMc(vpHomogeneousMatrix m)
{
  if (verbose)
    std::cout << "Setting eMc" << std::endl;
  eMc = m;
}

/**
Get the camera to effector matrix
\return the matrix
*/
vpHomogeneousMatrix vpRobotFlirPtu::get_eMc() { 
	return eMc; }




/**
Block the input until the position is reached (only in position mode)
\param b true to block
*/
void vpRobotFlirPtu::setBlockUntilPositioned(bool b)
{
  if (verbose)
    std::cout << "Setting blockUntilPositioned" << std::endl;
  blockUntilPositioned = b;
}

/**
Return true if the input is blocked until the position is reached (only in position mode)
\return boolean
*/
bool vpRobotFlirPtu::getBlockUntilPositioned() { return blockUntilPositioned; }

/**
Set the positioning velocity (only in position mode)
\param axis The axis (FLIR_AXIS_PAN or FLIR_AXIS_TILT)
\param i The velocity (rad/s)
*/
void vpRobotFlirPtu::setPositioningVelocity(FlirAxis axis, double i)
{
  if (i <= getMaxRotationVelocity(axis) && i >= 0) {
    if (axis == FLIR_AXIS_PAN) {
      if (verbose)
        std::cout << "Setting positioning velocity for pan axis" << std::endl;
      positioningVelocityPan = i;
    } else if (axis == FLIR_AXIS_TILT) {
      if (verbose)
        std::cout << "Setting positioning velocity for tilt axis" << std::endl;
      positioningVelocityTilt = i;
    } else
      throw vpException(vpRobotException::lowLevelError, "Axis unknown");
  } else {
    throw vpException(vpRobotException::lowLevelError, "Velocity > max or velocity < min");
  }
}

/**
Get the positioning velocity used in position mode
\param axis The axis (FLIR_AXIS_PAN or FLIR_AXIS_TILT)
\return The velocity (rad/s)
*/
double vpRobotFlirPtu::getPositioningVelocity(FlirAxis axis)
{
  if (axis == FLIR_AXIS_PAN)
    return positioningVelocityPan;
  else if (axis == FLIR_AXIS_TILT)
    return positioningVelocityTilt;
  else
    throw vpException(vpRobotException::lowLevelError, "Axis unknown");
}

/**
Set the max rotation velocity
\param axis The axis (FLIR_AXIS_PAN or FLIR_AXIS_TILT)
\param maxVr The max rotation velocity (in rad/s)
*/
void vpRobotFlirPtu::setMaxRotationVelocity(FlirAxis axis, const double maxVr)
{
  if (axis == FLIR_AXIS_PAN) {
    if (verbose)
      std::cout << "Setting max rotation velocity for pan axis" << std::endl;
    int maxPanSpeed;
    cpi_ptcmd(cer, &status, OP_PAN_UPPER_SPEED_LIMIT_GET, &maxPanSpeed); // thanks to this command, we can get maxPanSpeed of Pan
    if (maxVr > maxPanSpeed) {
      throw vpException(vpRobotException::lowLevelError, "The velocity is too high");
    } else if (maxVr <= 0)
      throw vpException(vpRobotException::lowLevelError, "You can't set a negative velocity");
    else
      maxPanVelocity = maxVr;
  } else if (axis == FLIR_AXIS_TILT) {
    if (verbose)
      std::cout << "Setting max rotation velocity for tilt axis" << std::endl;
    int maxTiltSpeed;
    cpi_ptcmd(cer, &status, OP_TILT_UPPER_SPEED_LIMIT_GET, &maxTiltSpeed); // thanks to this command, we can get maxPanSpeed of Tilt
    if (maxVr > maxTiltSpeed) {
      throw vpException(vpRobotException::lowLevelError, "The velocity is too high");
    } else if (maxVr <= 0)
      throw vpException(vpRobotException::lowLevelError, "You can't set a negative velocity");
    else
      maxTiltVelocity = maxVr;
  } else
    throw vpException(vpRobotException::lowLevelError, "Axis unknown");
}

/**
Get the max rotation velocity
\param axis The axis (FLIR_AXIS_PAN or FLIR_AXIS_TILT)
\return The max rotation velocity (in rad/s)
*/
double vpRobotFlirPtu::getMaxRotationVelocity(FlirAxis axis) const
{
  if (axis == FLIR_AXIS_PAN)
    return maxPanVelocity;
  else if (axis == FLIR_AXIS_TILT)
    return maxTiltVelocity;
  else
    throw vpException(vpRobotException::lowLevelError, "Axis unknown");
}

void vpRobotFlirPtu::getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q)
{

  vpColVector q_current; // current position

  getPosition(vpRobot::ARTICULAR_FRAME, q_current);

  if (frame == vpRobot::ARTICULAR_FRAME) {
    q = q_current - q_previous;
  }
  q_previous = q_current; // Update for next call of this method
}

// TODO
void vpRobotFlirPtu::get_eJe(vpMatrix &eJe) {
  
  double q1 = getPositioningVelocity(FLIR_AXIS_PAN);
  double q2 = getPositioningVelocity(FLIR_AXIS_TILT);
  
  std::cout << "on est dans eJe" << std::endl;

  vpMatrix M(6,2,{0, 0, 0, sin(q2), 0, cos(q2), 0, 0, 0, 0, -1, 0});
  M.reshape(6, 2);
  eJe = M;
}

void vpRobotFlirPtu::get_fJe(vpMatrix &fJe)
{
  double q1 = getPositioningVelocity(FLIR_AXIS_PAN);
  double q2 = getPositioningVelocity(FLIR_AXIS_TILT);

  vpMatrix M(6, 2, { 0, 0, 0, 0, 0, -1, 0, 0, 0, -sin(q1),cos(q1),0} );
  //M.reshape(6, 2);
  fJe = M;

}


/**
Get the camera to effector Velocity matrix (to check)
\return the matrix
*/
vpVelocityTwistMatrix vpRobotFlirPtu::get_cVe() { 
	return cVe ; }





void vpRobotFlirPtu::setVerbose(bool verbose)
{
  this->verbose = true;
  std::cout << " Verbose mode on" << std::endl;
}

////////
////
////	INITIALIZATION
////
////////

void vpRobotFlirPtu::init()
{
  if (initMode == INIT_IP) {
    if (verbose) {
      std::cout << "Initializing the robot in IP mode" << std::endl;
    }
    this->cer = init(ip);
  } else if (initMode == INIT_SERIAL) {
    if (verbose) {
      std::cout << "Initializing the robot in serial mode" << std::endl;
    }
    this->cer = init(port);
  }

  initValues();
}

/** Initialize the connection to the robot, via serial port
 * \param port  port number
 * \returns a Cerial handle.
 */
struct cerial *vpRobotFlirPtu::init(int port)
{
  if (verbose)
    std::cout << "Initializing the connection to the robot via SERIAL port" << std::endl;
  int baud, timeout;
  char errstr[128];
  struct cerial *cer;
  const char *portname;

  std::string portAsString = "COM" + std::to_string(port);
  portname = portAsString.c_str();

  if (verbose) {
    std::cout << "Use serial port: " << portname << std::endl;
  }

  cer = (cerial *)malloc(sizeof(struct cerial));

  /* get options */
  baud = 9600;
  timeout = 2000;

  if (verbose)
    std::cout << "Opening " << portname << std::endl;

  /* open a port */
  if (ceropen(cer, portname, CERIAL_FLAGS_NONE)) {
    std::string error = "Failed to open " + portAsString + " : " + cerstrerror(cer, errstr, sizeof(errstr));
    if (verbose)
      std::cout << error << std::endl;
    throw vpException(0, error);
  }

  if (!finishInit(baud, timeout, cer, true)) {
    cerclose(cer);
    throw vpException(0, "Error while finishing initialization");
  }

  return cer;
}

/** Initialize the connection to the robot, via ip address
 *
 * \param ip  ip address of the robot
 *
 * \returns a Cerial handle.
 */
struct cerial *vpRobotFlirPtu::init(std::string ip)
{
  if (verbose)
    std::cout << "Initializing connection via ip" << std::endl;
  int baud, timeout;
  char errstr[128];
  struct cerial *cer;

  cer = (cerial *)malloc(sizeof(struct cerial));

  /* get options */
  baud = 9600;
  timeout = 2000;

  /* open an ip address */
  if (verbose)
    std::cout << "Opening " << ip << std::endl;

  ip = "tcp:" + ip;
  if (ceropen(cer, ip.c_str(), CERIAL_FLAGS_NONE)) {
    std::string error = "Failed to open " + ip + " : " + cerstrerror(cer, errstr, sizeof(errstr));
    if (verbose)
      std::cout << error << std::endl;
    throw vpException(0, error);
  }

  if (!finishInit(baud, timeout, cer, true)) {
    cerclose(cer);
    throw vpException(0, "Error while finishing initialization");
  }

  return cer;
}

/**
 * Finish the initialization of the robot by sending different parameters to it
 * \param baud the baudrate
 * \param timeout the command timeout
 * \param cer the robot
 *
 * \returns true if the robot has been correctly initialized
 */
bool vpRobotFlirPtu::finishInit(int baud, int timeout, struct cerial *cer, bool immediateMode)
{
  if (verbose)
    std::cout << "Finishing initialization" << std::endl;

  /* set baudrate
   * ignore errors since not all devices are serial ports
   */
  cerioctl(cer, CERIAL_IOCTL_BAUDRATE_SET, &baud);

  /* flush any characters already buffered */
  cerioctl(cer, CERIAL_IOCTL_FLUSH_INPUT, NULL);

  /* set two second timeout */
  if (cerioctl(cer, CERIAL_IOCTL_TIMEOUT_SET, &timeout)) {
    std::cout << "cerial: timeout ioctl not supported" << std::endl;
    return false;
  }

  if (!syncAndLock(cer)) {
    throw vpRobotException(vpRobotException::lowLevelError, "Error during sync");
  }

  /* immediately execute commands
   * (slave mode should be opt-in)
   */
  if (immediateMode) {
    int rc;
    if ((rc = cpi_ptcmd(cer, &status, OP_EXEC_MODE_SET, (cpi_enum)CPI_IMMEDIATE_MODE))) {
      std::cout << "Set Immediate Mode failed: " << cpi_strerror(rc) << std::endl;
      exit(rc);
    }
  }

  return true;
}

/** Initialize the continuous boolean based on the current value of the robot
 */
void vpRobotFlirPtu::initContinuous()
{
  cpi_enable ret;
  cpi_ptcmd(cer, &status, OP_PAN_CONTINUOUS_GET, &ret);
  if (ret == CPI_ENABLE)
    continuousMode = true;
  else
    continuousMode = false;
}

/**
 *	Initialize maximum speeds and send them to the robot. Also activate the relative positioning.
 */
void vpRobotFlirPtu::initValues()
{
  // Initializing max speeds
  if (verbose)
    std::cout << "Initializing max speeds values" << std::endl;

  int maxPanSpeed;
  int maxTiltSpeed;

  if (cpi_ptcmd(cer, &status, OP_PAN_UPPER_SPEED_LIMIT_GET, &maxPanSpeed) ||
      cpi_ptcmd(cer, &status, OP_TILT_UPPER_SPEED_LIMIT_GET, &maxTiltSpeed)) {
    throw vpException(vpRobotException::lowLevelError, "Failed to read current position");
  }

  setMaxRotationVelocity(FLIR_AXIS_PAN, maxPanSpeed);
  setMaxRotationVelocity(FLIR_AXIS_TILT, maxTiltSpeed);

  setPositioningVelocity(FLIR_AXIS_PAN, 0.1);
  setPositioningVelocity(FLIR_AXIS_TILT, 0.1);

  setRelativePositioning(true);
  // setContinuous(false);
  initContinuous();
  q_previous = 0;
}

////////
////
////	USEFUL COMMANDS
////
////////

/** Convert the angle from position to radian
 * \param pos  the angle in position (Unit of the robot - 2pi radians = 36 000 positions)
 * \returns the angle in radian
 */
double vpRobotFlirPtu::posToRadian(int pos) { return vpMath::rad((double)pos / 100.0); }

/** Convert the angle from radian to position
 * \param rad  the angle in radian
 * \returns the angle in radian (2pi radians = 36 000 positions)
 */
int vpRobotFlirPtu::radianToPos(double rad) { return static_cast<int>(vpMath::deg(rad) * 100); }

/** Reset the robot. Must be called after setting the continious mode
 *
 */
void vpRobotFlirPtu::reset()
{
  if (verbose)
    std::cout << "Resetting the robot" << std::endl;

  int err;
  if (err = cpi_ptcmd(cer, &status, OP_RESET, (cpi_enum)CPI_RESET_ALL)) {
    if (!syncAndLock(cer)) {
      throw vpRobotException(vpRobotException::lowLevelError, "Error during reset :" + err);
    }
  }
}

/** Bring the robot to the "home" position
 *
 */
void vpRobotFlirPtu::goHome()
{
  if (verbose)
    std::cout << "Going to home position" << std::endl;

  // On exécute le déplacement absolu jusqu'en 0, en pan et en tilt
  if (cpi_ptcmd(cer, &status, OP_PAN_DESIRED_POS_SET, (int)0) ||
      cpi_ptcmd(cer, &status, OP_TILT_DESIRED_POS_SET, (int)0)) {
    throw vpException(vpRobotException::lowLevelError, "Failed to go to the position.");
  }
  // On bloque jusqu'à atteindre la position souhaitée
  if (cpi_block_until(cer, NULL, NULL, OP_PAN_CURRENT_POS_GET, 0) ||
      cpi_block_until(cer, NULL, NULL, OP_TILT_CURRENT_POS_GET, 0)) {
    throw vpException(vpRobotException::lowLevelError, "Blocking failed.\n");
  }
}

/** Synchronize to the robot
 *\param cer  the robot)
 */
bool vpRobotFlirPtu::syncAndLock(struct cerial *cer)
{
  if (verbose)
    std::cout << "Sync and lock" << std::endl;

  /* sync and lock */
  int attempt = 0;
  do {
    attempt++;
  } while (attempt <= 3 && (cpi_resync(cer) || cpi_ptcmd(cer, &status, OP_NOOP)));
  if (attempt > 3) {
    std::cout << "Cannot communicate with PTU" << std::endl;
    return false;
  }
  return true;
}

////////
////
////	SERVOING COMMANDS
//// 
////////

/**
 * Stop all operations in progress
 */
void vpRobotFlirPtu::halt()
{
  if (cpi_ptcmd(cer, &status, OP_HALT, (cpi_halt)CPI_HALT_ALL)) {
    throw vpException(vpRobotException::lowLevelError, "Failed to halt operations.");
  }
}

/** Returns true if relative positioning mode is on.
 *	\returns false if absolute positioning
 */
bool vpRobotFlirPtu::isRelativePositioning() { return relativePositioning; }

/** Set the relative positioning or absolute positioning mode.
 * \param b true for relative positioning, false for absolute positioning
 */
void vpRobotFlirPtu::setRelativePositioning(bool b)
{
  if (verbose)
    std::cout << "Setting relative positioning" << std::endl;

  relativePositioning = b;
}

/** Set continuous mode
 * /!\ Restart the robot (via reset()) to apply the modification
 * \param b	true for continuous mode
 */
void vpRobotFlirPtu::setContinuous(bool b)
{
  if (verbose)
    std::cout << "Setting continuous mode" << std::endl;

  continuousMode = b;
  if (cpi_ptcmd(cer, &status, OP_PAN_CONTINUOUS_SET, b ? (cpi_enable)CPI_ENABLE : (cpi_enable)CPI_DISABLE)) {
    throw vpException(vpRobotException::lowLevelError, "Failed to set continuous mode.");
  }
  reset();
}

/** Returns true if the robot is in continuous mode
 * \returns false overwise
 */
bool vpRobotFlirPtu::getContinuous() { return continuousMode; }

/** Returns the actual absolute position
 * \param frame the type of frame in which the position is computed (not yet implemented)
 * \param q the returning position (in rad)
 */
void vpRobotFlirPtu::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  int pan, tilt;
  if (cpi_ptcmd(cer, &status, OP_PAN_CURRENT_POS_GET, &pan) ||
      cpi_ptcmd(cer, &status, OP_TILT_CURRENT_POS_GET, &tilt)) {
    throw vpException(vpRobotException::lowLevelError, "Failed to read current position");
  }
  q[0] = posToRadian(pan);
  q[1] = posToRadian(tilt);
}

/** Returns the actual absolute position
 * \param frame the type of frame in which the position is computed (not yet implemented)
 * \returns the actual absolute position (in rad)
 */
vpColVector vpRobotFlirPtu::getPosition(const vpRobot::vpControlFrameType frame)
{
  vpColVector q(2);
  vpRobotFlirPtu::getPosition(frame, q);
  return q;
}

/** Send a position to the robot, absolute or relative depending on the activated mode (cf setRelativePositioning())
 *	\param frame the type of frame (not implemented yet)
 *	\param q the positioning vector in rad
 */
void vpRobotFlirPtu::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q)
{
  if (verbose)
    std::cout << "Setting position" << std::endl;

  if (q.getRows() != 2) {
    vpERROR_TRACE("Bad dimension for positioning vector.");
    throw vpRobotException(vpRobotException::lowLevelError, "Bad dimension for positioning vector.");
  }

  vpColVector pos_q(2);
  pos_q[0] = radianToPos(q[0]);
  pos_q[1] = radianToPos(q[1]);

  // On récupère les positions max/min du tilt
  int minPanPos, maxPanPos;
  int minTiltPos, maxTiltPos;
  if (cpi_ptcmd(cer, &status, OP_TILT_MIN_POSITION, &minTiltPos) ||
      cpi_ptcmd(cer, &status, OP_TILT_MAX_POSITION, &maxTiltPos)) {
    throw vpException(vpRobotException::lowLevelError, "Failed to read max/min tilt positions");
  }
  if (cpi_ptcmd(cer, &status, OP_PAN_MAX_POSITION, &maxPanPos) ||
      cpi_ptcmd(cer, &status, OP_PAN_MIN_POSITION, &minPanPos)) {
    throw vpException(vpRobotException::lowLevelError, "Failed to read max/min pan positions");
  }

  int currPanPos;
  int currTiltPos;
  cpi_ptcmd(cer, &status, OP_PAN_CURRENT_POS_GET, &currPanPos);
  cpi_ptcmd(cer, &status, OP_TILT_CURRENT_POS_GET, &currTiltPos);

  if (isRelativePositioning()) {
    if (!getContinuous()) {
      // On récupère les positions max/min du pan
      if (cpi_ptcmd(cer, &status, OP_PAN_MAX_POSITION, &maxPanPos) ||
          cpi_ptcmd(cer, &status, OP_PAN_MIN_POSITION, &minPanPos)) {
        throw vpException(vpRobotException::lowLevelError, "Failed to read max/min pan positions");
      }
      if (((currPanPos + pos_q[0]) > maxPanPos) || ((currPanPos + pos_q[0]) < minPanPos))
        throw vpException(vpRobotException::lowLevelError, "Out of Pan bounds");
    }
    if (((currTiltPos + pos_q[1]) > maxTiltPos) || ((currTiltPos + pos_q[1]) < minTiltPos))
      throw vpException(vpRobotException::lowLevelError, "Out of tilt bounds");
  } else {
    std::cout << "pos_q[1]: " << pos_q[1] << " " << minTiltPos << " " << maxTiltPos << std::endl;
    if ((pos_q[0] > maxPanPos) || (pos_q[0] < minPanPos))
      throw vpException(vpRobotException::lowLevelError, "Out of Pan bounds");

    if ((pos_q[1] > maxTiltPos) || (pos_q[1] < minTiltPos))
      throw vpException(vpRobotException::lowLevelError, "Out of tilt bounds");
  }

  // On active le mode "independant", pour envoyer une position
  if (cpi_ptcmd(cer, &status, OP_SPEED_CONTROL_MODE_SET, (cpi_control)CPI_CONTROL_INDEPENDENT)) {
    throw vpException(vpRobotException::lowLevelError, "Failed to set independent control mode");
  }

  /* Deprecated, used to set the speed in percentage of max speed
  // On récupère la vitesse max
  int maxTiltSpeed, maxPanSpeed;
  if (cpi_ptcmd(cer, &status, OP_PAN_UPPER_SPEED_LIMIT_GET, &maxPanSpeed) ||
          cpi_ptcmd(cer, &status, OP_TILT_UPPER_SPEED_LIMIT_GET, &maxTiltSpeed)) {
          throw vpException(vpRobotException::lowLevelError, "Failed to read max speeds");
  }
  */

  // On indique la vitesse souhaitée
  if (cpi_ptcmd(cer, &status, OP_PAN_DESIRED_SPEED_SET, radianToPos(getPositioningVelocity(FLIR_AXIS_PAN))) ||
      cpi_ptcmd(cer, &status, OP_TILT_DESIRED_SPEED_SET, radianToPos(getPositioningVelocity(FLIR_AXIS_TILT)))) {
    throw vpException(vpRobotException::lowLevelError, "Failed to set the velocity.");
  }

  if (isRelativePositioning()) {
    // On exécute le déplacement relatif jusqu'à la position souhaitée
    if (cpi_ptcmd(cer, &status, OP_PAN_DESIRED_POS_REL_SET, (int)pos_q[0]) ||
        cpi_ptcmd(cer, &status, OP_TILT_DESIRED_POS_REL_SET, (int)pos_q[1])) {
      throw vpException(vpRobotException::lowLevelError, "Failed to go to the position.");
    }
  } else {
    // On exécute le déplacement absolu jusqu'à la position souhaitée
    if (cpi_ptcmd(cer, &status, OP_PAN_DESIRED_POS_SET, (int)pos_q[0]) ||
        cpi_ptcmd(cer, &status, OP_TILT_DESIRED_POS_SET, (int)pos_q[1])) {
      throw vpException(vpRobotException::lowLevelError, "Failed to go to the position.");
    }
  }

  // Doit-on bloquer l'envoi de nouvelles commandes ?
  if (getBlockUntilPositioned()) {

    // On récupère la position finale absolue souhaitée
    int posAbsTilt, posAbsPan;
    if (cpi_ptcmd(cer, &status, OP_PAN_DESIRED_POS_GET, &posAbsPan) ||
        cpi_ptcmd(cer, &status, OP_TILT_DESIRED_POS_GET, &posAbsTilt)) {
      throw vpException(vpRobotException::lowLevelError, "Failed to read max speeds");
    }

    // On bloque jusqu'à atteindre la position souhaitée
    if (cpi_block_until(cer, NULL, NULL, OP_PAN_CURRENT_POS_GET, posAbsPan) ||
        cpi_block_until(cer, NULL, NULL, OP_TILT_CURRENT_POS_GET, posAbsTilt)) {
      throw vpException(vpRobotException::lowLevelError, "Blocking failed.\n");
    }
  }
}

/** Get the current robot velocity. For the desired velocity during positioning, see getPositioningVelocity().
 * \returns q[0] = velocity of the pan axis, q[1] = velocity of the tilt axis
 */
vpColVector vpRobotFlirPtu::getVelocity()
{
  int veloPan, veloTilt;
  if (cpi_ptcmd(cer, &status, OP_PAN_CURRENT_SPEED_GET, &veloPan) ||
      cpi_ptcmd(cer, &status, OP_TILT_CURRENT_SPEED_GET, &veloTilt)) {
    throw vpException(vpRobotException::lowLevelError, "Failed to get the velocity.");
  }
  vpColVector q(2);
  q[0] = posToRadian(veloPan);
  q[1] = posToRadian(veloTilt);
  return q;
}

// En radians
/** Move the robot based on pure velocity instead of position. The velocity is in rad/s.
 * \param frame the frame of the displacement (not yet implemented)
 * \param q_dot the velocity
 */
void vpRobotFlirPtu::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &q_dot)
{

  if (frame != vpControlFrameType::ARTICULAR_FRAME) {
    throw vpException(vpRobotException::lowLevelError, "Not implemented for anything except articular frame");
  }

  if (q_dot.getRows() != 2) {
    vpERROR_TRACE("Bad dimension for velocity vector.");
    throw vpRobotException(vpRobotException::lowLevelError, "Bad dimension for velocity vector.");
  }

  int vPan = radianToPos(q_dot[0]);
  int vTilt = radianToPos(q_dot[1]);

  // On active le mode "vélocité pure", pour envoyer seulement une vitesse
  if (cpi_ptcmd(cer, &status, OP_SPEED_CONTROL_MODE_SET, (cpi_control)CPI_CONTROL_PURE_VELOCITY)) {
    throw vpException(vpRobotException::lowLevelError, "Failed to set pure velocity");
  }

  // On indique la vitesse souhaitée
  if (cpi_ptcmd(cer, &status, OP_PAN_DESIRED_SPEED_SET, vPan) ||
      cpi_ptcmd(cer, &status, OP_TILT_DESIRED_SPEED_SET, vTilt)) {
    throw vpException(vpRobotException::lowLevelError, "Failed to set the velocity.");
  }
}

#endif