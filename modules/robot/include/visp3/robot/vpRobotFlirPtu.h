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

#ifndef vpRobotFlirPtu_h
#define vpRobotFlirPtu_h

/*!
  \file vpRobotFlirPtu.h
  Interface for Flir Ptu Cpi robot.
*/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FLIRPTUCPI

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobot.h>
#include <string.h>

#include <visp/vpDebug.h>
#include <visp/vpRobotException.h>
#include <visp/vpColVector.h>
#include <visp/vpRobot.h>

extern "C" {
#include <cpi.h>
//#include <cerial/cerial.h>
}
/*!
  \class vpRobotFlirPtu
  \ingroup group_robot_real_arm
  Interface for  Flir Ptu Cpi robot.
*/
class VISP_EXPORT vpRobotFlirPtu : public vpRobot
{
public:
  typedef enum { STOP, SPEED } vpControllerStatusType;

  typedef enum { FLIR_AXIS_PAN, FLIR_AXIS_TILT } FlirAxis;

  typedef enum { INIT_IP, INIT_SERIAL } InitMode;

  #ifndef DOXYGEN_SHOULD_SKIP_THIS
  // SHM
  typedef struct /* ControllerShm_struct */ {
    vpControllerStatusType status[2];
    double q_dot[2];
    double actual_q[2];
    double actual_q_dot[2];
    bool jointLimit[2];
    } shmType;
  #endif /* DOXYGEN_SHOULD_SKIP_THIS */

  vpRobotFlirPtu(int port);
  vpRobotFlirPtu(std::string ip);
  ~vpRobotFlirPtu();


  void init();

  void reset();
  void goHome();
  bool syncAndLock(struct cerial *cer);
  void halt();

  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q);

  void setBlockUntilPositioned(bool b);
  bool getBlockUntilPositioned();

  bool isRelativePositioning();
  void setRelativePositioning(bool b);

  void setContinuous(bool b);
  bool getContinuous();

  void setPositioningVelocity(FlirAxis axis, double i);
  double getPositioningVelocity(FlirAxis axis);

  void setMaxRotationVelocity(FlirAxis axis, const double maxVr);
  double getMaxRotationVelocity(FlirAxis axis) const;

  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);

  void setVerbose(bool verbose);

  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q);
  vpColVector getPosition(const vpRobot::vpControlFrameType frame);
  vpColVector getVelocity();
  void get_eJe(vpMatrix &);
  void get_fJe(vpMatrix &);
  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q);

  void set_cMe(vpHomogeneousMatrix m);
  vpHomogeneousMatrix get_cMe();

  void set_eMc(vpHomogeneousMatrix m);
  vpHomogeneousMatrix get_eMc();

  void set_cVe(vpVelocityTwistMatrix m);
  vpVelocityTwistMatrix get_cVe();

  double posToRadian(int pos);
  int radianToPos(double rad);

private:
  struct cerial *cer;
  uint16_t		status;
  double		positioningVelocityPan;
  double		positioningVelocityTilt;
  vpColVector	q_previous; // used for getDisplacement
  bool			blockUntilPositioned;
  InitMode		initMode;
  int			port;
  std::string	ip;
  double		maxPanVelocity;
  bool			relativePositioning;
  double		maxTiltVelocity;
  bool			continuousMode;
  vpHomogeneousMatrix cMe;
  vpHomogeneousMatrix eMc;
  vpVelocityTwistMatrix cVe;
  bool			verbose;

  struct cerial *init(int port);
  struct cerial *init(std::string ipAddress);
  void initValues();
  bool finishInit(int baud, int timeout, struct cerial *cer, bool immediateMode);
  void initContinuous();
};

#endif
#endif