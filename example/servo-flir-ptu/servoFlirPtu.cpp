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
 *   Tests the control law
 *
 * Authors:
 * AKOURIM Youness (ENS Rennes)
 *
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FLIRPTUCPI

#include <visp3/robot/vpRobotFlirPtu.h>
#include <visp3/core/vpTime.h>

int main()
{
  try {
    std::string ip1("169.254.110.254");
    std::string ip2("169.254.110.253");
    vpRobotFlirPtu robot(ip1);
    vpColVector q(2, 0);

	//robot.setVerbose(true);
    robot.setRelativePositioning(false);
    robot.setMaxRotationVelocity(vpRobotFlirPtu::FLIR_AXIS_PAN, vpMath::rad(2));
       // robot.setPositioningVelocity(100.);
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
  
    q = 0;
    std::cout << "Set position in the articular frame: " << q.t() << std::endl;
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
   

	int val;
    std::cout << "on attend" << std::endl;
        scanf("%d", &val);
	
    q[0] = vpMath::rad(10);
    q[1] = vpMath::rad(10);
    std::cout << "Set position in the articular frame: " << q.t() << std::endl;
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q);

    vpColVector qm(2);
    robot.getPosition(vpRobot::ARTICULAR_FRAME, qm);
    std::cout << "Position in the articular frame (rad):" << qm.t() << std::endl;
    std::cout << "Position in the articular frame (deg)" << vpMath::deg(qm[0]) << " " << vpMath::deg(qm[1])
               << std::endl;

	 std::cout << "on attend" << std::endl;
    scanf("%d", &val);

	
	
    

    vpColVector qdot(2);
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
#if 1
    qdot = 0 ;
    qdot[0] = vpMath::rad(10) ;
    qdot[1] = vpMath::rad(10) ;
    std::cout << "Set articular frame velocity " << qdot.t() << std::endl;

    robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;
    vpTime::sleepMs(2000) ;
  
	return EXIT_SUCCESS;

    qdot = 0 ;
    qdot[0] = vpMath::rad(-10) ;
    qdot[1] = vpMath::rad(-10) ;

    std::cout << "Set articular frame velocity " << qdot.t() << std::endl;
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot) ;
    vpTime::sleepMs(2000);
#endif

    qdot = 0;
    //  qdot[0] = vpMath::rad(0.1) ;
    qdot[1] = vpMath::rad(10);
    std::cout << "Set articular frame velocity " << qdot.t() << std::endl;
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot);
    vpTime::sleepMs(2000);

    qdot = 0;
    qdot[0] = vpMath::rad(-5);
    // qdot[1] = vpMath::rad(-5);

    std::cout << "Set articular frame velocity " << qdot.t() << std::endl;
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot);
    vpTime::sleepMs(2000);
  } catch (const vpException &e) {
    std::cout << "Catch Flir Ptu exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "You do not have an Flir Ptu robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}

#endif