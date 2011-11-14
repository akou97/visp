/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Moving edges.
 *
 * Authors:
 * Andrew Comport
 *
 *****************************************************************************/

/*!
  \file vpMeTracker.h
  \brief Contains abstract elements for a Distance to Feature type feature.
*/

// ===================================================================
/*!
  \class vpMeTracker
  \ingroup TrackingImageME
  \brief Contains abstract elements for a Distance to Feature type feature.

  2D state = list of points, 3D state = feature
  
 */
// ===================================================================

#ifndef vpMeTracker_HH
#define vpMeTracker_HH

#include <visp/vpColVector.h>
#include <visp/vpMeSite.h>
#include <visp/vpMe.h>
#include <visp/vpTracker.h>

#include <math.h>
#include <iostream>
#include <list>

class VISP_EXPORT vpMeTracker : public vpTracker
{
public:

  //! Tracking dependent variables/functions

  //! List of tracked moving edges points.
  std::list<vpMeSite> list ;
  //! Moving edges initialisation parameters
  vpMe *me ;
  //! Used for backwards compatibility...could be removed
  int nGoodElement;
  int query_range;
  unsigned int init_range;
  bool display_point;// if 1 (TRUE) displays the line that is being tracked

  // Constructor/Destructor
  vpMeTracker() ;
  vpMeTracker(const vpMeTracker& meTracker) ;
  virtual ~vpMeTracker() ;
  void init() ;
  void initTracking(const vpImage<unsigned char>& I);

  unsigned int numberOfSignal() ;

  //! Display contour.
  virtual void display(const vpImage<unsigned char> &I, vpColor col)=0;
  void display(const vpImage<unsigned char>& I);
  //! Displays the status of moving edge sites
  void display(const vpImage<unsigned char>& I, vpColVector &w, unsigned int &index_w);
  vpMeTracker& operator =(vpMeTracker& f);
  int outOfImage( int i , int j , int half , int rows , int cols) ;
  int outOfImage( vpImagePoint iP , int half , int rows , int cols) ;
  //!Sample pixels at a given interval
  virtual void sample(const vpImage<unsigned char> &image)=0;
  void setDisplay(vpMeSite::vpMeSiteDisplayType select)  { 
    selectDisplay = select ;
  }
  void setMe(vpMe *me) { this->me = me ; }
  unsigned int totalNumberOfSignal() ;

  //! Track sampled pixels.
  void track(const vpImage<unsigned char>& I);

protected:
  vpMeSite::vpMeSiteDisplayType selectDisplay ;

};


#endif

