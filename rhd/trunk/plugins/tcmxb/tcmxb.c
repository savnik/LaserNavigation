/** \file roboteq.c
 *  \ingroup hwmodule
 *  \brief PNI ForceField TCM XB sensor (Armadillo Scout - University of Hohenheim)
 *
 *
 * Tilt-compensated compass modules provide reliable, pinpoint-accurate pitch,
 * roll and compass heading. The TCMs use advanced algorithms to counter the effects
 * of hard and soft iron interference, providing highly accurate heading information
 *  in most any environment and any orientation.  PNI's patented magneto-inductive
 *  sensors and pioneering measurement technology combine to provide all this
 *  performance under a low power budget that extends mission duration.
 *
 * It is designed for the Armadillo Scout, which SDU build for UniHo. It is the first robot that can be controlled with both MobotWare and FroboMind
 *
 * Armadillo Scout official webpage: https://mpt.uni-hohenheim.de/90593?&L=1
 * Armadillo Scout Documentation webpage: http://mpt-internal.uni-hohenheim.de/doku.php?id=robots:armadillo:welcome
 *
 *  \author Claes Jæger-Hansen
 *  $Rev: 59 $
 *  $Date: 2012-05-30 13:30:06 +0200 (Wed, 30 May 2012) $
 */

/***************************************************************************
 *                  Copyright 2012 Claes Jæger-Hansen                      *
 *                                 cjh@uni-hohenheim.de                    *
 *                                 claeslund@gmail.com                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
/***************************** Plugin version  *****************************/
 #define RTQVERSION 	      "1.0"
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $"
 #define DATE             "$Date: 2012-05-30 13:30:06 +0200 (Wed, 30 May 2012) $"
 #define ID               "$Id: tcmxb.c 59 2012-10-21 06:25:02Z jcan $"
/***************************************************************************/
