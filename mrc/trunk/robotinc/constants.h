/** \file constants.h
 * Definition of global types used in the smrdemo
 * program.
 *
 * \author Lars Valdemar Mogensen
 *
 * \date 15/06-2006
 */

#ifndef SMRDEMO_CONST
#define SMRDEMO_CONST 

/// Old enum type for the robot types.
enum robottypes{SMR=0,MMR,ACKER};
/// \brief New enum type for the robot types. It is
/// backwards compatible with the old one.
enum robottype{DIFFERENTIAL=0,ACKERMAN,VELOMEGA,GRIPPER,HEXACOPTER};

#endif
