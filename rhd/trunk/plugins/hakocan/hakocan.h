/** \file hakocan.h
 *  \ingroup hwmodule
 *  \brief Hardware abstraction layer for HAKO Tractor can-bus control
 *
 *  This liberary inplements the hardware abstraction layer
 *  for communicating using CAN-BUS on the KU-Life
 *  HAKO Automated tractor
 * 
 *  Communication is adjusted directly for the instruction-set
 *  on the HAKO ECU and is based on the HAKO implemention
 *  project by Anders Reeske Nilsen and Asbjørn Mejnertsen in 2006
 *
 *  \author Nils A. Andersen & Anders Billesø Beck, DTU 
 *  $Rev: 203 $
 *  $Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $
 *  
 */

#ifndef HAKOCAN_H
  #define HAKOCAN_H

	extern int initXML(char *);
	extern int periodic(int);
	//extern ”C” int terminate (void) ; //No shutdown function
	#define KEEPAUTOMODE	0x070
	#define RPMCMD			0x200
	#define CURVATURECMD 		0x141
	#define STEERINGREPORTREQ	0x408 
	#define STEERINGREPORT		0x409
	#define CVTCONTROLCMD		0x080
	#define STEERINGANGLECMD	0x142
	#define HITCHCMD		0x090
	#define CVTACK			0x750
	#define STEERINGANGLEACK	0x75A
	#define STEERINGACK		0x758
	#define HORNCMD			0x610
	#define HORNACK			0x740

#endif

