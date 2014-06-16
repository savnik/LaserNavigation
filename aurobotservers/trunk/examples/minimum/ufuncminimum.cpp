/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#include "ufuncminimum.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncMinimum' with your classname, as used in the headerfile */
  return new UFuncMinimum();
}

#endif

///////////////////////////////////////////////////

bool UFuncMinimum::handleCommand(UServerInMsg * msg, void * extra)
{
  const int MRL = 500;
  char reply[MRL];
  bool ask4help;
  double a = 100.0, b = 100.0;
  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", NULL, 0);
  msg->tag.getAttDouble("a", &a, a);
  msg->tag.getAttDouble("b", &b, b);
  // ask4help = false, i.e. no 'help' option.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart("MINIMUM");
    sendText("--- MINIMUM is a demonstration plug-in that calculates the minimum of two numbers\n");
    sendText("a=A        Value of A (default is 100)\n");
    sendText("b=B        Value of B (default is 100)\n");
    sendText("help       This message\n");
    sendHelpDone();
  }
  else
  { // calculate minimum value
    snprintf(reply, MRL, "minimum(a=%g, b=%g) is %g", a, b, fmin(a,b));
    sendInfo(reply);
  }
  return true;
}

