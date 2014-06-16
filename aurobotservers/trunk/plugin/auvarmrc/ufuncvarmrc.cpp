/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                      *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *
 * $Date: 2011-07-04 17:03:31 +0200 (Mon, 04 Jul 2011) $
 * $Id: ufuncvarmrc.cpp 59 2012-10-21 06:25:02Z jcan $
 *
 ***************************************************************************/

#include <urob4/ufuncplugbase.h>

/**
 * plug-in to make global variable in camera or laser scanner server available to MRC.
 * @author Christian Andersen
*/
class UFuncVarMrc : public UFuncPlugBase
{
public:
  /**
  Constructor */
  UFuncVarMrc()
  { // set the command (or commands) handled by this plugin
    setCommand("varmrc", "varmrc", "Transfers global variable to mrc");
  }
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra)
  {  // handle a plugin command
    const int MRL = 500;
    char reply[MRL];
    bool ask4help;
    const int MVL = 30;
    char value[MVL];
    // check for 'help'
    ask4help = msg->tag.getAttValue("help", value, MVL);
    if (ask4help)
    { // create the reply in XML-like (html - like) format
      sendHelpStart(msg, "varmrc");
      sendText(msg, "--- available VarMrc options\n");
      sendText(msg, "Make a global variable available to the MRC in a MRC script\n");
      sendText(msg, "help            This message\n");
      sendText(msg, "src=G           G is the source global variable, e.g. 'campool.camscnt' or 'gmk.imageSerial[1]'\n");
      sendText(msg, "dst=D           where D is destination variable in MRC either 'vis0' to 'vis9' or 'l0', 'l1' to 'l9'\n");
      sendText(msg, "see also: VAR list\n");
      sendHelpDone(msg);
    }
    else
    { // do some action and send a reply
      const int MGL = 120;
      char src[MGL] = "";
      const int MDL = 10;
      char dst[MDL] = "";
      double val;
      //
      msg->tag.getAttValue("src", src, MGL);
      msg->tag.getAttValue("dst", dst, MDL);
      if (getGlobalValue(src, &val))
      { // MRC reply format */
        if (strncasecmp(dst, "vis", 3) == 0)
        {
          if (strlen(dst) == 0)
            strncpy(dst, "vis0", MDL);
          snprintf(reply, MRL, "<vision %s=\"%g\"/>\n", dst, val);
        }
        else
        {
          if (strlen(dst) == 0)
            strncpy(dst, "l0", MDL);
          snprintf(reply, MRL, "<laser %s=\"%g\"/>\n", dst, val);
        }
        // send this string as the reply to the client
        sendMsg(reply);
        //
      }
      else
      {
        snprintf(reply, MRL, "No such global variable '%s'", src);
        sendWarning(reply);
      }
    }
    // return true if the function is handled with a positive result
    return true;
  }
};

#ifdef LIBRARY_OPEN_NEEDED
/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFuncVarMrc' with your classname */
  return new UFuncVarMrc();
}
#endif
