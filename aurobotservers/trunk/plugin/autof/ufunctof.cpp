/***************************************************************************
 *   Copyright (C) 2011 by DTU (Kristian Villien)                          *
 *   jca@elektro.dtu.dk                                                    *
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
 *
 * $Date: 2011-09-19 15:10:00 +0200 (Mon, 19 Sep 2011) $
 * $Id: ufunctof.cpp 59 2012-10-21 06:25:02Z jcan $
 ***************************************************************************/

char CAMERA_IP[] = "192.168.0.69";
char ROBOT_IP[] = "192.168.0.99";
int SETTINGS_PORT = 8080;
int DATA_PORT = 50002;

#include <urob4/uvariable.h>
#include <urob4/uresposehist.h>
#include <urob4/uvarcalc.h>
#include <urob4/uimagepool.h>
#include <ucam4/ucammount.h>
#include <ucam4/ucampool.h>

#include "ufunctof.h"

#include <pthread.h>

#define MDAXMLConnectCP 	0
#define MDAXMLHeartbeat		1
#define MDAXMLDisconnectCP	2
#define MDAXMLGetIP		3
#define MDAXMLSetIP		4
#define MDAXMLGetSubNetmask	5
#define MDAXMLSetSubNetmask	6
#define MDAXMLGetGatewayAddress	7
#define MDAXMLSetGatewayAddress	8
#define MDAXMLGetDHCPMode	9
#define MDAXMLSetDHCPMode	10
#define MDAXMLGetXmlPortCP	11
#define MDAXMLSetXmlPortCP	12
#define MDAXMLGetTCPPortCP	13
#define MDAXMLSetTCPPortCP	14
#define MDAXMLHeartBeat		15
#define xmlOPS_GetIORegister	16
#define xmlOPS_SetIORegister	17
#define MDAXMLSetWorkingMode	18
#define MDAXMLGetFrontendData	19
#define MDAXMLSetFrontendData	20
#define MDAXMLGetTrigger	21
#define MDAXMLSetTrigger	22
#define MDAXMLTriggerImage	23
#define MDAXMLGetDebounceTrigger	24
#define MDAXMLSetDebounceTrigger	25
#define MDAXMLGetAverageDetermination	26
#define MDAXMLSetAverageDetermination	27
#define MDAXMLGetProgram		28
#define MDAXMLSetProgram		29
#define MDAXMLSetMedianFilterStatus	30
#define MDAXMLGetMedianFilterStatus	31
#define MDAXMLSetMeanFilterStatus	32
#define MDAXMLGetMeanFilterStatus	33


#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncTOF' with your classname, as used in the headerfile */
  return new UFuncTOF();
}

#endif


UFuncTOF * tofObj = NULL;

///////////////////////////////////////////////////


UFuncTOF::~UFuncTOF() // skal rettes
{
  stop(true);
}

/////////////////////////////////////////////////

void UFuncTOF::init()
{
  tofObj = this;

  threadRunningData = false;
  threadRunningSetting = false;
  threadStop = false;

}

///////////////////////////////////////////////////

bool UFuncTOF::handleCommand(UServerInMsg * msg, void * extra)
{ // message is unhandled
  bool result = false;
  //
  if (msg->tag.isTagA("tof"))
    result = handleTOF(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////
/*
void testfunctionstring(void)
{
    char string1[] = "POST /RPC2 HTTP/1.1\r\nUser-Agent: XMLRPC++ 0.7\r\nHost: 192.168.0.99:24921\r\nContent-Type:text/xml\r\nContent-length: 208\r\n\r\n<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n<methodCall><methodName>MDAXMLConnectCP</methodName>\r\n<params><param><value>192.168.0.99</value></param><param><value><i4>0</i4></value></param></params></methodCall>\r\n";
    char string2[] = "HTTP/1.1 200 OK\r\nServer: XMLRPC++ 0.7\r\n\r\nContent-Type: text/xml\r\nContent-length: 285\r\n<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n<methodResponse><fault>\r\n<value><struct><member><name>faultCode</name><value><i4>-1</i4></value></member$\r\nunknown method name</value></member></struct></value>\r\n</fault></methodResponse>";
    printf("string2: %s\r\n",string2);
  //  ParseResponse(string2, string1);

}*/

bool UFuncTOF::handleTOF(UServerInMsg * msg)
{
 bool result;

  // check for parameter 'help'
  if (msg->tag.getAttValue("help", NULL, 0) or msg->tag.getAttCnt() == 0)
  { // create the reply in XML-like (html - like) format
    sendHelpStart("TOF");
    sendText("--- Time Of Flight camera help\n");
    sendText("help is not done yet");
    /*snprintf(reply, MRL, "open=true|false     Start or stop camera stream to img=%d (depth) and img=%d color\n",
             varImagesC3D->getInt(1), varImagesC3D->getInt(0));
    sendText(reply);
    sendText(            "silent              do not send a reply to this command\n");
    if (varImagesC3D != NULL)
    {
      snprintf(reply, MRL, "debug=true|false  Produce debug image (is %s for color-depth img=%d)\n",
               bool2str(varImagesC3D->getBool(0)), varImagesC3D->getInt(1));
      sendText(reply);
    }
    snprintf(reply, MRL, "log=true|false      Opens or closes the logfile %s (open=%s)\n", logf.getLogFileName(), bool2str(logf.isOpen()));
    sendText(reply);
    snprintf(reply, MRL, "logImage=N          logging of every N images (depth and color) %s (N=%d)\n", logImage.getLogFileName(), varImageLogN->getInt());
    sendText(reply);
    snprintf(reply, MRL, "logAcc=N            logging of every N acc value to %s (N=%d)\n", logAcc.getLogFileName(), varAccLogN->getInt());
    sendText(reply);
    sendText("help       This message\n");
    sendText("----\n");
    sendText("see also: 'kinectPush' for event handling of 3d cloud\n");
    sendText("see also: 'var kinect' for parameters\n");*/
    sendHelpDone();
  }
  else if (msg->tag.getAttValue("open", NULL, 0))
  {
      printf("\r\nStarting device\r\n");
      SettingPort.setHost("192.168.0.69");
      SettingPort.setPort(8080);
      printf("host set: %s:%d\r\n",SettingPort.getHost(),SettingPort.getPort());
      printf("Trying to connect...\r\n");
      result = SettingPort.tryConnect();
      if(result) printf("OH MY GOD! it worked\r\n");
      else       printf("Failed as expected\r\n");

      result = SettingPort.isConnected();
      if(result)
      {
          printf("we are connected\r\n");
          printf("host IP set: %s\r\n",SettingPort.getHostIP());
          printf("Starting camera Session...\r\n");
          callXMLRPC(MDAXMLConnectCP);
          printf("getting setting...\r\n");
          callXMLRPC(MDAXMLGetFrontendData);//get setting
          callXMLRPC(MDAXMLGetTrigger);//check setting
          callXMLRPC(MDAXMLGetAverageDetermination);//check setting
          callXMLRPC(MDAXMLGetMedianFilterStatus);//check setting
          callXMLRPC(MDAXMLGetMeanFilterStatus);//check setting



          printf("\r\nopening data port\r\n");
          DataPort.setHost(CAMERA_IP);
          DataPort.setPort(DATA_PORT);
          printf("host set: %s:%d\r\n",DataPort.getHost(),DataPort.getPort());
          printf("Trying to connect...\r\n");
          result = DataPort.tryConnect();
          if(result)
          {
              printf("OH MY GOD! it worked\r\n");
              varIsOpen->setValued(1);
              start();
          }
          else
          {
              printf("Failed as expected\r\n");
              printf("Trying to close connection...\r\n");
              SettingPort.closeConnection();
              result = SettingPort.isConnected();
              varIsOpen->setValued(0);
          }
          printf("\r\n");
      }
      else
      {
          printf("we are NOT connected\r\n");
          varIsOpen->setValued(0);
      }

  }
  else if (msg->tag.getAttValue("close", NULL, 0))
  {
      result = SettingPort.isConnected();
      if(result)
      {
         printf("stopping");
          stop(true);
         printf("stopped?");
          callXMLRPC(MDAXMLDisconnectCP);
          printf("we are connected\r\n");
          printf("Trying to close connection...\r\n");
          SettingPort.closeConnection();
          result = SettingPort.isConnected();
          if(result) printf("we are still connected\r\n");
          else       printf("we are no longer connected\r\n");
          varIsOpen->setValued(0);
          printf("close data connection...\r\n");
          DataPort.closeConnection();
          result = DataPort.isConnected();
          if(result) printf("we are still connected\r\n");
          else       printf("we are no longer connected\r\n");
          varIsOpen->setValued(0);
      }
      else
      {
          printf("we are NOT connected\r\n");
          varIsOpen->setValued(0);
      }

  }
  else if (msg->tag.getAttValue("imageget", NULL, 0))
  {
//      printf("starting imageserver\r\n");
//      callXMLRPC(MDAXMLSetWorkingMode);
      result = DataPort.isConnected();
      if(result)
      {
          if(FreeRunning == 0)
          {
              printf("getting images... \r\n");
              TriggerImage = true;
              while(TriggerImage == true); // wait for trigger
              while(ImagesMissing != 0); // wait for images
              printf("done\r\n");
          }
          else
          {
              printf("free running enabled, image update constant.");
          }
      }
      else
      {
          printf("we are NOT connected\r\n");
      }

  }
  else if (msg->tag.getAttValue("test", NULL, 0))
  {
char string1[500];
//	    char string2[] = "HTTP/1.1 200 OK\r\nServer: XMLRPC++ 0.7\r\n\r\nContent-Type: text/xml\r\nContent-length: 285\r\n<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n<methodResponse><fault>\r\n<value><struct><member><name>faultCode</name><value><i4>-1</i4></value></member>\r\nunknown method name</value></member></struct></value>\r\n</fault></methodResponse>";
	char string2[] = "HTTP/1.1 200 OK\r\nServer: XMLRPC++ 0.7\r\nContent-Type: text/xml\r\nContent-length: 222\r\n<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n<methodResponse><params><param>\r\n<value><array><data><value><i4>0</i4></value><value>4041</value><value>O3D201AB$\r\n</param></params></methodResponse>";

    printf("string2: %s\r\n",string2);
    ParseResponse(string2, string1);
  }
  else
  { // get any command attributes (when not a help request)
        printf("\r\nCommand unknown\r\n");
  }
  return true;
}

/////////////////////////////////////////////////////////////////

bool UFuncTOF::callXMLRPC(int command)
{
    const int limit = 10000;
    int sp = 0; //string pointer
    char Call[limit];
    char Reply[limit];
    char Variables[limit];
    //bool failed = false;
    char *CurrentValue;
    char *NextValue;

    //bool result;
    if(GenerateRequest(command,Call))
        return false;
    sp = strlen(Call); //find size
    if(!SettingPort.blockSend(Call, sp)) //send the string)
        printf("ERROR in sending, oh shit");

    Reply[0] = 0;
    if(WaitForReply(Reply,1000, limit))
        printf("timeout, end not found");
    sp = strlen(Reply);
    if(sp == 0)
        printf("no reply, this sucks");
    if(ParseResponse(Reply, Variables) == false)  // get variables from reply
        printf("cant read reply");
    CurrentValue = Variables;

    // three possible senarios, 1) response was a fault. 2) command failed, 3) command succes
    // check for fault

    if(strstr(CurrentValue,"FAULT;") != NULL)
    {
        printf("There is something wrong with the XMLCall: \"%s\"\r\n",CurrentValue);
        printf("command call(%d):\r\n%s\r\n",sp,Call);
        printf("\r\nreply(%d):\r\n%s\r\n",sp,Reply);
        return false;
    }
    // find error value
    NextValue = GetNextValue(CurrentValue);
    //check if command failed
    if(CurrentValue[0] != '0')
    {
        printf("the command could not be completet, error code: %s",CurrentValue);
        printf("command call(%d):\r\n%s\r\n",sp,Call);
        printf("\r\nreply(%d):\r\n%s\r\n",sp,Reply);
        return false;
    }
  //  while
    CurrentValue = NextValue; NextValue = GetNextValue(CurrentValue);
    switch(command)
    {
    case MDAXMLConnectCP:
        sprintf(CameraVersion,"%s",CurrentValue);
        CurrentValue = NextValue; NextValue = GetNextValue(CurrentValue);
        sprintf(CameraName,"%s",CurrentValue);
        printf("camera IDed, type: %s - version: %s\r\n",CameraName,CameraVersion);
        break;
//    case MDAXMLDisconnectCP:
//    case MDAXMLGetIP:
//    case MDAXMLSetIP:
//    case MDAXMLGetSubNetmask:
//    case MDAXMLSetSubNetmask:
//    case MDAXMLGetGatewayAddress:
//    case MDAXMLSetGatewayAddress:
//    case MDAXMLGetDHCPMode:
//    case MDAXMLSetDHCPMode:
//    case MDAXMLGetXmlPortCP:
//    case MDAXMLSetXmlPortCP:
//    case MDAXMLGetTCPPortCP:
//    case MDAXMLSetTCPPortCP:
    case MDAXMLHeartBeat: // no reply
        break;
//    case xmlOPS_GetIORegister:
//    case xmlOPS_SetIORegister:
    case MDAXMLSetWorkingMode:
        DATA_PORT = atoi(CurrentValue);
        break;
    case MDAXMLSetFrontendData:
    case MDAXMLGetFrontendData: //0,Modulation frequency,double sampling,0,integration1,integration2,20,mute time
        CurrentValue = NextValue; NextValue = GetNextValue(CurrentValue); // error hint
        ModulationFrequencySetting = atoi(CurrentValue);
        CurrentValue = NextValue; NextValue = GetNextValue(CurrentValue);
        DoubleSampling = atoi(CurrentValue);
        CurrentValue = NextValue; NextValue = GetNextValue(CurrentValue); // always 0
        CurrentValue = NextValue; NextValue = GetNextValue(CurrentValue);
        IntegrationTime1 = atoi(CurrentValue);
        CurrentValue = NextValue; NextValue = GetNextValue(CurrentValue);
        IntegrationTime2 = atoi(CurrentValue);
        CurrentValue = NextValue; NextValue = GetNextValue(CurrentValue);//always 20
        CurrentValue = NextValue; NextValue = GetNextValue(CurrentValue);
        FrameMuteTime = atoi(CurrentValue);
        printf("(setting) ModulationFreq:%d - doubsamp:%d - integr1&2:%d/%d - mute:%d\r\n",ModulationFrequencySetting,DoubleSampling,IntegrationTime1,IntegrationTime2,FrameMuteTime);
        break;
    case MDAXMLGetTrigger:
        FreeRunningValue = atoi(CurrentValue);
        if(FreeRunningValue == 3) FreeRunning = 1;
        else            FreeRunning = 0;
        printf("freerunning: %d\r\n",FreeRunning);
        break;
    case MDAXMLSetTrigger: break;
    case MDAXMLTriggerImage: break;
//    case MDAXMLGetDebounceTrigger:
//    case MDAXMLSetDebounceTrigger:
    case MDAXMLGetAverageDetermination:
      NumberOfImagesAverage = atoi(CurrentValue);
      printf("NumberOfImagesAverage: %d\r\n",NumberOfImagesAverage);

      break;
    case MDAXMLSetAverageDetermination:
      break;
//    case MDAXMLGetProgram:
//    case MDAXMLSetProgram:
    case MDAXMLSetMedianFilterStatus: break;
    case MDAXMLGetMedianFilterStatus:
        MedianFilter = atoi(CurrentValue);
        printf("MedianFilter: %d\r\n",MedianFilter);
        break;
    case MDAXMLSetMeanFilterStatus: break;
    case MDAXMLGetMeanFilterStatus:
        MeanFilter = atoi(CurrentValue);
        printf("MeanFilter: %d\r\n",MeanFilter);
        break;
    default: break;
    }
    return true;
}

bool UFuncTOF::WaitForReply(char* Reply,int timeout,int limit)
{
    int i = 0;
    int sp = 0;
    while(strstr(Reply,"</methodResponse>") == NULL && ((i*10) < timeout))
    {
        sp += SettingPort.getDataFromLine(Reply+sp,limit,10); // wait for reply
        i++;
        Reply[sp] = 0;
    }
    if((i*10) >= timeout)
        return true;
    else
        return false;
}
char* UFuncTOF::GetNextValue(char* reply)
{
    char* NextValue = strstr(reply,";");
    if(NextValue != NULL)
    {
        NextValue[0] = '\0';
        NextValue++;
    }
    return NextValue;
}


#define NUMBER_OF_TAGS 10

#define END            0
#define METHODRESPONSE 1
#define FAULT          2
#define PARAMS         3
#define PARAM          4
#define VALUE          5
#define MEMBER         6
#define NAMEt           7
#define I4             8
#define ARRAY          9
#define DATAt           10
const char* PossibleTags[] = {
"</",
"<methodResponse>",
"<fault>",
"<params>",
"<param>",
"<value>",
"<member>",
"<name>",
"<i4>",
"<array>",
"<data>",
"...",
"...",
"..."
};

bool UFuncTOF::GenerateRequest(int type,char* Request)
{
    const int limit = 1000;
    int sp = 0,sp2; //string pointer
    char XMLCall[limit];
    bool failed = false;
    //generate XML body
    sp+=sprintf(XMLCall+sp, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n<methodCall><methodName>");
    switch(type)
    {
        case MDAXMLConnectCP: sp+=sprintf(XMLCall+sp, "MDAXMLConnectCP");break;
        case MDAXMLDisconnectCP: sp+=sprintf(XMLCall+sp, "MDAXMLDisconnectCP");break;
        case MDAXMLGetIP: sp+=sprintf(XMLCall+sp, "MDAXMLGetIP");break;
        case MDAXMLSetIP: sp+=sprintf(XMLCall+sp, "MDAXMLSetIP");break;
        case MDAXMLGetSubNetmask: sp+=sprintf(XMLCall+sp, "MDAXMLGetSubNetmask");break;
        case MDAXMLSetSubNetmask: sp+=sprintf(XMLCall+sp, "MDAXMLSetSubNetmask");break;
        case MDAXMLGetGatewayAddress: sp+=sprintf(XMLCall+sp, "MDAXMLGetGatewayAddress");break;
        case MDAXMLSetGatewayAddress: sp+=sprintf(XMLCall+sp, "MDAXMLSetGatewayAddress");break;
        case MDAXMLGetDHCPMode: sp+=sprintf(XMLCall+sp, "MDAXMLGetDHCPMode");break;
        case MDAXMLSetDHCPMode: sp+=sprintf(XMLCall+sp, "MDAXMLSetDHCPMode");break;
        case MDAXMLGetXmlPortCP: sp+=sprintf(XMLCall+sp, "MDAXMLGetXmlPortCP");break;
        case MDAXMLSetXmlPortCP: sp+=sprintf(XMLCall+sp, "MDAXMLSetXmlPortCP");break;
        case MDAXMLGetTCPPortCP: sp+=sprintf(XMLCall+sp, "MDAXMLGetTCPPortCP");break;
        case MDAXMLSetTCPPortCP: sp+=sprintf(XMLCall+sp, "MDAXMLSetTCPPortCP");break;
        case MDAXMLHeartBeat: sp+=sprintf(XMLCall+sp, "MDAXMLHeartbeat");break;
        case xmlOPS_GetIORegister: sp+=sprintf(XMLCall+sp, "xmlOPS_GetIORegister");break;
        case xmlOPS_SetIORegister: sp+=sprintf(XMLCall+sp, "xmlOPS_SetIORegister");break;
        case MDAXMLSetWorkingMode: sp+=sprintf(XMLCall+sp, "MDAXMLSetWorkingMode");break;
        case MDAXMLGetFrontendData: sp+=sprintf(XMLCall+sp, "MDAXMLGetFrontendData");break;
        case MDAXMLSetFrontendData: sp+=sprintf(XMLCall+sp, "MDAXMLSetFrontendData");break;
        case MDAXMLGetTrigger: sp+=sprintf(XMLCall+sp, "MDAXMLGetTrigger");break;
        case MDAXMLSetTrigger: sp+=sprintf(XMLCall+sp, "MDAXMLSetTrigger");break;
        case MDAXMLTriggerImage: sp+=sprintf(XMLCall+sp, "MDAXMLTriggerImage");break;
        case MDAXMLGetDebounceTrigger: sp+=sprintf(XMLCall+sp, "MDAXMLGetDebounceTrigger");break;
        case MDAXMLSetDebounceTrigger: sp+=sprintf(XMLCall+sp, "MDAXMLSetDebounceTrigger");break;
        case MDAXMLGetAverageDetermination: sp+=sprintf(XMLCall+sp, "MDAXMLGetAverageDetermination");break;
        case MDAXMLSetAverageDetermination: sp+=sprintf(XMLCall+sp, "MDAXMLSetAverageDetermination");break;
        case MDAXMLGetProgram: sp+=sprintf(XMLCall+sp, "MDAXMLGetProgram");break;
        case MDAXMLSetProgram: sp+=sprintf(XMLCall+sp, "MDAXMLSetProgram");break;
        case MDAXMLSetMedianFilterStatus: sp+=sprintf(XMLCall+sp, "MDAXMLSetMedianFilterStatus");break;
        case MDAXMLGetMedianFilterStatus: sp+=sprintf(XMLCall+sp, "MDAXMLGetMedianFilterStatus");break;
        case MDAXMLSetMeanFilterStatus: sp+=sprintf(XMLCall+sp, "MDAXMLSetMeanFilterStatus");break;
        case MDAXMLGetMeanFilterStatus: sp+=sprintf(XMLCall+sp, "MDAXMLGetMeanFilterStatus");break;
        default: failed = true; break;
    }
    sp+=sprintf(XMLCall+sp, "</methodName>\r\n");
    switch(type)
    {
        case MDAXMLConnectCP: //first is the robot IP address, second is 1 for watchdog, 0 for no.
            sp+=sprintf(XMLCall+sp, "<params><param><value>%s</value></param><param><value><i4>%d</i4></value></param></params>\r\n", ROBOT_IP,0);
            break;
        case MDAXMLDisconnectCP:
            sp+=sprintf(XMLCall+sp, "<params><param><value>%s</value></param></params>\r\n", ROBOT_IP);
            break;
        case MDAXMLGetIP: break;
        case MDAXMLSetIP:
            sp+=sprintf(XMLCall+sp, "<params><param><value>%s</value></param></params>\r\n", ROBOT_IP);
            break;
        case MDAXMLGetSubNetmask: break;
//        case MDAXMLSetSubNetmask: break;
        case MDAXMLGetGatewayAddress: break;
//        case MDAXMLSetGatewayAddress: break;
        case MDAXMLGetDHCPMode: break;
//        case MDAXMLSetDHCPMode: break;
        case MDAXMLGetXmlPortCP: break;
//        case MDAXMLSetXmlPortCP: break;
        case MDAXMLGetTCPPortCP: break;
//        case MDAXMLSetTCPPortCP: break;
        case MDAXMLHeartBeat: break;
//        case xmlOPS_GetIORegister: break;
//        case xmlOPS_SetIORegister: break;
        case MDAXMLSetWorkingMode: //1 arg, 0 for off, 1 for on
            sp+=sprintf(XMLCall+sp, "<params><param><value><i4>%d</i4></value></param></params>",1);
            break;
        case MDAXMLGetFrontendData: break;
        case MDAXMLSetFrontendData:
            sp+=sprintf(XMLCall+sp, "<params><param><value><i4>%d</i4</value></param><param><value><i4>%d</i4</value></param><param><value><i4>%d</i4</value></param><param><value><i4>%d</i4</value></param><param><value><i4>%d</i4</value></param><param><value><i4>%d</i4</value></param><param><value><i4>%d</i4</value></param><param><value><i4>%d</i4</value></param></params>", 0,ModulationFrequencySetting,DoubleSampling,0,IntegrationTime1,IntegrationTime2,20,FrameMuteTime);
            break;
        case MDAXMLGetTrigger: break;
        case MDAXMLSetTrigger:
            if(FreeRunning) FreeRunningValue = 3;
            else            FreeRunningValue = 4;
            sp+=sprintf(XMLCall+sp,"<params><param><value><i4>%d</i4></value></param><param><value><i4>%d</i4></value></param><param><value><i4>%d</i4></value></param></params>",0,0,FreeRunningValue);
            break;
        case MDAXMLTriggerImage: break;
        case MDAXMLGetDebounceTrigger: break;
//        case MDAXMLSetDebounceTrigger: break;
        case MDAXMLGetAverageDetermination: break;
        case MDAXMLSetAverageDetermination:
            sp+=sprintf(XMLCall+sp, "<params><param><value><i4>%d</i4></value></param><param><value><i4>%d</i4></value></param><param><value><i4>%d</i4></value></param></params>",0,0,NumberOfImagesAverage);
            break;
        case MDAXMLGetProgram: break;
//        case MDAXMLSetProgram: break;
        case MDAXMLSetMedianFilterStatus:
            sp+=sprintf(XMLCall+sp, "<params><param><value><i4>%d</i4></value></param></params>",MedianFilter);
            break;
        case MDAXMLGetMedianFilterStatus: break;
        case MDAXMLSetMeanFilterStatus:
            sp+=sprintf(XMLCall+sp, "<params><param><value><i4>%d</i4></value></param></params>",MeanFilter);
            break;
        case MDAXMLGetMeanFilterStatus: break;
        default: failed = true; break;
    }
    sp+=sprintf(XMLCall+sp, "</methodCall>\r\n");
    //generate header
    sp2 = sprintf(Request, "POST /RPC2 HTTP/1.1\r\nUser-Agent: XMLRPC++ 0.7\r\nHost: %s:%d\r\nContent-Type: text/xml\r\nContent-length: %d\r\n\r\n",CAMERA_IP,SETTINGS_PORT,sp);
    sp2 += sprintf(Request+sp2,"%s",XMLCall);
    Request[sp2] = 0;
    return failed;
}

bool UFuncTOF::ParseResponse(char* Response, char* Result)
{
    char* RR;
    char* Rmin,*Rnow, *Rstop;
    int i, Imin = 0;
    int sp = 0;


    Rstop = strstr(Response,"</methodResponse>"); // find the end tag
    RR = strstr(Response,"<methodResponse>");//find the start tag
    if(Rstop == NULL || RR == NULL)
        return false;
    while(1)
    {        // keep looking for tags until no more is found
        Rmin = Rstop;
        for(i=0;i<NUMBER_OF_TAGS;i++)// look for the next tag by cycling through all the tags
        {
           Rnow = strstr(RR,PossibleTags[i]);
           if(Rnow != NULL && Rnow < Rmin)
           {
               Rmin = Rnow;
               Imin = i;
           }
        }
         // if no tag is found, stop loop
        if(Rmin == Rstop)
            break;
        // if next tag is a tag, find out what to do
        else
        {
            RR = strstr(Rmin,">")+1;//go to end of tag
//            printf("\r\n\"%s\"(%d) found[%c]: ",PossibleTags[Imin],Imin,RR[0]);
            switch(Imin)
            {
            case FAULT:// if fault write this in result
                sp += sprintf(Result+sp,"FAULT;");// +1;
                break;
            case VALUE: // write value to result
            case NAMEt: // write fault name to result
            case I4:    // write value to result
                if(RR[0] != '<')
                {
                 Rnow = strstr(RR,"<");
                 i = Rnow-RR;
                 memcpy(Result+sp,RR,i);
                 sp += i;
                 sp += sprintf(Result+sp,";");// finish with ';'
                }
                break;
            default: break;
            }
        }
        Result[sp] = 0;
    }
    //printf("\"%s\"\r\n",Result);
    return true;
}
/////////////////////////////////////////////////////////////////

void UFuncTOF::createResources()
{
    varIsOpen = addVar("open", 0.0, "d", "(r) is the camera open or closed");
//get images variables
    varGetDistance = addVar("GetDistance", 1.0, "d", "Requests the distance measurement.");;
    varGetIntensity = addVar("GetIntensity", 1.0, "d", "Requests the intensity measurement.");;
    varGetNormalX = addVar("GetNormalX", 0.0, "d", "Requests the direction vector X.");;
    varGetNormalY = addVar("GetNormalY", 0.0, "d", "Requests the direction vector Y.");;
    varGetNormalZ = addVar("GetNormalZ", 0.0, "d", "Requests the direction vector Z.");;
    varGetKartesianX = addVar("GetKartesianX", 0.0, "d", "Requests the distance converted to Kartesian X.");;
    varGetKartesianY = addVar("GetKartesianY", 0.0, "d", "Requests the distance converted to Kartesian Y.");;
    varGetKartesianZ = addVar("GetKartesianZ", 0.0, "d", "Requests the distance converted to Kartesian Z.");;
//settings variables
    varModulationFrequency = addVar("ModulationFrq", 0.0, "d", "Sets the modulation frequency, 0=23MHz, 1=20.4MHz, 2=20.6, 3=double frequency.");
    varDoubleSampling = addVar("DoubleSampling", 1.0, "d", "enables double integration, using two different integration times.");
    varIntegrationTime1 = addVar("IntegrationTime1", 100.0, "d", "Integration time 1, the integration time sets the maximum length.");
    varIntegrationTime2 = addVar("IntegrationTime2", 1000.0, "d", "Integration time 2, the integration time sets the maximum length.");
    varFrameMuteTime = addVar("FrameMuteTime", 70.0, "d", "Sets the non-transmition time, used for frame frequency control, and to limit camera heating.");
    varFreeRunning = addVar("FreeRunning", 0.0, "d", "Set to 0 to only get pictures with the \"ImageGet\" command. Set to 1 to continuously take pictures.");
    varNumberOfImagesAverage = addVar("NumberOfImagesAverage", 1.0, "d", "Sets the number of images to combine to a finished image.");
    varMeanFilter = addVar("MeanFilter", 0.0, "d", "Sets the mean filter numbers. (temporal filter)");
    varMedianFilter = addVar("MedianFilter", 0.0, "d", "Sets the median filter numbers. (spartial filter)");
  //image variables
    varCamDeviceNum = addVar("CamDeviceNum", 20.0, "d", "Camera device number.");
    varFramerate = addVar("Framerate", 0.0, "d", "Time between updates.");
    varUpdateCnt = addVar("updCnt", 0.0, "d", "(r) Number of updates.");

    varExportData = addVar("ExportData", 0.0, "d", "export picture data to a datafile");
}

///////////////////////////////////////////

void UFuncTOF::callGotNewDataWithObject()
{
  gotNewData(NULL);
}

/////////////////////////////////////////////////////
bool UFuncTOF::processImages(ImageHeaderInformation *Image)
{
    int i,j,x,y;
    int PoolNumber;
    int temp[4];
    float TimeDif;
    char* Ip = (char*)Image;

    //change big indian to small indian
    for(i = 0;i < (int)sizeof(*Image);i +=4)
    {
       for(j=0;j<4;j++)
         temp[j] = Ip[i+j];
       for(j=0;j<4;j++)
         Ip[i+j] = temp[3-j];
    }

//    printf("image time: %f / %f \r\n",Image->TimeStamp.Seconds,Image->TimeStamp.Useconds);

    //check for new frame
    if(LastTime.Seconds != Image->TimeStamp.Seconds || LastTime.Useconds != Image->TimeStamp.Useconds)
    {
        CurrentImageTime.now();
        TimeDif = (Image->TimeStamp.Seconds - LastTime.Seconds) + (Image->TimeStamp.Useconds - LastTime.Useconds) / 1000000;
        TimeDif = 1/TimeDif;
        varFramerate->setValued((int)TimeDif);
        varUpdateCnt->setValued(varUpdateCnt->getInt() + 1);
        LastTime.Seconds = Image->TimeStamp.Seconds;
        LastTime.Useconds = Image->TimeStamp.Useconds;
//        printf("new image: %d \r\n",(int)varUpdateCnt->getInt());
//        printf("frame: %f \r\n",TimeDif);
    }
/*    printf("image found(%d-%d)\r\n",i,sizeof(*Image));
    printf("data size: (%f+%f) %f\r\n", Image->DataSize, Image->HeaderSize, Image->HeaderSize+Image->DataSize);*/
//    printf("image type: (%f)\r\n", Image->ImageType);
//    printf("imageNumber: %d \r\n",(int)Image->Version);
    switch((int)Image->ImageType)
    {
    case 1://distance image
        PoolNumber = 70;
        ImagesMissing &=~0x01;
        break;
    case 3://intensity image
        PoolNumber = 71;
        ImagesMissing &=~0x02;
        break;
    case 5://normal x image
        PoolNumber = 72;
        ImagesMissing &=~0x04;
        break;
    case 6://normal y image
        PoolNumber = 73;
        ImagesMissing &=~0x08;
        break;
    case 7://normal z image
        PoolNumber = 74;
        ImagesMissing &=~0x10;
        break;
    case 8://kartesian x image
        PoolNumber = 75;
        ImagesMissing &=~0x20;
        break;
    case 9://kartesian y image
        PoolNumber = 76;
        ImagesMissing &=~0x40;
        break;
    case 10://kartesian z image
        PoolNumber = 77;
        ImagesMissing &=~0x80;
        break;
    default:
        return true;
    }

    //initialize image
    UImagePool * imgPool = (UImagePool *)getStaticResource("imgPool", false, false);
    UImage * depthBW = imgPool->getImage(PoolNumber, true);
    int16_t * DP;
    if (depthBW != NULL)
    {
        if (depthBW->tryLock())
        {
            depthBW->setSize(64,50, 1, 16, "BW16S");
            switch((int)Image->ImageType)
            {
            case 1:depthBW->setName("TimOfFlight distance value");break;
            case 3:depthBW->setName("TimOfFlight Reflection intensity value");break;
            case 5:
                depthBW->setName("TimOfFlight Normal X value");
                depthBW->getIplImage()->depth=IPL_DEPTH_16S;// allows for negative numbers
                break;
            case 6:
                depthBW->setName("TimOfFlight Normal Y value");
                depthBW->getIplImage()->depth=IPL_DEPTH_16S;// allows for negative numbers
                break;
            case 7:
                depthBW->setName("TimOfFlight Normal Z value");
                depthBW->getIplImage()->depth=IPL_DEPTH_16S;// allows for negative numbers
                break;
            case 8:
                depthBW->setName("TimOfFlight Kartesian X value");
                depthBW->getIplImage()->depth=IPL_DEPTH_16S;// allows for negative numbers
                break;
            case 9:
                depthBW->setName("TimOfFlight Kartesian Y value");
                depthBW->getIplImage()->depth=IPL_DEPTH_16S;// allows for negative numbers
                break;
            case 10:depthBW->setName("TimOfFlight Kartesian Z value");
                depthBW->getIplImage()->depth=IPL_DEPTH_16S; // allows for negative numbers
                break;
            }
            depthBW->imgTime = CurrentImageTime;
            depthBW->imageNumber = varUpdateCnt->getInt();
            depthBW->camDevice = varCamDeviceNum->getInt();
            depthBW->used = false;
            DP = (int16_t*)depthBW->getData();
            for(y=0;y<(64);y++)
            {
                for(x=0;x<(50);x++)
                {
                    *DP = (int16_t)(Image->Data[x*64+(y)]*1000);
                    if(Image->ImageType == 1 || Image->ImageType == 3) // if negative numbers are not allowed
                    {
                        if(*DP < 0) *DP = 0;
                    }
                    DP++;
                }
            }

            depthBW->updated();
            depthBW->unlock();
        }

        if(varExportData->getInt())
        {
            FILE * fh;
            char filenamr[100];
            snprintf(filenamr,100,"exportdata%d_%d.dat",(int)Image->ImageType,varUpdateCnt->getInt());
            fh = fopen(filenamr,"w");
            for(y=0;y<(64);y++)
            {
                for(x=0;x<(50);x++)
                {
                    fprintf(fh,"%f ",Image->Data[x*64+(y)]);

                }
                fprintf(fh,"\r\n");
            }
            fclose(fh);
        }
    }
    return false;
}

///////////////////////////////////////////////////
// C function used to start a thread (will not work with object function
void * startUFuncTOFThreadSetting(void * obj)
{ // call the hadling function in provided object
  UFuncTOF * ce = (UFuncTOF *)obj;
  ce->runSetting();
  pthread_exit((void*)NULL);
  return NULL;
}
void * startUFuncTOFThreadData(void * obj)
{ // call the hadling function in provided object
  UFuncTOF * ce = (UFuncTOF *)obj;
  ce->runData();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////
// thread start
bool UFuncTOF::start()
{
  bool result = true;
  pthread_attr_t  thAttr;
  //
  if (not (threadRunningData|threadRunningSetting))
  {
    pthread_attr_init(&thAttr);
    //
    threadStop = false;
    // create socket server thread
    result = (pthread_create(&threadHandleSetting, &thAttr,
              &startUFuncTOFThreadSetting, (void *)this) == 0);
    result  &= (pthread_create(&threadHandleData, &thAttr,
              &startUFuncTOFThreadData, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return result;
}

///////////////////////////////////////////////////
//thread stop
void UFuncTOF::stop(bool andWait)
{
  if (threadRunningSetting and not threadStop)
  { // stop and join thread
    threadStop = true;
    pthread_join(threadHandleSetting, NULL);
    if (threadRunningData)
    { // stop and join thread
      pthread_join(threadHandleData, NULL);
    }
  }
  if(andWait) while(threadRunningSetting || threadRunningData);
}

/////////////////////////////////////////////////////
// thread run function
void UFuncTOF::runSetting()
{
    // XML data

    // heartbeat
    UTime t, t2;
    // image request
    char ImageString[10];
    int StringCount;

    if (threadRunningSetting) // prevent nested calls;
        return;
    threadRunningSetting = true;
    t.now();
    while (not threadStop)
    {
    // check heartbeat / watchdog
        t2.now();
        if((t2.GetSec()-t.GetSec()) > 5)//time for heartbeat
        {
            if(SettingPort.isConnected())      // if port is open
            {
                if(callXMLRPC(MDAXMLHeartBeat) == false) //if errors, close ports
                {
                    SettingPort.closeConnection();
                    DataPort.closeConnection();

                }
//                else
//                    printf("{H}");
            }
            t.now(); // get new time
        }


    // check setting
        if(SettingPort.isConnected())      // if port is open
        {
            if(ModulationFrequencySetting != varModulationFrequency->getInt() ||
               DoubleSampling != varDoubleSampling->getInt() ||
               IntegrationTime1 != varIntegrationTime1->getInt() ||
               IntegrationTime2 != varIntegrationTime2->getInt() ||
               FrameMuteTime != varFrameMuteTime->getInt())
            {
               ModulationFrequencySetting = varModulationFrequency->getInt();
               DoubleSampling = varDoubleSampling->getInt();
               IntegrationTime1 = varIntegrationTime1->getInt();
               IntegrationTime2 = varIntegrationTime2->getInt();
               FrameMuteTime = varFrameMuteTime->getInt();
               callXMLRPC(MDAXMLSetFrontendData);
            }
            if(FreeRunning != varFreeRunning->getInt())
            {
               FreeRunning = varFreeRunning->getInt();
               callXMLRPC(MDAXMLSetTrigger);
               callXMLRPC(MDAXMLGetTrigger);
            }
            if(NumberOfImagesAverage != varNumberOfImagesAverage->getInt())
            {
               NumberOfImagesAverage = varNumberOfImagesAverage->getInt();
               callXMLRPC(MDAXMLSetAverageDetermination);
            }
            if(MeanFilter != varMeanFilter->getInt())
            {
               MeanFilter = varMeanFilter->getInt();
               callXMLRPC(MDAXMLSetMeanFilterStatus);
               callXMLRPC(MDAXMLGetMeanFilterStatus);
            }
            if(MedianFilter != varMedianFilter->getInt())
            {
               MedianFilter = varMedianFilter->getInt();
               callXMLRPC(MDAXMLSetMedianFilterStatus);
               callXMLRPC(MDAXMLGetMedianFilterStatus);
            }
        }

    //image request check
        if(SettingPort.isConnected())
        {
    // if no pictures waiting, and free running or trigger request, request images
            if((FreeRunning || TriggerImage) && ImagesMissing == 0)
            {
                ImageRequestTime.now();
    //check for image trigger
                if(TriggerImage && FreeRunning == 0)
                    callXMLRPC(MDAXMLTriggerImage);
                StringCount = 0;
                if(varGetDistance->getValued() != 0)
                {
                    ImagesMissing |=0x01;
                    ImageString[StringCount++] = 'd';
                }
                if(varGetIntensity->getValued() != 0)
                {
                    ImagesMissing |=0x02;
                    ImageString[StringCount++] = 'i';
                }
                if(varGetNormalX->getValued() != 0)
                {
                    ImagesMissing |=0x04;
                    ImageString[StringCount++] = 'e';
                }
                if(varGetNormalY->getValued() != 0)
                {
                    ImagesMissing |=0x08;
                    ImageString[StringCount++] = 'f';
                }
                if(varGetNormalZ->getValued() != 0)
                {
                    ImagesMissing |=0x10;
                    ImageString[StringCount++] = 'g';
                }
                if(varGetKartesianX->getValued() != 0)
                {
                    ImagesMissing |=0x20;
                    ImageString[StringCount++] = 'x';
                }
                if(varGetKartesianY->getValued() != 0)
                {
                    ImagesMissing |=0x40;
                    ImageString[StringCount++] = 'y';
                }
                if(varGetKartesianZ->getValued() != 0)
                {
                    ImagesMissing |=0x80;
                    ImageString[StringCount++] = 'z';
                }
                ImageString[0] += 'A'-'a'; // make the first char upper case; this is to ensure it gets the next image data
                ImageString[StringCount] = 0;
                if(StringCount > 0)
                {
                    if(DataPort.isConnected())
                    {
                        if(DataPort.blockSend(ImageString, StringCount) == -1)
                            printf("Send error!(%s)\r\n", ImageString);
                    }
                }
                TriggerImage = false;

            }
        }
    }
    threadRunningSetting = false;
}
void UFuncTOF::runData()
{
    // image data
    char ImageBuffer[sizeof(ImageHeaderInformation)];
    ImageHeaderInformation *CurrentImage;
    int sp = 0,sp2;
    // image timeout
    UTime t;

    if (threadRunningData) // prevent nested calls;
        return;
    threadRunningData = true;

    CurrentImage = (ImageHeaderInformation*)ImageBuffer;
    while (not threadStop)
    {
    //check for new data, only get data enough for one image
        if(DataPort.isConnected())
        {
            sp2 = DataPort.getDataFromLine(ImageBuffer+sp,sizeof(ImageHeaderInformation)-sp,5);
            if(sp2 == -1)
                DataPort.closeConnection();
            else
            {
                sp += sp2;
    // if enough data process image
                if(sp>=(int)sizeof(ImageHeaderInformation))
                {
                    processImages(CurrentImage);
    // removed processed data
                    sp = 0;
                }
            }
        }
   //check for image request timeout
        t.now();
        if(ImagesMissing != 0 && (t.GetSec()-ImageRequestTime.GetSec()) > 4)
        {
            t.now();
            ImagesMissing = 0;
            printf("\"imageget\" timeout");
        }
    }
    threadRunningData = false;
}
