#include "lab.h"
#include <math.h>
#include <string.h>
/********************************************************************/
/* Navn       : DrawRect                                            */
/* Funktion   : Tegner rect                                         */
/* Kald       : -                                                   */
/* Ind        : *myRect    - pointer til rect som skal tegnes       */
/*              maskPlane  - det benyttede plot maskplane           */
/*              colour     - plot farven                            */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen, modificeret af Soren J Jensen*/
/* Dato       :                                                     */
/********************************************************************/

extern int FrameEdge (POLYGON *frame, short int (*edge)[2]);
extern int FindTrackPoint (int *x, int *y, int *itot, short int *minpt, short int (*ref)[2]);
extern int Conv_track_draw (short int (*ref)[2], double *aspect, double *c, double *s);
extern int FindTrackPart (int *x, int *y, int *itot, short int *minpt, short int (*ref)[2]);
extern int VisInstruktion (char *msg1, char *msg2);
int Message (char *msg, int colour, int *texty, int length, int maskplane);

DrawRect(POLYGON *myRect, int maskPlane, int colour)
{
  Cplotmp( maskPlane );
  Ccolour( colour );
  Cline(myRect->polypoint[0].x,myRect->polypoint[0].y
        ,myRect->polypoint[1].x,myRect->polypoint[1].y);
  Cline(myRect->polypoint[0].x,myRect->polypoint[0].y
        ,myRect->polypoint[2].x,myRect->polypoint[2].y);
  Cline(myRect->polypoint[1].x,myRect->polypoint[1].y
        ,myRect->polypoint[3].x,myRect->polypoint[3].y);
  Cline(myRect->polypoint[2].x,myRect->polypoint[2].y
        ,myRect->polypoint[3].x,myRect->polypoint[3].y);
}


/********************************************************************/
/* Navn       : MaskPlane                                           */
/* Funktion   : Genererer maskplane til brug ved kald af ScanBeam-  */
/*              rutiner.                                            */
/* Kald       : -                                                   */
/* Ind        : mask  - skrivemaske                                 */
/*              plane - skrive-'plane'                              */
/* Ud         : maskplane svarende til 'mask' og 'plane'            */
/* Reference  : ScanBeam firmware-manual side 6                     */
/* Skrevet af : Allan Theill Sorensen                               */
/* Dato       : 3/7 1990                                            */
/********************************************************************/
MaskPlane(int mask, int plane)
{
  return (mask<<8) + plane;
}



/********************************************************************/
/* Title    :  FrameConst                                           */
/* Funktion :  Calculates different constants for the labyrinth     */
/*             frame.                                               */
/*                                                                  */
/* Dependencies :  FrameEdge                                        */
/* In       :  frame  : The labyrinths frame corners                */
/*             width  : The width of the frame                      */
/*             aspect : The cameras Aspect Ratio                    */
/*             edge   : Array holding the borders of the frame      */
/*             c, s   : Cos and Sin to the slope of the labyrinth   */
/* Out      :   --                                                  */
/* Remark   :   --                                                  */
/* Ref.     :   --                                                  */
/* Made by  :  Soeren Juhl Jensen                                   */
/* Date     :  18/11 91                                             */
/********************************************************************/
FrameConst(POLYGON *frame, int *width, double *aspect, short int (*edge)[2], double *c, double *s)
{
   double x0, y0, ang, wid;

   x0 = frame->polypoint[1].x - frame->polypoint[0].x;
   y0 = (*aspect)*(frame->polypoint[1].y - frame->polypoint[0].y);
   wid = sqrt(x0*x0 + y0*y0);
   ang = acos(x0/wid);
   if(y0 > 0) ang = -ang;
   *c = cos(ang);
   *s = sin(ang);
   *width = (int)wid;
   FrameEdge(frame, edge);
}


/********************************************************************/
/* Title    :  ChangeTrack                                          */
/* Funktion :  Changes a point on the labyrinth track by clicking   */
/*             the mouse.                                           */
/*                                                                  */
/* Dependencies :  FindTrackPoint   Conv_track_draw                 */
/* In       : frame  : The labyrinth frame corners                  */
/*            itot   : The number of points on the track            */
/*            ref    : Array holding the track points               */
/*            aspect : The cameras Aspect Ratio                     */
/*            c, s   : Cos and Sin to the labyrinth slope           */
/* Out      :  --                                                   */
/* Remark   :  --                                                   */
/* Ref.     :  --                                                   */
/* Made by  : Soeren Juhl Jensen                                    */
/* Date     : 18/11 91                                              */
/********************************************************************/
ChangeTrack(POLYGON *frame, int *itot, short int (*ref)[2], double *aspect, double *c, double *s)
{
   int x,y,x0,y0,x1,y1,x2,y2,x3,y3;
   short int i, tref[MAXPUNKT][2];

   Cplotmp(PLANE4);
   x0 = frame->polypoint[0].x;
   y0 = frame->polypoint[0].y;
   mark_point(PLANE4);
   x = Fmousex() - x0;
   y = Fmousey() - y0;
   x1 = (*c)*x - (*s)*y*(*aspect);
   y1 = (*s)*x + (*c)*y*(*aspect);
   FindTrackPoint(&x1,&y1,itot,&i,ref);
   if(i != (*itot))
   {
      x1 = ref[i+1][0];
      y1 = ref[i+1][1];
   }
   else
      { x1 = ref[i-1][0]; y1 = ref[i-1][1]; }
   x2 = (*c)*x1 + (*s)*y1*(*aspect);
   y2 = -(*s)*x1 + (*c)*y1*(*aspect);
   if(i != 0)
   {
      x1 = (int)ref[i-1][0];
      y1 = (int)ref[i-1][1];
      x3 = (*c)*x1 + (*s)*y1*(*aspect);
      y3 = -(*s)*x1 + (*c)*y1*(*aspect);
   }
   else
      { x3 = x2; y3 = y2; }
   x1 = ref[i][0]; y1 = ref[i][1];
   while(Fmousel())
   {
      x = Fmousex() - x0;
      y = Fmousey() - y0;
      if((x != x1) || (y != y1))
      {
         Ccolour(BLACK);
         Cline(x2+x0,y2+y0,x1+x0,y1+y0);
         Cline(x3+x0,y3+y0,x1+x0,y1+y0);
         Ccolour(GREEN);
         Cline(x2+x0,y2+y0,x+x0,y+y0);
         Cline(x3+x0,y3+y0,x+x0,y+y0);
         x1 = x; y1 = y;
      }
   }
   x1 = (*c)*x - (*s)*y*(*aspect);
   y1 = (*s)*x + (*c)*y*(*aspect);
   ref[i][0] = x1;  ref[i][1] = y1;
   Cfillpl(PLANE4,BLACK);
   for(i=0;i<MAXPUNKT;i++)
   {
       tref[i][0] = ref[i][0];
       tref[i][1] = ref[i][1];
   }
   Conv_track_draw(tref,aspect,c,s);
   i = 1;
   while(tref[i][0] != -1)
   {
      Cline(x0+tref[i][0],y0+tref[i][1],
            x0+tref[i-1][0],y0+tref[i-1][1]);
      i++;
   }
}


/********************************************************************/
/* Title    :  NewTrackPoint                                        */
/* Funktion :  Makes a new point on the labyrinth track by          */
/*             clicking the mouse.                                  */
/*                                                                  */
/* Dependencies :  FindTrackPart   Conv_track_draw                  */
/* In       : frame  : The labyrinth frame corners                  */
/*            itot   : The number of points on the track            */
/*            ref    : Array holding the track points               */
/*            aspect : The cameras Aspect Ratio                     */
/*            c, s   : Cos and Sin to the labyrinth slope           */
/* Out      :  --                                                   */
/* Remark   :  --                                                   */
/* Ref.     :  --                                                   */
/* Made by  : Soeren Juhl Jensen                                    */
/* Date     : 18/11 91                                              */
/********************************************************************/
NewTrackPoint(POLYGON *frame, int *itot, short int (*ref)[2], double *aspect, double *c, double *s)
{
   int x,y,x0,y0,x1,y1,x2,y2,x3,y3;
   short int i, j, tref[MAXPUNKT][2];

   Cplotmp(PLANE4);
   x0 = frame->polypoint[0].x;
   y0 = frame->polypoint[0].y;
   while(!Fmousel());
   x = Fmousex() - x0;
   y = Fmousey() - y0;
   x1 = (*c)*x - (*s)*y*(*aspect);
   y1 = (*s)*x/(*aspect) + (*c)*y;
   FindTrackPart(&x1,&y1,itot,&i,ref);
   x1 = ref[i][0];
   y1 = ref[i][1];
   x2 = (*c)*x1 + (*s)*y1*(*aspect);
   y2 = -(*s)*x1/(*aspect) + (*c)*y1;
   x1 = ref[i+1][0];
   y1 = ref[i+1][1];
   x3 = (*c)*x1 + (*s)*y1*(*aspect);
   y3 = -(*s)*x1/(*aspect) + (*c)*y1;
   x1 = ref[i][0]; y1 = ref[i][1];
   while(Fmousel())
   {
      x = Fmousex() - x0;
      y = Fmousey() - y0;
      if((x != x1) || (y != y1))
      {
         Ccolour(BLACK);
         Cline(x2+x0,y2+y0,x1+x0,y1+y0);
         Cline(x3+x0,y3+y0,x1+x0,y1+y0);
         Ccolour(GREEN);
         Cline(x2+x0,y2+y0,x+x0,y+y0);
         Cline(x3+x0,y3+y0,x+x0,y+y0);
         x1 = x; y1 = y;
      }
   }
   x1 = (*c)*x - (*s)*y*(*aspect);
   y1 = (*s)*x/(*aspect) + (*c)*y;
   for(j=(*itot)+2;j>i+1;j--)
   {
      ref[j][0] = ref[j-1][0];
      ref[j][1] = ref[j-1][1];
   }
   ref[i+1][0] = x1;
   ref[i+1][1] = y1;
   (*itot)++;
   Cfillpl(PLANE4,BLACK);
   for(i=0;i<MAXPUNKT;i++)
   {
      tref[i][0] = ref[i][0];
      tref[i][1] = ref[i][1];
   }
   Conv_track_draw(tref,aspect,c,s);
   i = 1;
   while(tref[i][0] != -1)
   {
      Cline(x0+tref[i][0],y0+tref[i][1],
            x0+tref[i-1][0],y0+tref[i-1][1]);
      i++;
   }
}

/*********************************************************************
*                                                                    *
* Funktionen drawstring(x,y,str,a1,a2,a3) udskriver strengen givet   *
* ved "str" paa                                                      *
* koordinaterne x,y pa monitoren. Skrivemaske og plan kan saettes    *
* med ved kald                                                       *
* af Ccharmp. Udskriften sker formatteret, saaledes at               *
* integervaerdierne "a1"                                             *
* "a2" og "a3" kan indflettes i strengen, hvor der i denne er et %d  *
* jvf kapitel                                                        *
* 7.3 i KR om formatteret output (Printf) side 145.                  *
*                                                                    *
* Programmeret af: Kaj H. Jensen                                     *
*********************************************************************/
drawstring(int x, int y, char *str, int a1, int a2, int a3)
       /* a1, a2 og a3 kan have forskellige typer fra kald til kald */
            
             
{
        char    temp[256],*s;
        Fsprintf(temp,str,a1,a2,a3);
        Ccharxy(x,y);
        s = temp;
        while (*s) Ccharput(*s++);
}


drawstring4(int x, int y, char *str, int a1, int a2, int a3)
       /* a1, a2 og a3 kan have forskellige typer fra kald til kald */
            
             
{
        char    temp[256],*s;
        Ccharmp(PLANE4);
        Fsprintf(temp,str,a1,a2,a3);
        Ccharxy(x,y);
        s = temp;
        while (*s)
                Ccharput(*s++);
}



/********************************************************************/
/* Title    :  Help                                                 */
/* Funktion :  Writes a help screen to the Scan Beam terminal.      */
/*                                                                  */
/* Dependencies :  VisInstruktion   Message                         */
/* In       : language : The actual language                        */
/* Out      :  --                                                   */
/* Remark   :  --                                                   */
/* Ref.     :  --                                                   */
/* Made by  : Soeren Juhl Jensen                                    */
/* Date     : 22/11 91                                              */
/********************************************************************/
Help(int *language)
{
   int texty;
   char msg[2000];

   VisInstruktion("Tryk h\34jre musetast for at afslutte",
                  "Press right mousebotton to get back");
   Cfillpl(PLANE0,200);
   texty = MESSAGE_Y;
   Cfillpl(PLANE4,BLACK);
   if(*language == DANSK)
   {
      strcpy(msg,"   For at starte labyrinten, skal systemet informeres ");
      strcat(msg,"om labyrintens ramme, samt om ruten p\35 labyrintpladen.@");
      strcat(msg,"   Informationen om labyrintens ramme kan f\35s ved at ");
      strcat(msg,"klikke rammen med musen, eller ved at bede systemet om ");
      strcat(msg,"at finde den automatisk.@");
      strcat(msg,"   Hvis systemet skal kunne finde rammen automatisk, skal ");
      strcat(msg,"det have information om labyrintens huller. ");
      strcat(msg,"Hullerne kan enten klikkes ind med musen eller indl\33ses ");
      strcat(msg,"fra disk.@   Ruten kan enten klikkes ind med musen eller ");
      strcat(msg,"indl\33ses fra disk.@   Hvis ruten under k\34rsel viser ");
      strcat(msg,"sig, at v\33re uhensigtsm\33ssig p\35 visse steder, kan ");
      strcat(msg,"den \33ndres i rute-menuen, hvorefter labyrinten kan ");
      strcat(msg,"opstartes igen.");
      Message(msg,BLUE,&texty,42,PLANE4);
      texty += 2*LINEHEIGHT;
      strcpy(msg,"Klik venstre musetast for n\33ste side !");
      Message(msg,RED,&texty,42,PLANE4);
      while(!Fmouser() && !Fmousel());
      if(Fmousel())
      {
         while(!Fmousel());
         while(Fmousel());
         texty = MESSAGE_Y;
         Cfillpl(PLANE4,BLACK);
         strcpy(msg,"   Der findes under menu-punktet 'Start Labyrint' en ");
         strcat(msg,"mulighed for at foretage opstart af labyrinten automatisk, ");
         strcat(msg,"uden forudg\35ende information om systemet. Dette kr\33ver ");
         strcat(msg,"dog, at rute og huller ligger p\35 disk, og at systemet er ");
         strcat(msg,"istand til, at finde rammen. Dette punkt kan ogs\35 ");
         strcat(msg,"benyttes, hvis systemet i forvejen f.eks. kender rammen.@");
         strcat(msg,"   Under menu-punktet 'Parametre' kan nye parametre ");
         strcat(msg,"v\33lges til selve labyrint reguleringen, eller forskellige ");
         strcat(msg,"ops\33tninger af systemet kan \33ndres.");
         Message(msg,BLUE,&texty,42,PLANE4);
         texty += 2*LINEHEIGHT;
         strcpy(msg,"SLUT");
         Message(msg,RED,&texty,42,PLANE4);
      }
   }
   else if(*language == ENGLISH)
   {
      strcpy(msg,"   To start the labyrinth, the system must have information ");
      strcat(msg,"about the labyrinth frame, and the track.@");
      strcat(msg,"   The information about the frame can be obtained either ");
      strcat(msg,"by clicking the mouse or by letting the system search it ");
      strcat(msg,"automaticaly.@   If the system is to find the frame ");
      strcat(msg,"automaticaly,it must have information about the holes ");
      strcat(msg,"of the labyrinth. The holes can either be clicked by the ");
      strcat(msg,"mouse or be read from disk.@   The track can either be ");
      strcat(msg,"clicked by teh mouse or be read from disk.@   If the track ");
      strcat(msg,"during running shows to be inconvenient at some places, ");
      strcat(msg,"it can be changed in the Track-menu, whereafter the ");
      strcat(msg,"labyrinth can be started again.");
      Message(msg,BLUE,&texty,42,PLANE4);
      texty += 2*LINEHEIGHT;
      strcpy(msg,"Click left mousebotton for next page !");
      Message(msg,RED,&texty,42,PLANE4);
      while(!Fmouser() && !Fmousel());
      if(Fmousel())
      {
         while(!Fmousel());
         while(Fmousel());
         texty = MESSAGE_Y;
         Cfillpl(PLANE4,BLACK);
         strcpy(msg,"   There is, under the menu-point 'Start Labyrinth' ");
         strcat(msg,"a possibility of starting the labyrinth automaticaly, ");
         strcat(msg,"without any knowledge about the system. This however ");
         strcat(msg,"requires, that the holes and the track is located on ");
         strcat(msg,"the disk, and that the system is able to find the frame. ");
         strcat(msg,"This point can also be used if the system forinstance ");
         strcat(msg,"already has got information about the frame.@");
         strcat(msg,"   Under the menu-point 'Parameters' new parameters for ");
         strcat(msg,"the labyrinth regulation can be chosen, or different ");
         strcat(msg,"system-setup modes can be changed.");
         Message(msg,BLUE,&texty,42,PLANE4);
         texty += 2*LINEHEIGHT;
         strcpy(msg,"THE END");
         Message(msg,RED,&texty,42,PLANE4);
      }
   }
   while(!Fmouser());
   while(Fmouser());
   Cfillpl(PLANE4,BLACK);
   Cscanfrm();
   Cfillmp(PLANE5);
}



/********************************************************************/
/* Title    :  Message                                              */
/* Funktion :  Writes a message to a given maskplane and in a       */
/*             given colour. The message is seperated at spaces     */
/*             with a maximum linelength.                           */
/*             NB : @ in msg-string means change to new line        */
/*                                                                  */
/* Dependencies :  drawstring                                       */
/* In       : msg       : The message to be written                 */
/*            colour    : The given colour                          */
/*            texty     : The y-position to start the text          */
/*            length    : The given linelength                      */
/*            maskplane : The given maskplane                       */
/* Out      :  --                                                   */
/* Remark   :  --                                                   */
/* Ref.     :  --                                                   */
/* Made by  : Soeren Juhl Jensen                                    */
/* Date     : 22/11 91                                              */
/********************************************************************/
Message(char *msg1, int colour, int *texty, int length, int maskplane)
{
   int i, i1, dum;
   char msg[256];
   
   strcpy(msg,msg1);
   Cplotmp( maskplane );
   Ccharmp( maskplane );
   Ccharcol( BLACK, colour );
   Ccharxy(MARGIN,*texty);

   i = i1 = 0;
   while(strlen(msg+i1) >= length)
   {
      for(dum=i1;dum<i1+length;dum++)
         if(*(msg+dum) == '@') i = dum;
      if(i == i1)
      {
         i = i1 + length;
         while(*(msg+i) != ' ')
            i--;
      }
      *(msg+i) = '\0';
      drawstring( MARGIN, *texty += LINEHEIGHT, msg+i1,0,0,0);
      *(msg+i) = ' ';
      i1 = ++i;
   }
   drawstring(MARGIN, *texty += LINEHEIGHT, msg+i1,0,0,0);
}


/********************************************************************/
/* Navn       : JusterLab                                           */
/* Funktion   : Scanner indtil hojre musetast tastes                */
/* Kald       : -                                                   */
/* Ind        : -                                                   */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/* Dato       : 9/7 1990                                            */
/********************************************************************/
JusterLab(void)
{
  VisInstruktion("Slut scanning med h\34jre musetast",
                 "End scanning with right mousebotton");
  Cscanon();
  while(!Fmouser());
  Cscanoff();
  Cscanfrm();
  while(Fmouser());
}

