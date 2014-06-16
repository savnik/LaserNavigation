#include "lab.h"
#include <string.h>

/*******************************************************************/
/* Title    : KlikFrame                                            */
/* Funktion : Gets the labyrinth frame-corners by clicking the     */
/*            mouse.                                               */
/*                                                                 */
/* Dependencies :  VisInstruktion                                  */
/* In       : frame     : Structure holding the frame-corners.     */
/*            maskplane : The maskplane in which to draw frame.    */
/*            colour    : The drawing colour.                      */
/* Out      :  --                                                  */
/* Remark   :  --                                                  */
/* Ref.     :  --                                                  */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 18/11 91                                             */
/*******************************************************************/

extern int VisInstruktion (char *msg1, char *msg2);
/*
KlikFrame(POLYGON *frame, int maskplane, int colour)
{
        short int x_off,y_off,i,last[2],stop;
        short int x,y,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5;
        char  inst1[80], inst2[80];

        Cfillpl(maskplane,BLACK);

        stop = FALSE;
        i = 0;
        while( (i < frame->polysize) && !stop )
        {
                        switch(i)
                        {
                                case 0 :
                                        strcpy(inst1,"Marker rammens \34verste venstre hj\34rne med venstre musetast, afbryd med h\34jre");
                                        strcpy(inst2,"Mark the frames upper left corner with left mousebotton, end with right");
                                        last[0] = last[1] = 0;
                                        x_off = y_off = 20;
                                        break;
                                case 1 :
                                        strcpy(inst1,"Marker rammens \34verste h\34jre hj\34rne med venstre musetast, afbryd med h\34jre");
                                        strcpy(inst2,"Mark the frames upper right corner with left mousebotton, end with right");
                                        last[0] = last[1] = 0;
                                        x_off = 0; y_off = 20;
                                        break;
                                case 2 :
                                        strcpy(inst1,"Marker rammens nederste venstre hj\34rne med venstre musetast, afbryd med h\34jre");
                                        strcpy(inst2,"Mark the frames lower left corner with left mousebotton, end with right");
                                        last[0] = last[1] = 0;
                                        x_off = 20; y_off = 0;
                                        break;
                                case 3 :
                                        strcpy(inst1,"Marker rammens nederste h\34jre hj\34rne med venstre musetast, afbryd med h\34jre");
                                        strcpy(inst2,"Mark the frames lower right corner with left mousebotton, end with right");
                                        last[0] = 2 ; last[1] = 1;
                                        x_off = y_off = 0;
                                        break;
                        }

                        x1 = y1 = x4 = y4 = x5 = y5 = 0;
                        VisInstruktion(inst1,inst2);
                        Cplotmp(maskplane);

                while(!Fmouser() && !Fmousel());
                if(Fmouser())
                {
                        stop = TRUE;
                        while(Fmouser());
                }
                else
                {
                        while(Fmousel())
                        {
                                x = Fmousex();
                                y = Fmousey();
                                if((x != x1) || (y != y1))
                                {
                                        if(x_off == 0)
                                        {
                                                x2 = frame->polypoint[last[0]].x;
                                                y2 = frame->polypoint[last[0]].y;
                                        }
                                        else
                                        {
                                                x2 = x + x_off;
                                                y2 = y;
                                        }
                                        if(y_off == 0)
                                        {
                                                x3 = frame->polypoint[last[1]].x;
                                                y3 = frame->polypoint[last[1]].y;
                                        }
                                        else
                                        {
                                                x3 = x;
                                                y3 = y + y_off;
                                        }

                                        Ccolour(BLACK);
                                        Cline(x1,y1,x4,y4);
                                        Cline(x1,y1,x5,y5);
                                        Ccolour(colour);
                                        Cline(x,y,x2,y2);
                                        Cline(x,y,x3,y3);
                                        x4 = x2;  y4 = y2;
                                        x5 = x3;  y5 = y3;
                                        x1 = x;   y1 = y;
                                }
                        }
                        Ccolour(BLACK);
                        if(x_off>0) Cline(x1,y1,x4,y4);
                        if(y_off>0) Cline(x1,y1,x5,y5);
                        frame->polypoint[i].x = x;
                        frame->polypoint[i].y = y;
                        i++;
                }
        }
        if(i==4) return TRUE;
        else return FALSE;
}
*/

KlikFrame(POLYGON *frame, int maskplane, int colour)
{
        short int x_off,y_off,i,last[2],stop;
        short int x,y,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5;
        char  inst1[80], inst2[80];

        Cfillpl(maskplane,BLACK);

        stop = FALSE;
        i = 0;
        while( (i < frame->polysize) && !stop )
        {
                        switch(i)
                        {
                                case 0 :
                                        strcpy(inst1,"Marker rammens \34verste venstre hj\34rne med venstre musetast, afbryd med h\34jre");
                                        strcpy(inst2,"Mark the frames upper left corner with left mousebotton, end with right");
                                        last[0] = last[1] = 0;
                                        x_off = y_off = 20;
                                        break;
                                case 1 :
                                        strcpy(inst1,"Marker rammens \34verste h\34jre hj\34rne med venstre musetast, afbryd med h\34jre");
                                        strcpy(inst2,"Mark the frames upper right corner with left mousebotton, end with right");
                                        last[0] = last[1] = 0;
                                        x_off = 0; y_off = 20;
                                        break;
                                case 2 :
                                        strcpy(inst1,"Marker rammens nederste venstre hj\34rne med venstre musetast, afbryd med h\34jre");
                                        strcpy(inst2,"Mark the frames lower left corner with left mousebotton, end with right");
                                        last[0] = last[1] = 0;
                                        x_off = 20; y_off = 0;
                                        break;
                                case 3 :
                                        strcpy(inst1,"Marker rammens nederste h\34jre hj\34rne med venstre musetast, afbryd med h\34jre");
                                        strcpy(inst2,"Mark the frames lower right corner with left mousebotton, end with right");
                                        last[0] = 2 ; last[1] = 1;
                                        x_off = y_off = 0;
                                        break;
                        }

                        x1 = y1 = x4 = y4 = x5 = y5 = 0;
                        VisInstruktion(inst1,inst2);
                        Cplotmp(maskplane);

               
                if(Fmouser())
                {
                        stop = TRUE;
                        while(Fmouser());
                }
                else
                {

                    mark_point(maskplane);    
                    x = Fmousex();
                    y = Fmousey();
                    frame->polypoint[i].x = x;
                    frame->polypoint[i].y = y;
                    i++;
                }
        }
        if(i==4) return TRUE;
        else return FALSE;
}



/*******************************************************************/
/* Title    : FrameEdge                                            */
/* Funktion : Find an array of borderpoints for the labyrinth frame*/
/*                                                                 */
/* Dependencies :  --                                              */
/* In       : frame  : Structure holding the frame-corners.        */
/*            edge   : Array holding the frame-borders.            */
/* Out      :  --                                                  */
/* Remark   :  --                                                  */
/* Ref.     :  --                                                  */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 18/11 91                                             */
/*******************************************************************/
FrameEdge(POLYGON *frame, short int (*edge)[2])
{
        short int c, r, left;
        UBYTE *pt, *pt0;

        Cfillpl(PLANE1,MIN_GREY);
        Ccolour(MAX_GREY);
        Cplotmp(PLANE1);
        Cline(frame->polypoint[0].x, frame->polypoint[0].y
                , frame->polypoint[1].x, frame->polypoint[1].y);
        Cline(frame->polypoint[0].x, frame->polypoint[0].y
                , frame->polypoint[2].x, frame->polypoint[2].y);
        Cline(frame->polypoint[1].x, frame->polypoint[1].y
                , frame->polypoint[3].x, frame->polypoint[3].y);
        Cline(frame->polypoint[2].x, frame->polypoint[2].y
                , frame->polypoint[3].x, frame->polypoint[3].y);
        for(r=0;r<ROWS;r++)
                edge[r][0] = edge[r][1] = 0;
        pt0 = (UBYTE *)Fgetmp(PLANE1);
        for(r=0;r<ROWS;r++)
        {
                left = FALSE;
                for(c=1;c<COLS-1;c++)
                {
                        pt = pt0 + (int)c + (int)r*ROW_FACTOR;
                        if( !left && (*pt == MAX_GREY) && (*(pt+1) == MIN_GREY) )
                        {
                                edge[r][0] = c;
                                left = TRUE;
                        }
                        if( left && (*pt == MIN_GREY) && (*(pt+1) == MAX_GREY) )
                                edge[r][1] = c;
                }
        }
}                               


