#include "lab.h"
#include "stdlib.h"

/*******************************************************************/
/* Title    : KlikHoles                                            */
/* Funktion : Gets the holes by clicking the mouse.                */
/*                                                                 */
/* Dependencies : VisInstruktion                                   */
/* In       : hole_ref : Array holding the holes.                  */
/*            maskplane, colour : the plot plane and colour.       */
/* Out      :  --                                                  */
/* Remark   :  --                                                  */
/* Ref.     :  --                                                  */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 18/11 91                                             */
/*******************************************************************/

extern int VisInstruktion (char *msg1, char *msg2);
int Find_area (short int (*hole)[2], int *max_i);
int Ajust_holes (short int (*hole)[2], int *maxi);
int Moment (RECT *area, short int (*hole)[2], int *max_i);

KlikHoles(short int (*hole_ref)[2], int maskplane, int colour)
{
	short int stop, x, y, x1, y1;
	int i;

	VisInstruktion("Klik huller med venstre musetast, afslut med h\34jre",
	               "Click holes with left mousebotton, end with right");
	Cplotmp(maskplane);
	Cfillmp(maskplane);
	Ccolour(colour);

	stop = FALSE;
	i = 0;
	while(!stop)
	{
		if(Fmouser())
		{
			stop = TRUE;
			while(Fmouser());
		}
		else if(Fmousel())
		{
			x1 = y1 = COLS/2;
			while(Fmousel())
			{
				x = (short int)Fmousex();
				y = (short int)Fmousey();
				if((x != x1) || (y != y1))
				{
					Ccolour(BLACK);
					Ccircle(x1,y1,6);
					Ccolour(colour);
					Ccircle(x,y,6);
					x1 = x;
					y1 = y;
				}
			}
			hole_ref[i][0] = x;
			hole_ref[i][1] = y;
			i++;
		}
	}
	hole_ref[i][0] = -1;
	hole_ref[i][1] = 0;
	return ( i );
} 


/********************************************************************/
/* Navn       : DrawHoles                                           */
/* Funktion   : Skalerer og tegner rute i hukommelsen.              */
/*              Benyttes efter en rute er laest ind fra disk.       */
/* Kald       : -                                                   */
/* Ind        : ref[][2]  - array med knaekpunkter for ruten        */
/*              x0        - x-koordinat paa rammens oeverste hoejre */
/*                          hjoerne.                                */
/*              y0        - y-koordinat paa rammens oeverste hoejre */
/*                          hjoerne.                                */
/*              nyBredde  - Bredde af den aktuelle ramme. Ruten     */
/*                          skaleres vha. denne bredde, samt bred-  */
/*                          den af rammen da ruten blev gemt paa    */
/*                          disk.                                   */
/*              maskPlane - maskplane til grafikrutiner             */
/*              colour    - farve til grafikrutiner                 */
/* Ud         : indeks til sidste punkt i ruten                     */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/* Dato       : 11/7 1990                                           */
/********************************************************************/
DrawHoles(short int (*ref1)[2], short int (*ref2)[2], int x0, int y0, int nyBredde, int maskPlane, int colour)
{
  int i;
  double glBredde, faktor;

  Cfillpl( maskPlane, BLACK );
  Cplotmp( maskPlane );
  Ccolour( colour );

  i = 0;
  while( ref1[i][0] != -1 ) i++;
  glBredde = (int)ref1[i][1];
  faktor = nyBredde/glBredde;

  ref1[0][0] = (short int)((double) ref1[0][0] * faktor);
  ref1[0][1] = (short int)((double) ref1[0][1] * faktor);
  ref2[0][0] = (short int)((double) ref2[0][0] * faktor);
  ref2[0][1] = (short int)((double) ref2[0][1] * faktor);
  Ccircle(x0+ref2[0][0], y0+ref2[0][1], 6);
  i = 1;
  while( ref1[i][0] != -1 )
  {
    ref1[i][0] = (short int)((double) ref1[i][0] * faktor);
    ref1[i][1] = (short int)((double) ref1[i][1] * faktor);
    ref2[i][0] = (short int)((double) ref2[i][0] * faktor);
    ref2[i][1] = (short int)((double) ref2[i][1] * faktor);
    Ccircle(x0+ref2[i][0], y0+ref2[i][1], 6);
    i++;
  }
  ref1[i][1] = (short int)nyBredde;
  ref2[i][1] = (short int)nyBredde;
  return i-1;
}



/*******************************************************************/
/* Title    : Conv_holes_save                                      */
/* Funktion : Converts the holes so that they can be saved in the  */
/*            propper format.                                      */
/*                                                                 */
/* Dependencies :  --                                              */
/* In       : ref    : Arrray holding the track points             */
/*            aspect : The cameras Aspect Ratio                    */
/*            c, s   : Cos and Sin to the labyrinths slope         */
/* Out      :  --                                                  */
/* Remark   :  --                                                  */
/* Ref.     :  --                                                  */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 18/11 91                                             */
/*******************************************************************/
Conv_holes_save(short int (*ref)[2], double *aspect, double *c, double *s)
{
   double x0,y0;
   int i;

   i = 0;
   while(ref[i][0] != -1)
   {
      x0 = (*c)*ref[i][0] - (*s)*ref[i][1]*(*aspect);
      y0 = (*s)*ref[i][0]/(*aspect) + (*c)*ref[i][1];
      ref[i][0] = x0;
      ref[i][1] = y0;
      i++;
   }
}


/*******************************************************************/
/* Title    : Conv_holes_draw                                      */
/* Funktion : Converts the holes so that they can be drawn on the  */
/*            screen.                                              */
/*                                                                 */
/* Dependencies :  --                                              */
/* In       : ref    : Arrray holding the track points             */
/*            aspect : The cameras Aspect Ratio                    */
/*            c, s   : Cos and Sin to the labyrinths slope         */
/* Out      :  --                                                  */
/* Remark   :  --                                                  */
/* Ref.     :  --                                                  */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 18/11 91                                             */
/*******************************************************************/
Conv_holes_draw(short int (*ref)[2], double *aspect, double *c, double *s)
{
   double x0,y0;
   int i;

   i = 0;
   while(ref[i][0] != -1)
   {
      x0 = (*c)*ref[i][0] + (*s)*ref[i][1]*(*aspect);
      y0 = -(*s)*ref[i][0]/(*aspect) + (*c)*ref[i][1];
      ref[i][0] = x0;
      ref[i][1] = y0;
      i++;
   }
}



/*******************************************************************/
/* Title    :  Find_hole                                           */
/*                                                                 */
/* CPU card :  Scan Beam                                           */
/*                                                                 */
/* Funktion :  Mainprogram for finding the holes of the labyrinth  */
/*             by use of invariant moments.                        */
/*                                                                 */
/*                                                                 */
/* Dependencies :  Find_area   Ajust_holes                         */
/*                                                                 */
/* In       : hole  : array of found holes to be returned          */
/*            max_i : number of holes found to be returned         */
/*                                                                 */
/* Out      : -                                                    */
/*                                                                 */
/* Remark   : -                                                    */
/*                                                                 */
/* Ref.     : -                                                    */
/*                                                                 */
/* Made by  : Soeren Juhl Jensen                                   */
/*                                                                 */
/* Date     : 5/10 91                                              */
/*******************************************************************/
Find_hole(short int (*hole)[2], int *max_i)
{
	Find_area(hole, max_i);
	Ajust_holes(hole, max_i);
}


/*******************************************************************/
/* Title    :  Find_area                                           */
/* Funktion :  Generates the area in which to calculate the        */
/*             invariant moments.                                  */
/*                                                                 */
/* Dependencies :  Moment                                          */
/* In       : hole  : array of found holes to be returned          */
/*            max_i : number of holes found to be returned         */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 5/10 91                                              */
/*******************************************************************/
Find_area(short int (*hole)[2], int *max_i)
{
	RECT area;          /*  area variable                    */
	int i,j,c,r,sum;
	UBYTE *pt, *point;  /*  pointer variables                */

	pt = (UBYTE *)(Fgetmp(PLANE0) + MOM_EDGE + MOM_EDGE*ROW_FACTOR);

	for(r=MOM_EDGE;r<COLS-MOM_EDGE;r+=2)
	{
		for(c=MOM_EDGE;c<ROWS-MOM_EDGE;c+=2)
		{
			sum = 0;
			if(*pt == MIN_GREY)
			{
				for(i=-MOM_EDGE;i<=MOM_EDGE;i++)
					for(j=-MOM_EDGE;j<=MOM_EDGE;j++)
						if(*(pt + i + j*ROW_FACTOR)==MIN_GREY) sum++;
				if(sum == (2*MOM_EDGE + 1)*(2*MOM_EDGE + 1))
				{
					if(c<MOM_AREA) area.p1.x = 0;
					else area.p1.x = c-MOM_AREA;
					if(c>COLS-MOM_AREA+1) area.p2.x = COLS-1;
					else area.p2.x = c+MOM_AREA;
					if(r<MOM_AREA) area.p1.y = 0;
					else area.p1.y = r-MOM_AREA;
					if(r>ROWS-MOM_AREA+1) area.p2.y = ROWS-1;
					else area.p2.y = r+MOM_AREA;

					Moment(&area, hole, max_i);
				}
			}
			pt +=2;

		}
		pt += ROW_FACTOR + 2*MOM_EDGE + COLS;
	}
}


/*******************************************************************/
/* Title    :  Moment                                              */
/* Funktion :  Calculates the 1'st and 2'nd invariant moments      */
/*             given by Hu (see reference)                         */
/*                                                                 */
/* Dependencies : -                                                */
/* In       : area  : the area in which to calculate moments       */
/*            hole  : array of found holes to be returned          */
/*            max_i : number of holes found to be returned         */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : Hu, Ming-Kuei :                                      */
/*            Visual Pattern Recognition by Moment Invariants, 1962*/
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 510 91                                               */
/*******************************************************************/
Moment(RECT *area, short int (*hole)[2], int *max_i)
{
	int   m00, m11, m20, m02, m10, m01;  /* moment variables */
	double midtx, midty;   /*  mass center variables         */
	double f1, f2;         /*  invariant moment variables    */
	double dum1, dum2, dum3;
	int   c, r, i, again;
	UBYTE *pt;             /*  pointer variable              */

	pt = (UBYTE *)(Fgetmp(PLANE0) + area->p1.x + ROW_FACTOR*(area->p1.y));
	m00 = m11 = m20 = m02 = m10 = m01 = 0;
	dum1 = dum2 = dum3 = f1 = f2 = 0;

	for(r=area->p1.y;r<=area->p2.y;r++)
	{
		for(c=area->p1.x;c<=area->p2.x;c++)
		{
			if(*pt == MIN_GREY)
			{
				m00++;
				m10 += c;
				m01 += r;
				m11 += r*c;
				m20 += c*c;
				m02 += r*r;
			}
			pt++;
		}
		pt += (ROW_FACTOR - 1 - area->p2.x + area->p1.x);
	}
	if(m00 != 0)
	{
		dum1 = m20 - (m10/(double)m00)*m10;
		dum2 = m02 - (m01/(double)m00)*m01;
		dum3 = m11 - (m01/(double)m00)*m10;

		f1 = (dum1/(double)m00) + (dum2/(double)m00);
		f2 = ((dum1 - dum2)/(double)(m00*m00))*(dum1 - dum2)
			 + (4*dum3/(double)(m00*m00))*dum3;
		midtx = m10/(double)m00;
		midty = m01/(double)m00;
	}
	if((f1>F1_MIN) && (f1<F1_MAX) && (f2>F2_MIN) && (f2<F2_MAX))
	{
		again = FALSE;
		for(i=0;i<*max_i;i++)
		{
			if((abs((int)midtx - hole[i][0]) < EPS_HOLE) &&
				(abs((int)midty - hole[i][1]) < EPS_HOLE))
				again = TRUE;
		}
		if(!again)
		{
			hole[*max_i][0] = (int)midtx;
			hole[*max_i][1] = (int)midty;
			(*max_i)++;
		}
	}
}


/*******************************************************************/
/* Title    :  Threshold                                           */
/* Funktion :  Makes a threshold of the image placed in Plane 0    */
/*             of the Scan Beam video memory. The thresholdvalue   */
/*             is found as the local minimum pixel value plus an   */
/*             offset.                                             */
/*                                                                 */
/* Dependencies : -                                                */
/* In       : -                                                    */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 5/10 91                                              */
/*******************************************************************/
Threshold(void)
{
	int c,r,i,j;             /*  collum and row variables        */
	int min, max;             /*  minimum pixel value             */
	UBYTE *pt0, *pt;     /*  pointer variable                */
	int thres;           /*  threshold variable              */
	int units = (COLS-2*D)/THRES_TIME + 1;

	pt0 = (UBYTE *)Fgetmp(PLANE0);

	for(i=0;i<THRES_TIME;i++)
	{
		for(j=0;j<THRES_TIME;j++)
		{
			min = MAX_GREY;
			max = MIN_GREY;
			for(c=0;c<units;c++)
			{
				for(r=0;r<units;r++)
				{
					pt = (UBYTE *)(pt0 + c+D+(i*units)
						 + (r+D+(units*j))*ROW_FACTOR);
					if(*pt<min) min=*pt;
					if(*pt>max) max=*pt;
				}
			}
			if((max - min) < THRES_DIF)
				thres = MIN_GREY;
			else
				thres = min + THRES_ADD;

			for(c=0;c<units;c++)
			{
				for(r=0;r<units;r++)
				{
					pt = (UBYTE *)(pt0 + c+D+(i*units)
						 + (r+D+(units*j))*ROW_FACTOR);
					if(*pt >= thres) *pt = MAX_GREY;
					else *pt = MIN_GREY;
				}
			}
		}
	}
}


/*******************************************************************/
/* Title    :  Ajust_holes                                         */
/* Funktion :  Ajust the holes in the found holes array            */
/*                                                                 */
/* Dependencies : -                                                */
/* In       :  hole  : array of found holes to be returned         */
/*             maxi  : number of holes found                       */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     :                                                      */
/*******************************************************************/
Ajust_holes(short int (*hole)[2], int *maxi)
{
	int   m00, m11, m10, m01;  /* moment variables */
	int   c, r, i;
	UBYTE *pt;             /*  pointer variable              */
	RECT area;

	Cplotmp(PLANE4);
	Ccolour(RED);
	for(i=0;i<*maxi;i++)
	{
		c = hole[i][0];
		r = hole[i][1];
		if(c<MOM_AREA) area.p1.x = 0;
		else area.p1.x = c-MOM_AREA;
		if(c>COLS-MOM_AREA+1) area.p2.x = COLS-1;
		else area.p2.x = c+MOM_AREA;
		if(r<MOM_AREA) area.p1.y = 0;
		else area.p1.y = r-MOM_AREA;
		if(r>ROWS-MOM_AREA+1) area.p2.y = ROWS-1;
		else area.p2.y = r+MOM_AREA;

		pt = (UBYTE *)(Fgetmp(PLANE0) + area.p1.x + ROW_FACTOR*(area.p1.y));
		m00 = m11 = m10 = m01 = 0;
		for(r=area.p1.y;r<=area.p2.y;r++)
		{
			for(c=area.p1.x;c<=area.p2.x;c++)
			{
				if(*pt == MIN_GREY)
				{
					m00++;
					m10 += c;
					m01 += r;
					m11 += r*c;
				}
				pt++;
			}
			pt += (ROW_FACTOR - 1 - area.p2.x + area.p1.x);
		}
		if(m00 != 0)
		{
			hole[i][0] = (short int)(m10/(double)m00);
			hole[i][1] = (short int)(m01/(double)m00);
			Ccircle(hole[i][0],hole[i][1],6);
		}
	}

}
/*------------------------------------------------------------------*/


