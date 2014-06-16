/*******************************************************************/
/* Title    :  Region_Grow                                         */
/*                                                                 */
/* Funktion :  Removes all black areas of an image, that is larger */
/*             than a given amount of pixels. The program uses the */
/*             principles of Region Growing.                       */
/*                                                                 */
/* Dependencies :   Find_start                                     */
/*                                                                 */
/* In       : -                                                    */
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
#include "lab.h"


/*-----------------------------------------------------------------*/



int Find_start (UBYTE *pt0);
int Reg_count (UBYTE *pt0, int *c, int *r, int *sum_flag);
int Reg_Grow (UBYTE *pt0, int *c, int *r);
int Find_pix (UBYTE *pt0, int (*pt)[2], int *ptemp, int *k, int *max, int *i);
int Find_count (UBYTE *pt1, int (*pt)[2], int *ptemp, int *k, int *max, int *i, int *sum);

Region_Grow(void)
{
	int i, c, r;
	UBYTE *pt0, *pt;

	pt0 = (UBYTE *)Fgetmp(PLANE0);
	Cplotmp(PLANE0);
	Ccolour(MAX_GREY);

	for (i=0;i<30;i++)
		Cbox(i,COLS-1-i,i,ROWS-1-i);

     /*  Copy PLANE 0 to PLANE 1             */

	for(c=0;c<COLS;c++)
		for(r=0;r<ROWS;r++)
		{
			pt = pt0 + c + r*ROW_FACTOR;
			*(pt+COLS) = *pt;
		}
	Find_start(pt0);
}


/*******************************************************************/
/* Title    :  Find_start                                          */
/* Funktion :  Find areas of the image that is larger than         */
/*             MAX_PIX pixels (larger than an hole)                */
/*                                                                 */
/* Dependencies :  Reg_Grow   Reg_count                            */
/* In       : pt0  :  pointer to PLANE 0                           */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 5/10 91                                              */
/*******************************************************************/
Find_start(UBYTE *pt0)
{
	UBYTE *pt;  /*  video pointers                    */
	int c,r;           /*  collum and row variables          */
	int sum_flag;           /*  black pixel counter flag       */
	register int i,j;  /*  counter variables                 */

	for(c=3;c<COLS-3;c+=2)
		for(r=3;r<ROWS-3;r+=2)
		{
			sum_flag = FALSE;
			pt = pt0 + c + r*ROW_FACTOR;
			if(*(pt+COLS) == 0)
			{
				Reg_count(pt0,&c,&r,&sum_flag);
				if(sum_flag)
					Reg_Grow(pt0,&c,&r);
			}
		}
}



/*******************************************************************/
/* Title    :  Reg_grow                                            */
/* Funktion :  Removes all black pixels connected, starting from   */
/*             a given point.                                      */
/*                                                                 */
/* Dependencies :  Find_pix                                        */
/* In       : pt0  : pointer to PLANE 0                            */
/*            c, r : the collum and row of where to start          */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl jensen                                   */
/* Date     : 5/10 91                                              */
/*******************************************************************/
Reg_Grow(UBYTE *pt0, int *c, int *r)
{
	int  k;
	int  stp, stop=FALSE;  /*  stop flags                    */
	int  i, counter;    /*  counter variables                */
	int  max=1;
	int pt[MAX_POINT][2], ptemp[2], p[2];

	pt[0][0] = *c;
	pt[0][1] = *r;

	while((!stop) && (max < MAX_POINT/2) )
	{
		stp = TRUE;
		counter = max;
		for(i=0;i<=counter;i++)
		{
			k = 0;

			/*   pixel above actuel pixel            */
				ptemp[0] = pt[i][0];
				ptemp[1] = pt[i][1] - 1;
			if(i<max && *(pt0 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_pix(pt0,pt,ptemp,&k,&max,&i);
		/*   pixel left of actuel pixel            */
			ptemp[0] = pt[i][0] - 1;
			ptemp[1] = pt[i][1];
			if(i<max && *(pt0 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_pix(pt0,pt,ptemp,&k,&max,&i);
		/*   pixel right of actuel pixel           */
			ptemp[0] = pt[i][0] + 1;
			ptemp[1] = pt[i][1];
			if(i<max && *(pt0 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_pix(pt0,pt,ptemp,&k,&max,&i);
		/*   pixel under actuel pixel              */
			ptemp[0] = pt[i][0];
			ptemp[1] = pt[i][1] + 1;
			if(i<max && *(pt0 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_pix(pt0,pt,ptemp,&k,&max,&i);
		/*   pixel left over actual pixel          */
			ptemp[0] = pt[i][0] - 1;
			ptemp[1] = pt[i][1] - 1;
			if(i<max && *(pt0 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_pix(pt0,pt,ptemp,&k,&max,&i);
		/*   pixel right over actual pixel         */
			ptemp[0] = pt[i][0] + 1;
			ptemp[1] = pt[i][1] - 1;
			if(i<max && *(pt0 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_pix(pt0,pt,ptemp,&k,&max,&i);
		/*   pixel left under actual pixel         */
			ptemp[0] = pt[i][0] - 1;
			ptemp[1] = pt[i][1] + 1;
			if(i<max && *(pt0 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_pix(pt0,pt,ptemp,&k,&max,&i);
		/*   pixel right under actual pixel        */
			ptemp[0] = pt[i][0] + 1;
			ptemp[1] = pt[i][1] + 1;
			if(i<max && *(pt0 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_pix(pt0,pt,ptemp,&k,&max,&i);

			pt[i][0] = pt[max-1][0];
			pt[i][1] = pt[max-1][1];
			max--;
			stp = (stp && (k == 0));

		}
		stop = stp;
	}
}



/*******************************************************************/
/* Title    :  Find_pix                                            */
/* Funktion :  Controls the managing of the point array            */
/*                                                                 */
/* Dependencies : -                                                */
/* In       : pt0    : pointer to PLANE 0                          */
/*            pt     : array of points                             */
/*            ptemp  : a place in the image                        */
/*            k, i   : counter variables                           */
/*            max    : number of points in the array               */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 5/10 91                                              */
/*******************************************************************/
Find_pix(UBYTE *pt0, int (*pt)[2], int *ptemp, int *k, int *max, int *i)
{
	pt[*max][0] = ptemp[0];
	pt[*max][1] = ptemp[1];
	*(pt0 + pt[*max][0] + pt[*max][1]*ROW_FACTOR) = MAX_GREY;
	*(pt0 + (ROW_FACTOR-COLS) + pt[*max][0] + pt[*max][1]*ROW_FACTOR)
           = MAX_GREY;
	(*max)++;
	(*k)++;
}


/*******************************************************************/
/* Title    :  Reg_count                                           */
/* Funktion :  Counts the number of pixels in a region, and stops  */
/*             when MAX_PIX pixels is reached.                     */
/*                                                                 */
/* Dependencies :  Find_count                                      */
/* In       : pt0      : pointer to PLANE 0                        */
/*            c, r     : the collum and row of where to start      */
/*            sum_flag : flag indicating if MAX_PIX is reached     */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl jensen                                   */
/* Date     : 5/10 91                                              */
/*******************************************************************/
Reg_count(UBYTE *pt0, int *c, int *r, int *sum_flag)
{
	int  k;
	int  stp, stop=FALSE;  /*  stop flags                    */
	int  i, counter, sum;    /*  counter variables                */
	int  max=1;
	int pt[MAX_POINT][2], ptemp[2], p[2];
	UBYTE *pt1;

	pt1 = pt0 + (ROW_FACTOR-COLS);
	pt[0][0] = p[0] = *c;
	pt[0][1] = p[1] = *r;
	sum = 0;
	while((!stop) && (max < MAX_POINT/2) && !(*sum_flag))
	{
		stp = TRUE;
		counter = max;
		if(sum > MAX_PIX) *sum_flag = TRUE;
		for(i=0;i<=counter;i++)
		{
			k = 0;
		/*   pixel above actuel pixel            */
			ptemp[0] = pt[i][0];
			ptemp[1] = pt[i][1] - 1;
			if(i<max && *(pt1 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_count(pt1,pt,ptemp,&k,&max,&i,&sum);
		/*   pixel left of actuel pixel            */
			ptemp[0] = pt[i][0] - 1;
			ptemp[1] = pt[i][1];
			if(i<max && *(pt1 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_count(pt1,pt,ptemp,&k,&max,&i,&sum);
		/*   pixel right of actuel pixel           */
			ptemp[0] = pt[i][0] + 1;
			ptemp[1] = pt[i][1];
			if(i<max && *(pt1 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_count(pt1,pt,ptemp,&k,&max,&i,&sum);
		/*   pixel under actuel pixel              */
			ptemp[0] = pt[i][0];
			ptemp[1] = pt[i][1] + 1;
			if(i<max && *(pt1 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_count(pt1,pt,ptemp,&k,&max,&i,&sum);
		/*   pixel left over actual pixel          */
			ptemp[0] = pt[i][0] - 1;
			ptemp[1] = pt[i][1] - 1;
			if(i<max && *(pt1 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_count(pt1,pt,ptemp,&k,&max,&i,&sum);
		/*   pixel right over actual pixel         */
			ptemp[0] = pt[i][0] + 1;
			ptemp[1] = pt[i][1] - 1;
			if(i<max && *(pt1 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_count(pt1,pt,ptemp,&k,&max,&i,&sum);
		/*   pixel left under actual pixel         */
			ptemp[0] = pt[i][0] - 1;
			ptemp[1] = pt[i][1] + 1;
			if(i<max && *(pt1 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_count(pt1,pt,ptemp,&k,&max,&i,&sum);
		/*   pixel right under actual pixel        */
			ptemp[0] = pt[i][0] + 1;
			ptemp[1] = pt[i][1] + 1;
			if(i<max && *(pt1 + ptemp[0] + ptemp[1]*ROW_FACTOR) == MIN_GREY)
				Find_count(pt1,pt,ptemp,&k,&max,&i,&sum);

			pt[i][0] = pt[max-1][0];
			pt[i][1] = pt[max-1][1];
			max--;
			stp = (stp && (k == 0));

		}
		stop = stp;
	}
}



/*******************************************************************/
/* Title    :  Find_count                                          */
/* Funktion :  Controls managing of the point array in counting    */
/*             the pixels in Reg_count.                            */
/* Dependencies : -                                                */
/*            pt1    : pointer to PLANE 1                          */
/*            pt     : array of points                             */
/*            ptemp  : a place in the image                        */
/*            k, i   : counter variables                           */
/*            max    : number of points in the array               */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 5/10 91                                              */
/*******************************************************************/
Find_count(UBYTE *pt1, int (*pt)[2], int *ptemp, int *k, int *max, int *i, int *sum)
{
	pt[*max][0] = ptemp[0];
	pt[*max][1] = ptemp[1];
	*(pt1 + pt[*max][0] + pt[*max][1]*ROW_FACTOR) = MAX_GREY;
	(*max)++;
	(*k)++;
	(*sum)++;
}


/*-----------------------------------------------------------------*/
