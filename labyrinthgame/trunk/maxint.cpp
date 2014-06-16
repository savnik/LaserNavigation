/********************************************************************/
/* Navn       : MaxInt                                              */
/* Funktion   : Finder koordinater og intensitet af det lyseste     */
/*              punkt i et givet interesseomraade.                  */
/* Kald       : -                                                   */
/* Ind        : *ramme - pointer til rect med koordinater for       */
/*                       teresseomraadet.                           */
/*              *xmax  - pointer til variabel som skal indeholde    */
/*                       x-koordinat for lyseste punkt.             */
/*              *ymax  - pointer til variabel som skal indeholde    */
/*                       y-koordinat for lyseste punkt.             */
/* Ud         : intensitet af lyseste punkt                         */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/*              ( omskrevet fra T. G. Jensen og J. Carlsens         */
/*                'whspot' )                                        */
/* Dato       : 11/3 1990                                           */
/********************************************************************/
#include "lab.h"

MaxInt(RECT *area, double *xmax, double *ymax, short int (*edge)[2])
{
    /* temp:
           indeholder pixelvaerdien for den pixel som pt peger paa. */
    /* l,k: loekke variable.                                        */
    /* pt: pointer til pixel i billed.                              */
    int temp,l,k,max,a_vram;
    UBYTE *pt;
    
    a_vram = (int)Fgetaddr(0);
    max=0;
    /* udpeger den foerste pixel i interesseomraadet.               */
    pt = (UBYTE *)
         (area->p1.y * ROW_FACTOR + area->p1.x + a_vram);
    for(l = area->p1.y; l <= area->p2.y; l++ )   /* linie loekke. */
    {
		if(edge[l][0] == edge[l][1])
			pt += ROW_FACTOR;
		else
		{
	        for(k = area->p1.x; k <= area->p2.x; k++)
        	                                     /* kolonne loekke. */
	        {
        		if( (k > (int)edge[l][0]) && (k < (int)edge[l][1]) )
        		{
	            	temp = *pt;                     /* aflaes pixel vaerdi. */
        	    	if (temp>max)
            		{
	              		max = temp;       /* ny max-vaerdi                    */
        	      		*xmax = k;
              			*ymax = l;
	            	}
        	    }
	            pt++; /* udpeg naeste pixel i linien.                   */
        	}
	        pt += (ROW_FACTOR - 1 - area->p2.x + area->p1.x );
                                         /* udpeg naeste linie. */
		}
    }
    return max;
}
