/*******************************************************************/
/* Title    :  Detect_Frame                                        */
/*                                                                 */
/* CPU card :  Scan Beam                                           */
/*                                                                 */
/* Funktion :  Mainprogram for controlling the detection of the    */
/*             labyrinth frame, based upon the knowledge of its    */
/*             position relative to the holes of the labyrinth.    */
/*                                                                 */
/* Dependencies :  Find_far   Find_near    Find_ang_k   Alike      */
/*                 Frame      Corner       Find_match   Init_near  */
/*                 Threshold  Region_Grow  Find_hole               */
/*                                                                 */
/* In       :  -                                                   */
/*                                                                 */
/* Out      :  -                                                   */
/*                                                                 */
/* Remark   :  -                                                   */
/*                                                                 */
/* Ref.     :  -                                                   */
/*                                                                 */
/* Made by  :  Soeren Juhl Jensen                                  */
/*                                                                 */
/* Date     :  5/10 91                                             */
/*******************************************************************/
#include "lab.h"
#include <math.h>

/*-----------------------------------------------------------------*/


int Init_near (short int (*hole_ref)[2], int *maxhole, double (*near)[2]);
extern int VisInstruktion (char *msg1, char *msg2);
extern int JustNiv (void);
extern int Threshold (void);
extern int Region_Grow (void);
extern int Find_hole (short int (*hole)[2], int *max_i);
int Find_match (short int *p, short int *p1, short int *p3, short int *p4, short int *found, short int *found_ref, short int (*hole_ref)[2], short int (*hole)[2], int *maxhole, int *maxi, short int *bingo, double (*near)[2]);
int Frame (short int *p, short int *p1, short int *p3, short int *p4, short int *found, short int *found_ref, short int (*hole_ref)[2], short int (*hole)[2], int *maxhole, int *maxi, POLYGON *frame, double *ASPECT);
int Corner (short int *p, short int *p1, float *p2, double *k, double *angl);

#define TEST    1
Detect_Frame(POLYGON *frame, short int (*hole_ref)[2], int maxhole, double *ASPECT)
{
        short int hole[MAXHOLES][2];      /* Found holes array        */
        double    near1[MAXHOLES][2];       /* Angle and distance to    */
                                                      /*  the nearest holes        */
        short int pt[MAXHOLES][2];
        int maxi;         /* Maximum values i ref. -  */
                               /*  and found holes array   */
        short int p[2], p1[2], p2[2], p3[2], p4[2], p5[2];
                                                                 /* Point variables     */
        short int i, j, z, count, x, y, n;  /* Counter variables   */
        short int found_ref, found;      /* Var's for matching holes */
        double angl, k, angl_dum, k_dum;   /* Angle and distance var's */
        short int  bingo_ref, bingo, ny_ref;


     /* clear the used arrays         */

        for(i=0;i<MAXHOLES;i++)
        {
                hole[i][0] = hole[i][1] = 0;
                near1[i][0] = near1[i][1] = 0.0;
        }
     /*  Initialize the near array    */

        Init_near(hole_ref,&maxhole,near1);


        VisInstruktion("Finder rammen","Finding the frame");
        JustNiv();

     /*  Threshold the image          */
#if (TEST==1)
VisInstruktion("Threshold","");
while(!Fmousel());
while(Fmousel());    
#endif
        Threshold();

     /*  Remove "large" black areas from the image               */
#if (TEST==1)
VisInstruktion("Region Grow","");
while(!Fmousel());
while(Fmousel());    
#endif
        Region_Grow();

     /*  Find the holes in the labyrinth                         */
#if (TEST==1)
VisInstruktion("Find Huller","");
while(!Fmousel());
while(Fmousel());    
#endif
        maxi = 0;
        Find_hole(hole,&maxi);

     /*  Ajust the hole coordinates for the camera's Aspect ratio */

        for(i=0;i<frame->polysize;i++)
        {
                frame->polypoint[i].y = frame->polypoint[i].y * (*ASPECT);
        }
        for(i=0;i<maxhole;i++)
        {
                hole_ref[i][1] = (short int)((float)hole_ref[i][1] * (*ASPECT));
        }
        for(i=0;i<maxi;i++)
        {
                hole[i][1] = (short int)((float)hole[i][1] * (*ASPECT));
        }
#if (TEST==1)
VisInstruktion("Find Match","");
while(!Fmousel());
while(Fmousel());    
#endif
     /*  Find a matching between the reference- and the found holes */
     Find_match(p,p1,p3,p4,&found,&found_ref,hole_ref,hole
                ,&maxhole,&maxi,&bingo,near1);

     /*  Search for the frame og the labyrinth               */

        if(bingo)
        {
#if (TEST==1)
VisInstruktion("Find Ramme","");
while(!Fmousel());
while(Fmousel());    
#endif              
                Frame(p,p1,p3,p4,&found,&found_ref,hole_ref,hole,&maxhole,&maxi,frame,ASPECT);
                return TRUE;
        }
        else
                return FALSE;

}

/*******************************************************************/
/* Title    :  Find_far                                            */
/* Funktion :  Find the n points farest from a given point,        */
/*             based upon a given array of points.                 */
/*                                                                 */
/* Dependencies : -                                                */
/* In       :  p      : a given point                              */
/*             pt     : array holding sorted elem. to be returned  */
/*             array  : array to be searched                       */
/*             no     : the number of the given point in array     */
/*             maxt   : number of points in the array              */
/*             n      : number of elements to sort                 */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 5/10 91                                              */
/*******************************************************************/
Find_far(short int *p, short int (*pt)[2], short int (*array)[2], short int *no, int *maxt, short int *n)
{
        short int  d1, d2, i, tal, tp[2];
        double  max[MAXHOLES], temp;        /* Distance variable          */

        for(i=0;i<=*n;i++) max[i] = 0;
        for(tal=0;tal<*maxt;tal++)
        {

                if(tal != *no)
                {
                        d1 = p[0] - array[tal][0];
                        d2 = p[1] - array[tal][1];
                        max[0] = sqrt((double)(d1*d1 + d2*d2));
                        pt[0][0] = array[tal][0];
                        pt[0][1] = array[tal][1];
                        for(i=1;i<=*n;i++)
                        {
                                if(max[i-1] > max[i])
                                {
                                        temp = max[i];
                                        max[i] = max[i-1];
                                        max[i-1] = temp;
                                        tp[0] = pt[i][0];
                                        tp[1] = pt[i][1];
                                        pt[i][0] = pt[i-1][0];
                                        pt[i][1] = pt[i-1][1];
                                        pt[i-1][0] = tp[0];
                                        pt[i-1][1] = tp[1];
                                }
                        }
                }
        }
}



/*******************************************************************/
/* Title    :  Find_near                                           */
/* Funktion :  Find the n points nearest to a given point,         */
/*             based upon a given array of points.                 */
/*                                                                 */
/* Dependencies : -                                                */
/* In       :  p      : a given point                              */
/*             pt     : array holding sorted elem. to be returned  */
/*             array  : array to be searched                       */
/*             no     : the number of the given point in array     */
/*             maxt   : number of points in the array              */
/*             n      : number of elements to sort                 */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 5/10 91                                              */
/*******************************************************************/
Find_near(short int *p, short int (*pt)[2], short int (*array)[2], short int *no, int *maxt, short int *n)
{
        short int  d1, d2, i, tal, tp[2];
        double  min[MAXHOLES], temp;        /* Distance variable          */
        for(i=0;i<=*n;i++)  min[i] = 2*COLS;
        for(tal=0;tal<*maxt;tal++)
        {
                if(tal != *no)
                {
                        d1 = p[0] - array[tal][0];
                        d2 = p[1] - array[tal][1];
                        min[0] = sqrt((double)(d1*d1 + d2*d2));
                        pt[0][0] = array[tal][0];
                        pt[0][1] = array[tal][1];
                        for(i=1;i<=*n;i++)
                        {
                                if(min[i-1] < min[i])
                                {
                                        temp = min[i];
                                        min[i] = min[i-1];
                                        min[i-1] = temp;
                                        tp[0] = pt[i][0];
                                        tp[1] = pt[i][1];
                                        pt[i][0] = pt[i-1][0];
                                        pt[i][1] = pt[i-1][1];
                                        pt[i-1][0] = tp[0];
                                        pt[i-1][1] = tp[1];
                                }
                        }
                }
        }
}


/*******************************************************************/
/* Title    : Find_ang_k                                           */
/* Funktion : Find the angle between, and the relationship between */
/*            distances to two points, with a given point as offset*/
/*                                                                 */
/* Dependencies : -                                                */
/* In       : p      : a given point                               */
/*            p1, p2 : the two other points                        */
/*            k      : distance relationship to be returned        */
/*            angl   : the angle to be returned                    */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 5/10 91                                              */
/*******************************************************************/
Find_ang_k(short int *p, short int *p1, short int *p2, double *k, double *angl)
{
        double  x1, x2, y1, y2;
        double  dum1, dum2, dum3, dum4, dum5;

        x1 = (double)(p1[0]) - (double)(p[0]);
        x2 = (double)(p2[0]) - (double)(p[0]);
        y1 = (double)(p1[1]) - (double)(p[1]);
        y2 = (double)(p2[1]) - (double)(p[1]);

        dum1 = sqrt(x1*x1 + y1*y1);
        dum2 = sqrt(x2*x2 + y2*y2);
        dum3 = x1*x2 + y1*y2;
        dum4 = x1*y2 - x2*y1;
        dum5 = acos(dum3/(dum1*dum2));
        *k = dum1/dum2;
        if(dum4 >= 0.0) *angl = dum5;
        else *angl = -dum5;
}


/*******************************************************************/
/* Title    :  Alike                                               */
/* Funktion :  Find the number of points in two arrays that are    */
/*             alike within a given tolerance                      */
/*                                                                 */
/* Dependencies : -                                                */
/* In       : ref_ang, ang  : the two arrays to be compared        */
/*            count         : the number of matches to be returned */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 5/10 91                                              */
/*******************************************************************/
Alike(double (*ref_ang)[2], double (*ang)[2], short int *count)
{
        short int  i, j, max;
        double temp[2];
        int match;

        *count = 0;
        max = N_near;

        for(i=0;i<N_near;i++)
        {
                if((ang[i][0]>0.0) || (ang[i][1]>0.0))
                {
                        j = i;
                        match = FALSE;
                        while((j<=max) && !match)
                        {
                                if((fabs(ref_ang[i][0]-ang[j][0])<EPS_K) &&
                                        (fabs(ref_ang[i][1]-ang[j][1])<EPS_ANG))
                                {
                                        (*count)++;
                                        match = TRUE;
                                        if(j>i)
                                        {
                                                temp[0] = ang[i][0];
                                                temp[1] = ang[i][1];
                                                ang[i][0] = ang[j][0];
                                                ang[i][1] = ang[j][1];
                                                ang[j][0] = temp[0];
                                                ang[j][1] = temp[1];
                                        }
                                }
                                j++;
                        }
                        if(!match)
                        {
                                max++;
                                ang[max][0] = ang[i][0];
                                ang[max][1] = ang[i][1];
                                ang[i][0] = 0.00;
                                ang[i][1] = 0.00;
                        }
                }
        }
}




/*******************************************************************/
/* Title    :  Frame                                               */
/* Funktion :  Find the frame corners, given two matching holes in */
/*             the found - and the reference holes array.          */
/* Dependencies :  Find_near   Find_far   Find_ang_k   Corner      */
/* In       :  p, p3  : the matching holes (found & reference)     */
/*             p1,p4  : the farest holes from the matching holes   */
/*             found, found_ref : array no. of the matching holes  */
/*             hole, hole_ref   : the hole arrays                  */
/*             maxi, maxhole    : the number of holes              */
/*             frame            : the reference frame corners      */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  :  Soeren Juhl Jensen                                  */
/* Date     :                                                      */
/*******************************************************************/
Frame(short int *p, short int *p1, short int *p3, short int *p4, short int *found, short int *found_ref, short int (*hole_ref)[2], short int (*hole)[2], int *maxhole, int *maxi, POLYGON *frame, double *ASPECT)
{
        short int pt[MAXHOLES][2], pt_ref[MAXHOLES][2], pt_dum[MAXHOLES][2], pt_temp[2];
        float frame_out[MAXHOLES][4][2];
        float mean[4][2], disp[4][2], mean_out[4][2],meanpt[2],meanpt_ref[2];
        short int i, j, n, p2[2], p5[2], ptemp[2];
        int max, count, match, frame_no[4];
        double ang[MAXHOLES][2], ref_ang[MAXHOLES][2], temp[2], k, angl;

     /*  Find and sort the nearest reference holes     */

        n = *maxhole - 2;
        Find_near(p3, pt_dum, hole_ref, found_ref, maxhole, &n);
        for(i=1;i<*maxhole-1;i++)
        {
                pt_ref[i-1][0] = pt_dum[*maxhole-i-1][0];
                pt_ref[i-1][1] = pt_dum[*maxhole-i-1][1];
        }

     /*  Find and sort the nearest found holes         */

        n = *maxi - 2;
        Find_near(p, pt_dum, hole, found, maxi, &n);
        for(i=1;i<*maxi-1;i++)
        {
                pt[i-1][0] = pt_dum[*maxi-i-1][0];
                pt[i-1][1] = pt_dum[*maxi-i-1][1];
        }

        for(i=0;i<MAXHOLES;i++)
                ang[i][0] = ang[i][1] = ref_ang[i][0] = ref_ang[i][1] = 0;

     /*  Find angle and distance for the reference holes     */

        for(i=0;i<*maxhole-2;i++)
        {
                Find_ang_k(p3,pt_ref[i],p4,&k,&angl);
                ref_ang[i][0] = k;
                ref_ang[i][1] = angl;
        }

     /*  Find angle and distance for the found holes     */

        for(i=0;i<*maxi-2;i++)
        {
                Find_ang_k(p,pt[i],p1,&k,&angl);
                ang[i][0] = k;
                ang[i][1] = angl;
        }

     /*  Order the angle/distance and points for the found holes,  */
     /*    such that matching holes has similar places in the      */
     /*    reference- and found holes araay.                       */

        max = *maxi-2;
        count = 0;

        for(i=0;i<*maxhole-2;i++)
        {
                if((pt[i][0]>0) || (pt[i][1]>0))
                {
                        j = i;
                        match = FALSE;
                        while((j<=max) && !match)
                        {
                                if((fabs(ref_ang[i][0]-ang[j][0])<EPS_K) &&
                                        (fabs(ref_ang[i][1]-ang[j][1])<EPS_ANG))
                                {
                                        count++;
                                        match = TRUE;
                                        if(j>i)
                                        {
                                                temp[0] = ang[i][0];    temp[1] = ang[i][1];
                                                ang[i][0] = ang[j][0];  ang[i][1] = ang[j][1];
                                                ang[j][0] = temp[0];    ang[j][1] = temp[1];
                                                pt_temp[0] = pt[i][0];  pt_temp[1] = pt[i][1];
                                                pt[i][0] = pt[j][0];    pt[i][1] = pt[j][1];
                                                pt[j][0] = pt_temp[0];  pt[j][1] = pt_temp[1];
                                        }
                                }
                                j++;
                        }
                        if(!match)
                        {
                                ang[max][0] = ang[i][0];  ang[max][1] = ang[i][1];
                                pt[max][0] = pt[i][0];    pt[max][1] = pt[i][1];
                                pt[i][0] = pt[i][1] = 0;
                                max++;
                        }
                }
        }


     /*  Find the mass center of the matching holes in both the   */
     /*    reference- and the found holes array.                  */

        meanpt[0] = (float)p[0];        meanpt[1] = (float)p[1];
        meanpt_ref[0] = (float)p3[0];   meanpt_ref[1] = (float)p3[1];
        count = 1;
        for(i=0;i<*maxhole-2;i++)
        {
                if((pt[i][0] > 0) || (pt[i][1] > 0))
                {
                        meanpt_ref[0] += (float)pt_ref[i][0];
                        meanpt_ref[1] += (float)pt_ref[i][1];
                        meanpt[0] += (float)pt[i][0];
                        meanpt[1] += (float)pt[i][1];
                        count++;
                }
        }

        p2[0] = meanpt[0]/(float)count;      p2[1] = meanpt[1]/(float)count;
        p5[0] = meanpt_ref[0]/(float)count;  p5[1] = meanpt_ref[1]/(float)count;

     /*  Find a set of frame corners from each matching hole     */

        count = 0;
        for(j=0;j<frame->polysize;j++)
        {
                ptemp[0] = frame->polypoint[j].x;
                ptemp[1] = frame->polypoint[j].y;
                Find_ang_k(p3,ptemp,p5,&k,&angl);
                Corner(p,p2,frame_out[count][j],&k,&angl);
        }
        count++;
        for(i=0;i<*maxhole-2;i++)
        {
                if((pt[i][0] > 0) || (pt[i][1] > 0))
                {
                        for(j=0;j<frame->polysize;j++)
                        {
                                ptemp[0] = frame->polypoint[j].x;
                                ptemp[1] = frame->polypoint[j].y;
                                Find_ang_k(pt_ref[i],ptemp,p5,&k,&angl);
                                Corner(pt[i],p2,frame_out[count][j],&k,&angl);
                        }
                        count++;
                }
        }


        for(i=0;i<frame->polysize;i++)
        {
                mean[i][0] = mean[i][1] = 0;
                disp[i][0] = disp[i][1] = 0;
                mean_out[i][0] = mean_out[i][1] = 0;
        }

     /*  Find the mean value and dispersion of the four frame corners */

        for(i=0;i<count;i++)
        {
                for(j=0;j<frame->polysize;j++)
                {
                        mean[j][0] += (frame_out[i][j][0]/(float)count);
                        mean[j][1] += (frame_out[i][j][1]/(float)count);
                        disp[j][0] += ((frame_out[i][j][0]*frame_out[i][j][0])/(float)count);
                        disp[j][1] += ((frame_out[i][j][1]*frame_out[i][j][1])/(float)count);
                }
        }

        for(i=0;i<frame->polysize;i++)
        {
                disp[i][0] = sqrt(disp[i][0] - mean[i][0]*mean[i][0]);
                disp[i][1] = sqrt(disp[i][1] - mean[i][1]*mean[i][1]);
        }


        for(i=0;i<frame->polysize;i++) frame_no[i] = 0;

     /*  Find new mean values based upon points that lie within     */
     /*    the dispersion from the old mean values.                 */

        for(i=0;i<count;i++)
        {
                for(j=0;j<frame->polysize;j++)
                {
                        if((fabs(frame_out[i][j][0] - mean[j][0]) < disp[j][0])
                                && (fabs(frame_out[i][j][1] - mean[j][1]) < disp[j][1]))
                        {
                                mean_out[j][0] += frame_out[i][j][0];
                                mean_out[j][1] += frame_out[i][j][1];
                                frame_no[j]++;
                        }
                }
        }

        for(i=0;i<frame->polysize;i++)
        {
                mean_out[i][0] /= (float)frame_no[i];
                mean_out[i][1] /= (float)frame_no[i];
        }

     /*  Ajust the found frame corners to the used camera's    */
     /*    Aspect ratio                                        */

        for(i=0;i<frame->polysize;i++)
        {
                frame->polypoint[i].x = mean_out[i][0];
                frame->polypoint[i].y = mean_out[i][1]/(*ASPECT);
        }
}



/*******************************************************************/
/* Title    :  Corner                                              */
/* Funktion :  Find a point which is given by the angle and a      */
/*             distance from two other points                      */
/*                                                                 */
/* Dependencies : -                                                */
/* In       : p, p1 : two points given                             */
/*            p2    : the point to be returned                     */
/*            k     : distance relationship between p-p1 and p-p2  */
/*            angl  : the angle between p1 and p2 taken from p     */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 5/10  91                                             */
/*******************************************************************/
Corner(short int *p, short int *p1, float *p2, double *k, double *angl)
{
        float x1, x2, y1, y2;

        x1 = (p1[0]) - (p[0]);
        y1 = (p1[1]) - (p[1]);

        x2 = x1*cos(*angl) + y1*sin(*angl);
        y2 = -x1*sin(*angl) + y1*cos(*angl);

        p2[0] = (*k)*x2 + (float)p[0];
        p2[1] = (*k)*y2 + (float)p[1];
}


/*******************************************************************/
/* Title    : Find_match                                           */
/* Funktion : Find a matching hole between the reference- and the  */
/*            found holes array.                                   */
/* Dependencies : Alike   Find_far   Find_near   Find_ang_k        */
/* In       :  p, p3  : the matching holes (found & reference)     */
/*             p1,p4  : the farest holes from the matching holes   */
/*             found, found_ref : array no. of the matching holes  */
/*             hole, hole_ref   : the hole arrays                  */
/*             maxi, maxhole    : the number of holes              */
/*             bingo  : match found flag                           */
/*             near   : array of angle/distance to nearest points  */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  :  Soeren Juhl Jensen                                  */
/* Date     :                                                      */
/*******************************************************************/
Find_match(short int *p, short int *p1, short int *p3, short int *p4, short int *found, short int *found_ref, short int (*hole_ref)[2], short int (*hole)[2], int *maxhole, int *maxi, short int *bingo, double (*near1)[2])
{
        short int i, j, n, z;
        short int pt[MAXHOLES][2];
        short int ny_ref, bingo_ref;
        double k, angl, k_dum, angl_dum;
        double ang[MAXHOLES][2], ref_ang[MAXHOLES][2];
        short int count;

        i = 0;
        *bingo = FALSE;
        while((!(*bingo)) && (i < (short int)*maxi))
        {
                p[0] = hole[i][0];
                p[1] = hole[i][1];
                n = 1;
                Find_far(p, pt, hole, &i, maxi, &n);
                p1[0] = pt[n][0];
                p1[1] = pt[n][1];
                n = 1;
                Find_near(p, pt, hole, &i, maxi, &n);
                Find_ang_k(p, pt[n], p1, &k, &angl);
                j = 0;
                ny_ref = FALSE;
                while((!ny_ref) && (j<(short int)*maxhole))
                {
                        bingo_ref = FALSE;
                        if((fabs(k - near1[j][0]) < EPS_K) && (fabs(angl - near1[j][1]) < EPS_ANG))
                        {       
                                *found_ref = j;
                                bingo_ref = TRUE;
                        }

                        if(bingo_ref)
                        {
                                n = N_near;
                                Find_near(p, pt, hole, &i, maxi, &n);
                                for(z=1;z<=N_near;z++)
                                {
                                        Find_ang_k(p, pt[z], p1, &k_dum, &angl_dum);
                                        ang[z-1][0] = k_dum;
                                        ang[z-1][1] = angl_dum;
                                }


                                p3[0] = hole_ref[*found_ref][0]; /*  actual reference hole  */
                                p3[1] = hole_ref[*found_ref][1];
                                n = 1;
                                Find_far(p3, pt, hole_ref, found_ref, maxhole, &n);
                                p4[0] = pt[n][0];
                                p4[1] = pt[n][1];
                                n = N_near;
                                Find_near(p3, pt, hole_ref, found_ref, maxhole, &n);
                                for(z=1;z<=N_near;z++)
                                {
                                        Find_ang_k(p3, pt[z], p4, &k_dum, &angl_dum);
                                        ref_ang[z-1][0] = k_dum;
                                        ref_ang[z-1][1] = angl_dum;
                                }
                                Alike(ref_ang, ang, &count);

                                if(count >= N_near-1)
                                {
                                        *found = i;
                                        ny_ref = TRUE;
                                        *bingo = TRUE;
                                }
                        }
                        j++;
                }
                i++;
        }
}


/*******************************************************************/
/* Title    : Init_near                                            */
/* Funktion : Find the angle/distance between farest and nearest   */
/*            hole, for all the holes in the reference hole array. */
/* Dependencies : Find_near   Find_far   Find_ang_k                */
/* In       : hole_ref  : reference holes array                    */
/*            maxhole   : number of reference holes                */
/*            near      : array of angle/distance to nearest holes */
/* Out      : -                                                    */
/* Remark   : -                                                    */
/* Ref.     : -                                                    */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     :                                                      */
/*******************************************************************/
Init_near(short int (*hole_ref)[2], int *maxhole, double (*near1)[2])
{
        short int p[2],p1[2],pt[MAXHOLES][2];
        short int n,i;
        double k, angl;

        for(i=0;i<(short int)*maxhole;i++)
        {
                p[0] = hole_ref[i][0];
                p[1] = hole_ref[i][1];
                n = 1;
                Find_far(p, pt, hole_ref, &i, maxhole, &n);
                p1[0] = pt[n][0];
                p1[1] = pt[n][1];
                n = 1;
                Find_near(p, pt, hole_ref, &i, maxhole, &n);
                Find_ang_k(p, pt[n], p1, &k, &angl);
                near1[i][0] = k;
                near1[i][1] = angl;
        }
}



/*----------------------------------------------------------------*/


