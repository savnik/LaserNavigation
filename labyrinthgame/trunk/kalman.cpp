
#include <stdio.h>
#include "lab.h"
#include "kalman.h"


// kalman parameters for each direction
kalman_parm kalmanXparam;
kalman_parm kalmanYparam;
kalman_parm * gkalx = &kalmanXparam;
kalman_parm * gkaly = &kalmanYparam;









/********************************************************************/
/* Navn       : LoadKalman                                          */
/* Funktion   : Indlaeser kalman parametre fra disk og placerer den    */
/*              et forudbestemt sted i ScanBeam-kortets hukommelse. */
/* Kald       : -                                                   */
/* Ind        : cmd - hvis DEFAULT indlaeses DEMO.KAL, hvis         */
/*                    PROMT promtes der paa terminalen for et fil-  */
/*                    navn                                          */
/*              sprog - det anvendte sprog                          */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/*              Modifiseret af Soeren Juhl Jensen  22/11 91         */
/* Dato       : 11/7 1990                                           */
/********************************************************************/ 
bool LoadKalman()
{
  FILE *fp;
  int  i;
  const char * kalmanParams = "kalman.ini";
  fp = fopen(kalmanParams, "r");
  if (fp==NULL)
  {
    printf("Failed to load Kalman parameters from %s\n", kalmanParams);
    return false;
  }
  i=0;
  {
    int nkal,nokal,nikal,j;
    fscanf(fp,"%d %d %d",&nkal,&nikal,&nokal);
    // file say nkal=5 nikal=1 nokal=1
    gkalx->n=nkal;
    gkaly->n=nkal;
    gkalx->ni=nikal;
    gkaly->ni=nikal;
    gkalx->no=nokal;
    gkaly->no=nokal;
    // 5x5 A matrix
    for (i=0;i<nkal;i++){
      for (j=0;j<nkal;j++){
        fscanf(fp,"%lf",&gkalx->a[i][j]);
        gkaly->a[i][j]=gkalx->a[i][j];
      }
    }
    // 5x1 B matrix
    for (i=0;i<nkal;i++){
      for (j=0;j<nikal;j++){
        fscanf(fp,"%lf ",&gkalx->b[i][j]);
        gkaly->b[i][j]=gkalx->b[i][j];
      }
    }
    // 5x1 C matrix
    for (i=0;i<nokal;i++){
      for (j=0;j<nkal;j++){
        fscanf(fp,"%lf",&gkalx->c[i][j]);
        gkaly->c[i][j]=gkalx->c[i][j];
      }
    }
    // 1x1 D matrix - men KalmanOutput(...) antager D er 1x3 ??? /chr
    for (i=0;i<nikal;i++){
      fscanf(fp,"%lf ",&gkalx->d[i]);
      gkaly->d[i]=gkalx->d[i];
    }
    // 1x5 vector
    for (i=0;i<nkal;i++){
      fscanf(fp,"%lf",&gkalx->initvect[i]);
      gkaly->initvect[i]=gkalx->initvect[i];
    }
    // 1x5 K vector
    for (i=0;i<nkal;i++){
      fscanf(fp,"%lf",&gkalx->K[i]);
      gkaly->K[i]=gkalx->K[i];
    }
    // total 3 + 25 + 5 + 5 + 1 + 5 + 5 = 49 (men der er 50 i filen?? / chr)
  }
  fclose(fp);
  return true;
}



/********************************************************************/
/* Navn       : InitKalman                                          */
/* Funktion   : Initialiserer variablene i en tilstandsregulator    */
/* Kald       : -                                                   */
/* Ind        : -                                                   */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/* Dato       : 9/7 1990                                            */
/********************************************************************/
void InitKalman(kalman_parm *parm, double x0)
{
  int i,j;
/*
#include "kalx40.prm"
*/

  for(i = 0; i < parm->n; i++)
  {
    parm->x[i] = parm->initvect[i] * x0;
  }
  for(i = 0; i < parm->no; i++)
  {
    parm->dum[i]=0;
    for(j = 0; j < parm->n; j++)
      parm->dum[i]+=parm->c[i][j]*parm->x[j];
  }
}


/********************************************************************/
/* Navn       : KalmanOutput                                        */
/* Funktion   : Beregner styresignalet fra en tilstandsbaseret      */
/*              regulator                                           */
/* Kald       : -                                                   */
/* Ind        : *parm - pointer til struct med regulatorparametre   */
/*              y     - fejlsignal                                  */
/* Ud         : styresignalet fra regulatoren                       */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/* Dato       : 9/7 1990                                            */
/********************************************************************/
double KalmanOutput(kalman_parm *parm, double y,double r,double v)
{ // y = current position, r = reference position v=velocity ref
  return y * parm->d[0] + r*parm->d[1] + v*parm->d[2] + parm->dum[0];
}

/********************************************************************/
/* Navn       : KalmanStates                                        */
/* Funktion   : Beregner tilstandene i kalmanfiltret                */
/*              regulator                                           */
/* Kald       : -                                                   */
/* Ind        : *parm - pointer til struct med regulatorparametre   */
/*              y     - fejlsignal                                  */
/* Ud         : filtertilstandene                                   */
/* Reference  : -                                                   */
/* Skrevet af : Nils Andesen                                        */
/* Dato       : 15/7 1998                                            */
/********************************************************************/
// void KalmanStates(kalman_parm *parm, double y,double states[])
// {
//   int i;
//   for (i=0;i< parm->n;i++)
//     states[i]=parm->x[i]+parm->K[i]*y;
// }





/********************************************************************/
/* Navn       : OpdatKalman                                         */
/* Funktion   : Opdaterer variable i en tilstandsbaseret regulator  */
/* Kald       : -                                                   */
/* Ind        : *parm - pointer til struct med regulatorvariable    */
/*              y     - fejlsignalet                                */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/* Dato       : 11/7 1990                                           */
/********************************************************************/
void OpdatKalman(kalman_parm *parm, double y,double r,double v)
{
  double xtemp[6];
  int i,j;

  for(i=0; i<parm->n; i++)
  {
    xtemp[i]=0;
    for(j=0; j<parm->n; j++)
    {
      xtemp[i] += parm->a[i][j] * parm->x[j];
    }
  }

  parm->dum[0] = 0;
  for(i=0; i<parm->n; i++){
    parm->x[i]=  parm->b[i][0] * y + parm->b[i][1]*r+parm->b[i][2]*v+ xtemp[i];
    parm->dum[0] += parm->c[0][i] * parm->x[i];
  }
}
