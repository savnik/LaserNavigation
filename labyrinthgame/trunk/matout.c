#include "lab.h"
#include <stdio.h>

extern int Find_near (short int *p, short int (*pt)[2], short int (*array)[2], short int *no, int *maxt, short int *n);
extern int Find_ang_k (short int *p, short int *p1, short int *p2, double *k, double *angl);
extern int Find_far (short int *p, short int (*pt)[2], short int (*array)[2], short int *no, int *maxt, short int *n);

main(void)
{
	FILE *fp;
	int more,pi[2],maxhole;
	double k,angl;
	short int pt[MAXHOLES][2],hole_ref[MAXHOLES][2],p[2],p1[2],n,i;
	
	fp = fopen("/h0/user/soren/lab/DEMO.HOL","r");
	for(i=0;i<4;i++)
	{
		fscanf(fp,"%hd %hd",&(p[0]),&(p[1]));
	}
	i = 0;
	more = TRUE;
	while(more)
	{
		fscanf(fp,"%hd %hd",&(hole_ref[i][0]),&(hole_ref[i][1]));
		if(hole_ref[i][0] == -1) more = FALSE;
		i++;
	}
	maxhole = i-1;
	fclose(fp);
printf("indlaest\n");	

	fp = fopen("/h0/user/soren/lab/near12.out","w");
	for(i=0;i<maxhole;i++)
	{
		p[0] = hole_ref[i][0];
		p[1] = hole_ref[i][1];
		n = 2;
		Find_near(p,pt,hole_ref,&i,&maxhole,&n);
		Find_ang_k(p,pt[2],pt[1],&k,&angl);
		fprintf(fp,"%-6.4f  %-6.4f\n",k,angl);
		printf("%-6.4f  %-6.4f\n",k,angl);
	}
	fclose(fp);
printf("11\n");	

	fp = fopen("/h0/user/soren/lab/far12.out","w");
	for(i=0;i<maxhole;i++)
	{
		p[0] = hole_ref[i][0];
		p[1] = hole_ref[i][1];
		n = 2;
		Find_far(p,pt,hole_ref,&i,&maxhole,&n);
		Find_ang_k(p,pt[1],pt[2],&k,&angl);
		fprintf(fp,"%-6.4f  %-6.4f\n",k,angl);
	}
	fclose(fp);
printf("12\n");	

	fp = fopen("/h0/user/soren/lab/nearfar11.out","w");
	for(i=0;i<maxhole;i++)
	{
		p[0] = hole_ref[i][0];
		p[1] = hole_ref[i][1];
		n = 1;
		Find_far(p,pt,hole_ref,&i,&maxhole,&n);
		p1[0] = pt[1][0]; p1[1] = pt[1][1];
		n = 1;
		Find_near(p,pt,hole_ref,&i,&maxhole,&n);
		Find_ang_k(p,pt[1],p1,&k,&angl);
		fprintf(fp,"%-6.4f  %-6.4f\n",k,angl);
	}
	fclose(fp);
printf("11\n");	

	fp = fopen("/h0/user/soren/lab/nearfar12.out","w");
	for(i=0;i<maxhole;i++)
	{
		p[0] = hole_ref[i][0];
		p[1] = hole_ref[i][1];
		n = 2;
		Find_far(p,pt,hole_ref,&i,&maxhole,&n);
		p1[0] = pt[1][0]; p1[1] = pt[1][1];
		n = 1;
		Find_near(p,pt,hole_ref,&i,&maxhole,&n);
		Find_ang_k(p,pt[1],p1,&k,&angl);
		fprintf(fp,"%-6.4f  %-6.4f\n",k,angl);
	}
	fclose(fp);
printf("12\n");	

	fp = fopen("/h0/user/soren/lab/nearfar13.out","w");
	for(i=0;i<maxhole;i++)
	{
		p[0] = hole_ref[i][0];
		p[1] = hole_ref[i][1];
		n = 3;
		Find_far(p,pt,hole_ref,&i,&maxhole,&n);
		p1[0] = pt[1][0]; p1[1] = pt[1][1];
		n = 1;
		Find_near(p,pt,hole_ref,&i,&maxhole,&n);
		Find_ang_k(p,pt[1],p1,&k,&angl);
		fprintf(fp,"%-6.4f  %-6.4f\n",k,angl);
	}
	fclose(fp);
printf("13\n");	

	fp = fopen("/h0/user/soren/lab/nearfar14.out","w");
	for(i=0;i<maxhole;i++)
	{
		p[0] = hole_ref[i][0];
		p[1] = hole_ref[i][1];
		n = 4;
		Find_far(p,pt,hole_ref,&i,&maxhole,&n);
		p1[0] = pt[1][0]; p1[1] = pt[1][1];
		n = 1;
		Find_near(p,pt,hole_ref,&i,&maxhole,&n);
		Find_ang_k(p,pt[1],p1,&k,&angl);
		fprintf(fp,"%-6.4f  %-6.4f\n",k,angl);
	}
	fclose(fp);
printf("14\n");	

	fp = fopen("/h0/user/soren/lab/nearfar15.out","w");
	for(i=0;i<maxhole;i++)
	{
		p[0] = hole_ref[i][0];
		p[1] = hole_ref[i][1];
		n = 5;
		Find_far(p,pt,hole_ref,&i,&maxhole,&n);
		p1[0] = pt[1][0]; p1[1] = pt[1][1];
		n = 1;
		Find_near(p,pt,hole_ref,&i,&maxhole,&n);
		Find_ang_k(p,pt[1],p1,&k,&angl);
		fprintf(fp,"%-6.4f  %-6.4f\n",k,angl);
	}
	fclose(fp);
printf("15\n");	

	fp = fopen("/h0/user/soren/lab/nearfar21.out","w");
	for(i=0;i<maxhole;i++)
	{
		p[0] = hole_ref[i][0];
		p[1] = hole_ref[i][1];
		n = 2;
		Find_near(p,pt,hole_ref,&i,&maxhole,&n);
		p1[0] = pt[1][0]; p1[1] = pt[1][1];
		n = 1;
		Find_far(p,pt,hole_ref,&i,&maxhole,&n);
		Find_ang_k(p,p1,pt[1],&k,&angl);
		fprintf(fp,"%-6.4f  %-6.4f\n",k,angl);
	}
	fclose(fp);
printf("21\n");	

	fp = fopen("/h0/user/soren/lab/nearfar31.out","w");
	for(i=0;i<maxhole;i++)
	{
		p[0] = hole_ref[i][0];
		p[1] = hole_ref[i][1];
		n = 3;
		Find_near(p,pt,hole_ref,&i,&maxhole,&n);
		p1[0] = pt[1][0]; p1[1] = pt[1][1];
		n = 1;
		Find_far(p,pt,hole_ref,&i,&maxhole,&n);
		Find_ang_k(p,p1,pt[1],&k,&angl);
		fprintf(fp,"%-6.4f  %-6.4f\n",k,angl);
	}
	fclose(fp);
printf("31\n");	

	fp = fopen("/h0/user/soren/lab/nearfar41.out","w");
	for(i=0;i<maxhole;i++)
	{
		p[0] = hole_ref[i][0];
		p[1] = hole_ref[i][1];
		n = 4;
		Find_near(p,pt,hole_ref,&i,&maxhole,&n);
		p1[0] = pt[1][0]; p1[1] = pt[1][1];
		n = 1;
		Find_far(p,pt,hole_ref,&i,&maxhole,&n);
		Find_ang_k(p,p1,pt[1],&k,&angl);
		fprintf(fp,"%-6.4f  %-6.4f\n",k,angl);
	}
	fclose(fp);
printf("41\n");	

	fp = fopen("/h0/user/soren/lab/nearfar51.out","w");
	for(i=0;i<maxhole;i++)
	{
		p[0] = hole_ref[i][0];
		p[1] = hole_ref[i][1];
		n = 5;
		Find_near(p,pt,hole_ref,&i,&maxhole,&n);
		p1[0] = pt[1][0]; p1[1] = pt[1][1];
		n = 1;
		Find_far(p,pt,hole_ref,&i,&maxhole,&n);
		Find_ang_k(p,p1,pt[1],&k,&angl);
		fprintf(fp,"%-6.4f  %-6.4f\n",k,angl);
	}
	fclose(fp);
printf("51\n");	

	fp = fopen("/h0/user/soren/lab/nearfar61.out","w");
	for(i=0;i<maxhole;i++)
	{
		p[0] = hole_ref[i][0];
		p[1] = hole_ref[i][1];
		n = 6;
		Find_near(p,pt,hole_ref,&i,&maxhole,&n);
		p1[0] = pt[1][0]; p1[1] = pt[1][1];
		n = 1;
		Find_far(p,pt,hole_ref,&i,&maxhole,&n);
		Find_ang_k(p,p1,pt[1],&k,&angl);
		fprintf(fp,"%-6.4f  %-6.4f\n",k,angl);
	}
	fclose(fp);
printf("61\n");	

}

VisInstruktion(char *msg1, char *msg2)
{
	printf(msg1);
	printf("\n");
}
