  if (camok)
  {
#define LW	160
#define CW	120
#define FW	10
#define DLIM	40
#define NCOL	120
       unsigned char *line,col[CW];
       double y[LW],dy[LW],lmin,dmax,dmin,crosspos;
       int  buf[FW],pos,pdmax,pdmin;
       image = getNewRawImage(&rcm, true);
       if (image !=NULL) {
         line=getYline(image,100);
         for (i=0;i<CW;i++)
           col[i]=image->py[i*LW+NCOL];
         if (imgptr <10000) {
           for (i=0;i<CW;i++)
             imglog[imgptr][i]=col[i];
           imgptr++;
         }
         movavg(y,buf,FW,LW,line);
         lmin=min(&pos,LW,y);
         for (i=0;i  <(LW-FW);i++)
           dy[i]=y[i+FW]-y[i];
         for (i=(LW-FW); i<LW;i++)
           dy[i]=0;
         dmax=max(&pdmax,LW-2*FW,&dy[FW]);
         dmin=min(&pdmin,LW-2*FW,&dy[FW]);   
         safechange(&nolinecamr, (dmax  < DLIM || dmin > -DLIM),3,
		    (dmax > DLIM && dmin < -DLIM ),3,&lcountr);       
         if (!lcountr){
           camlinepos=-((pdmax+pdmin)/2+1+2*FW-80)/5.0*0.3;
//         printf(" %lf\n",camlinepos);
         }
	 movavg(y,buf,FW,CW,col);
	 lmin=min(&pos,CW,y);
	 for (i=0;i  <(CW-FW);i++)
	   dy[i]=y[i+FW]-y[i];
	 for (i=(CW-FW); i<CW;i++)
	   dy[i]=0;
	 dmax=max(&pdmax,CW-2*FW,&dy[FW]);
	 dmin=min(&pdmin,CW-2*FW,&dy[FW]);
	 dlog[0]=dmax;
	 dlog[1]=dmin;
	 safechange(&crossingliner, (dmax  > DLIM && dmin < -DLIM),3,
		    (dmax < DLIM || dmin > -DLIM ),3,&lcountr1);    
       
         crosspos=((pdmax+pdmin)/2+1+2*FW);
         crossingblackliner= crossingliner && (crosspos >90 && crosspos <100);
         dlog[2]=crossingliner;
         dlog[3]=crosspos;
      }
  }

