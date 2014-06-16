global robpar ts pose time;
obj=zeros(1,41);
robpar=[0.26,0.035,0.035];
nsam=20;
ts=0.01;
ts1=nsam*ts;
targetpose=[3 -0.6 0];
pose=zeros(100000,7);
time=1;
pose(1,:)=[0,0,0,0,0,0,0];
v=0.2;
clf
obstaclepos=[1.2 -0.15
             1.2 0
             1.2 0.3
             0.9  0.3];
obstaclepos=[1 0];         
if (1)
obstaclepos=[0.1,0.3;
             0.4 0.3
             0.7 0.3
             0.9 0.3
             1.0 0.3
             1.0 0
             1.0 -0.3
             0.1,-0.3
             0.4 ,-0.3
             0.4 -0.6
             0.4 -0.9
             0.7 -0.9
             1.0 -0.9
             ];
end         
plot(obstaclepos(:,1),obstaclepos(:,2),'r*')   
axis equal
hold on
[nobst,temp]=size(obstaclepos);
wc=0;
dwcon=0.05;
dwmax=40*dwcon;
cornersrob=[-0.05 -0.15
            0.25 -0.15
            0.25  0.15
            -0.05  0.15];
        mirror=[ones(nobst,1) -ones(nobst,1)];
        
while( time < 1000)
if (rem(time,nsam)==1)
  throb=pose(time,3);
  posrob=pose(time,1:2);
  for j=1:4
    corners(j,:)=[cos(throb) -sin(throb);sin(throb) cos(throb)]*cornersrob(j,:)'+posrob';
  end  
  plot([corners(:,1)' corners(1,1)],[corners(:,2)' corners(1,2)])
  for j=1:nobst
    obstposrob(j,:)=[cos(throb) sin(throb);-sin(throb) cos(throb)]*(obstaclepos(j,:)'-posrob');
  end 
  %figure(2)
 % clf
  %plot(obstposrob(:,1),obstposrob(:,2),'r*')  
  
   %figure(1)
   for i=1:41
       w=(i-21)*dwcon        
       if (w==0)
          R=-1;
       else
          R=abs(v/w)
       end    
         if(w < 0)
           [d,th]=obstacle1(obstposrob,R,[0.22,0.25,0.05]);
         else
          [d,th]=obstacle1(obstposrob.*mirror,R,[0.22,0.25,0.05]);
         end
       [dmin,id]=min(d);
       dmin=min(dmin,2);
      tgp=targetpose-pose(time,1:3);
      th1=atan2(tgp(2),tgp(1));
      head=1-abs(th1-throb-w*ts1)/pi;
      du=1-abs((i-21)*dwcon-wc)/dwmax;
      obj1(i)=dmin/2;
      obj2(i)=head;
      obj3(i)=du;
      obj(i)=obj1(i)+obj2(i);
    
   end
   obj;
   obj2;
   obj3;
   obj;
 
   [m,i]=max(obj);
   wc=(i-21)*dwcon;
end
  w0=v/robpar(2);
  dw=0.5*robpar(1)/robpar(2)*wc;
  pose(time,4:5)=[v,wc*180/pi];
  wr=w0+dw;
  wl=w0-dw;
  pose(time+1,1:3)=kinupdate(pose(time,1:3),robpar,ts,[wr, wl])';
  time=time+1;
  
end