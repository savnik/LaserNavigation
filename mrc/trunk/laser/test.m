load laser_labdoor01.log
[r,c]=size(laser_labdoor01)
for i=500:1:r
ztmp=laser_labdoor01(i,:);
i
delete ztmp
save -ASCII ztmp ztmp
[s,res]=system('testlaserdata ztmp');
sc=laserchr2rth(ztmp);
scxy=polarcarth(sc);
hold off 
plot(scxy(1,:),scxy(2,:),'.');
hold on
a=strread(res);
p1=a(1,1)+1
p2=a(1,2)+1;
x=a(2,1);
y=a(2,2);
th=a(2,3);
plot(scxy(1,p1),scxy(2,p1),'r*');
plot(scxy(1,p2),scxy(2,p2),'r*');

plot([x x-cos(th)],[y y-sin(th)],'r');
pause
end
 
 

