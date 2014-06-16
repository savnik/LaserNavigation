function [d,th]=obstacle(pos,R,dim)
dmax=2;
thmax=2*pi;
b=dim(1);
c=dim(2);
dr=dim(3);
[nob temp]=size(pos);
if (R<0)
  for i=1:nob
    th(i)=0;
    if(abs(pos(i,2))>b)
      d(i)=dmax;
    else
       if (pos(i,1)>c)
         d(i)=pos(i,1)-c;
         d(i)=min(d(i),dmax);
       else
         d(i)=dmax;
       end
    end
  end
else
rmin=R-b;
ra=sqrt(c^2+(R+b)^2);
rb=sqrt(c^2+(R-b)^2);
rc=sqrt(dr^2+(R-b)^2);
rd=sqrt(dr^2+(R+b)^2);    
v=pos+[zeros(nob,1) R*ones(nob,1)];
for i=1:nob
  radii(i)=v(i,:)*v(i,:)';
end
radii=sqrt(radii);
if (R > b)
for i=1:nob
      x=pos(i,1);
      y=pos(i,2);
      r=radii(i);
      if (y>b && r < rd)
          % thad
          %'case 1'
          th(i)=pi-asin((R+b)/r)-atan2(y+R,x);
      else
          if (r<rb )
              if (r >rmin)
                   %thbc
            %        'case 2'
                   th(i)=acos((b-R)/r)-atan2(x,-y-R);
              else
             %     'case 3'
                    th(i)=thmax;
              end
           else
               if (r < ra)
                    %thab
              %      'case4'
                    th(i)=pi-asin(c/r)-atan2(x,-y-R);
                    if (th(i) <0)
                       th(i)=th(i)+2*pi;
                    end
               else
               %    'case5'
                    th(i)=thmax;
               end
           end
      end
end
else 
    for i=1:nob
      x=pos(i,1);
      y=pos(i,2);
      r=radii(i);
      if (y>b && r < rd)
             % thad
             %'case 6'
             th(i)=pi-asin((R+b)/r)-atan2(y+R,x);
      else
          if(x < -dr && r <rc)
                   %thdc
                 %  'case7'
                  th(i)=-asin(dr/r)-atan2(x,-y-R);
          else
              if (r>ra)
                  %'case8'
                  th(i)=thmax;
              else
                  if (r > rb)
                      %thab
                     % 'case 9'
                      th(i)=pi-asin(c/r)-atan2(x,-y-R);
		      if (th(i) < 0)
		        th(i)=th(i)+2*pi;
		      end
                  else
                      %thbc
                    %  'case10'
                      th(i)=acos((b-R)/r)-atan2(x,-y-R);
		      if (th(i) < 0)
		        th(i)=th(i)+2*pi;
		      end;
                  end
              end
          end
      end
    end
end
d=th.*radii; 
end
             
             
    
