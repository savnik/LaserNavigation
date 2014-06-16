R=0.08;
dim=[0.15 0.25 0.05];
pos=[0 0.16;
     0 -0.16]
[d th]=obstacle1(pos,R,dim)
f=fopen('testdata','w')
fprintf(f,'%f %f %f %f\n',[R dim]);    
fprintf(f,'%f %f %f %f\n',[pos' ;d ;th]);
fclose(f)    
