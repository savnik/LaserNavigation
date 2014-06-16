cd /vhome/naa/smr05
load /vhome/naa/robot2005/src/smrdemo/log
optpar=odocalib1([0.26 1],log)
fid=fopen('tempcalib','w')
fprintf(fid,'%8.5f %8.5f  %10.8f \n',optpar(1),optpar(2),0.0001026);
fclose(fid)
