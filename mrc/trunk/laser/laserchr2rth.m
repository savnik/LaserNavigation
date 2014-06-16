function sc=laserchr2rth(scanchr)
  th=(scanchr(4)+(0:scanchr(5)-1)*scanchr(3))/180*pi;
  sc=[th;scanchr(6:scanchr(5)+5)];
