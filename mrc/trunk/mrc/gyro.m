load log
dv=(log-mean(log(1:490)))/12.5
th=filter(0.01*[0.5 0.5],[1 -1],dv);
plot(th)
