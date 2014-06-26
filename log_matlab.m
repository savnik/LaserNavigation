clear;
M = dlmread('/misc/shome/ex25/LaserNavigation/log');

hold on;

[rows,cols] = size(M);
scatter( M(10,1), M(10,2), 2, [1 0 0], 'filled');


for i = 1:rows
   scatter(M(i,5),M(i,6),2,[0 1 0],'filled');
end

sn = @(x) -0.0706151*x+(0.351056); 
 fplot(sn,[0,2])

hold off;

