function pc = polarcarth(p)
for j = 1:length(p),
    x(j) = p(2,j)*cos(p(1,j));
    y(j) = p(2,j)*sin(p(1,j));   
end
pc = [x;y];