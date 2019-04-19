len = length(x);
xx = [];zz = [];
count = 1;
for i=1:1:len
    if y(i) == 0
       xx(count) = x(i);
       zz(count) = z(i);
       count = count + 1;
    end
end

scatter(xx,zz,'g*')
axis equal