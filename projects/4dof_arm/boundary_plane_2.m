x = x'; y = z';

%# convex hull
dt = DelaunayTri(x,y);
k = convexHull(dt);

%# area of convex hull
ar = polyarea(dt.X(k,1),dt.X(k,2))

%# plot
plot(dt.X(:,1), dt.X(:,2), '.'), hold on
fill(dt.X(k,1),dt.X(k,2), 'r', 'facealpha', 0.2);
hold off
title( sprintf('area = %g',ar) )