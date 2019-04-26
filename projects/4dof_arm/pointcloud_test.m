x = 0.1 * x;
y = 0.1 * y;
z = 0.1 * z;

xyzpoints(:,1) = x'; xyzpoints(:,2) = y'; xyzpoints(:,3) = z';

pc = pointCloud(xyzpoints);
pcshow(pc);

xlabel('X/(m)')
ylabel('Y/(m)')
zlabel('Z/(m)')

