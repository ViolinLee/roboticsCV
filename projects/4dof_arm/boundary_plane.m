xx = x'; zz = z';
plot(xx,zz,'.')

len = length(xx);
xx(len+1) = xx(1); zz(len+1) = zz(1);

k = boundary(xx,zz);% generate boundary of data points. %this is index of all point located in the boundary
hold on;
plot(xx(k),zz(k));
X=xx(k);Y=zz(k);%return the original x,y coordinate corresponding to each index k
A = polyarea(X,Y);%calculate the area of boundary