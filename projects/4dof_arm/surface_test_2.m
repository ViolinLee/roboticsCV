x = [422   424   424   422   422  421   421   425   421    424];
y = [87   96   87   87   83    93    85   88    86   91];
z = [92.73  94.88  92.92  92.73  92.04  93.92  92.24  93.23  92.42  93.78];
figure(1)
stem3(x, y, z)
grid on
xv = linspace(min(x), max(x), 20);
yv = linspace(min(y), max(y), 20);
[X,Y] = meshgrid(xv, yv);
Z = griddata(x,y,z,X,Y);
figure(2)
surf(X, Y, Z);
grid on
set(gca, 'ZLim',[0 100])
shading interp