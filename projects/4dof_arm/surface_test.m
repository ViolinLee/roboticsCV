% data = rand(100,3) ;
% x = data(:,1) ; y = data(:,2) ; z = data(:,3) ;
x = x';y = y';z = z';
dt = delaunayTriangulation(x,y) ;
tri = dt.ConnectivityList ;
xi = dt.Points(:,1) ; yi = dt.Points(:,2) ;
F = scatteredInterpolant(x,y,z);
zi = F(xi,yi) ;
trisurf(tri,xi,yi,zi) 
view(2)
shading interp