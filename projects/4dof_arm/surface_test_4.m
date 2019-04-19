T = x; D = y; K = z;

[X,Y,Z]=griddata(T,D,K,linspace(min(T),max(T))',linspace(min(D),max(D)),'v4');
figure,surf(X,Y,Z);