len = length(x);
y = ones(1, len);
xx = x'; zz = z'; yy = y';

P(:,1) = x'; P(:,2) = y'; P(:,3) = z';
P(len+1:2*len, 1) = xx(:,1);
P(len+1:2*len, 3) = zz(:,1);
P(len+1:2*len, 2) = 2*ones(1, length(x));

[k,v] = boundary(P,1);
hold on
trisurf(k,P(:,1),P(:,2),P(:,3),'Facecolor','red','FaceAlpha',0.1)