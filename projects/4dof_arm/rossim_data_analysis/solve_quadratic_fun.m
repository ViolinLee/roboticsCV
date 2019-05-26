syms a; syms b; syms c;

eq1 = '1.2 = -(0 - a*b)^2 + c';
eq2 = '1.2 = -(a*60 - a*b)^2 + c';
eq3 = '1.4 = -(a*30 - a*b)^2 + c';

[a,b,c] = solve(eq1, eq2, eq3, 'a', 'b', 'c')

a1 = double(a(1));
b1 = double(b(1));
c1 = double(c(1));

x = linspace(0,60,61);
z_true = -(a1*x - a1*b1).^2 + c1;

plot(z_true)