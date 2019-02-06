
params = struct();

params.g = 9.81;
params.mr = 0.25;
params.ir = 0.0001;
params.d = 0.1;
params.r = 0.02;

% 1. Learn how to use the symbolic toolbox in MATLAB
% you will need the state variables, and control input to be declared as symbolic
syms th phi dth dphi u
X = [th phi dth dphi];

% 2. call your "eom" function to get \ddot{q} symbolically
qdd = eom_custom(params, th, phi, dth, dphi, u);

% 3. Linearize the system at 0 (as shown in lecture)
% You should end up with A (4x4), and b (4x1)
Xd = [dth, dphi, qdd(1), qdd(2)];
Xd_x = jacobian(Xd, X);
Xd_u = jacobian(Xd, u);

A = subs(Xd_x, {th,phi,dth,dphi,u}, {0,0,0,0,0});
b = subs(Xd_u, {th,phi,dth,dphi,u}, {0,0,0,0,0});
A = double(A);
b = double(b);

% 4. Check that (A,b) is  controllable
% Number of uncontrollable states should return 0
Co = ctrb(A,b);
unco = length(A) - rank(Co)

% 5. Use LQR to get K as shown in the lecture
K = lqr(A, b, eye(4), 1, 0)
