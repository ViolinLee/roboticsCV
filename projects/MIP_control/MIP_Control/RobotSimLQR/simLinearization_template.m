
params = struct();

params.g = 9.81;
params.mr = 0.25;
params.ir = 0.0001;
params.d = 0.1;
params.r = 0.02;

% 1. Learn how to use the symbolic toolbox in MATLAB
% you will need the state variables, and control input to be declared as symbolic

% 2. call your "eom" function to get \ddot{q} symbolically

% 3. Linearize the system at 0 (as shown in lecture)
% You should end up with A (4x4), and b (4x1)

% 4. Check that (A,b) is  controllable
% Number of uncontrollable states should return 0

% 5. Use LQR to get K as shown in the lecture
