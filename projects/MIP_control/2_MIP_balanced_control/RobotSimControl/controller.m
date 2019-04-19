
function u = controller(params, t, phi, phidot)
  % STUDENT FILLS THIS OUT
  % 
  % Initialize any added state like this:
  % 
  % persistent newstate
  % if isempty(newstate)
  %   % initialize
  %   newstate = 0;
  % end
  % Note from LeeChan: MATLAB clears persistent variables when you clear or modify a function that is in memory.
  % So we can text 'clear controller' to clear persistent variables
  
  %% Initialize states using persistent variables
  persistent T ei_T
  if isempty(T)
      % initialize
      T = t; ei_T = 0;
  end
  
  dt = t - T;
  T = t;
  
  %% Controller
  kp = 38;
  kd = 0.4;
  ki = 800;
  ei_T = ei_T + (0-phi)*dt;
  
  u = kp*(0-phi) + kd*(0-phidot) + ki*(ei_T);
  u = -u;

end

