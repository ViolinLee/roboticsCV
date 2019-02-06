function qdd = eom_custom(params, th, phi, dth, dphi, u)
  % This is the starter file for the week5 assignment
  % Note from Leechan: Return the second derivative of th and phi
  
  % Provided params are
  % params.g: gravitational constant
  % params.mr: mass of the "rod"
  % params.ir: rotational inertia of the rod
  % params.d: distance of rod CoM from the wheel axis
  % params.r: wheel radius

  % Provided states are:
  % th: wheel angle (relative to body)
  % phi: body pitch
  % dth, dphi: time-derivatives of above
  % u: torque applied at the wheel

  qdd = [0;0]; 
  % THE STUDENT WILL FILL THIS OUT
  g = params.g;
  m = params.mr;
  i = params.ir;
  l = params.d;
  r = params.r;
  
  A = [ m*r^2,                                                                         m*r*(r + l*cos(phi));
        (m*r*(2*r + 2*l*cos(phi)))/2,   i + (m*((2*r + 2*l*cos(phi))*(r + l*cos(phi)) + 2*l^2*sin(phi)^2))/2];
 
  B = [                                                                                                                                                                                                                               l*m*r*sin(phi)*dphi^2 + u;
        (m*(2*l*sin(phi)*dphi*(r*(dphi + dth) + l*cos(phi)*dphi) - 4*l^2*cos(phi)*sin(phi)*dphi^2 + l*sin(phi)*dphi^2*(2*r + 2*l*cos(phi))))/2 + (m*(2*l^2*cos(phi)*sin(phi)*dphi^2 - 2*l*sin(phi)*dphi*(r*(dphi + dth) + l*cos(phi)*dphi)))/2 + g*l*m*sin(phi)];
  

%   A = [m*(r^2+r^2*dphi-r^2*dth+r*l*cos(phi))                                m*r^2;
%        m*r^2                                     m*(r^2+i-r^2*dth-r^2*dphi-r*l*cos(phi))-i*dphi];
%   B = [u+m*r*l*sin(phi);
%        m*(r*l*sin(phi)+l^2*cos(phi)*sin(phi)-r*dth*l*sin(phi)-r*l*dphi*sin(phi)+l^2*sin(phi)*cos(phi)+g*l*sin(phi))];
  
  qdd = linsolve(A,B); % X = linsolve(A,B) solves the linear system A*X = B using LU factorization with partial pivoting when A is square and QR factorization with column pivoting otherwise
  
end