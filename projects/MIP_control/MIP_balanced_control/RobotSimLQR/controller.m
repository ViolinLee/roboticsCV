
function u = controller(params, t, X)
  % You have full state feedback available
  K = [-1.0000 -113.1949   -1.2465  -13.9340];
  % After doing the steps in simLinearization, you should be able to substitute the linear controller u = -K*x
  u = -K*X; % Note from LeeChan: We see that the controller has nothing to do with time t!
end

