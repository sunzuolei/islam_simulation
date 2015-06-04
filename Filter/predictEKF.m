function [x, P] = predictEKF(x, P, u, Q, L, dt)
    % This function will effect the global state and covariance.
    % That is, the robot pose in x and P will change.
    % Inputs:
    %  x, P - global state and its covariance.
    %   u   - control vector with noise.
    %   Q   - process noise covariance.
    %   L   - wheelbase of robot.
    %   dt  - time interval of control signals.
    % Outputs:
    %  x, P - the prior state and covariance
    %%
    v   = u(1); g = u(2);  % Velocity and steering.
    s   = sin(g + x(3)); c   = cos(g + x(3));
    vts = v * dt * s;    vtc = v * dt * c;
    %% Jacobian 
    Fr = [1  0  -vts;
          0  1   vtc;
          0  0   1];
    Fu = [dt*c         -vts;
          dt*s          vtc;
          dt*sin(g)/L  v*dt*cos(g)/L]; 
    %% Predict covariance
    P(1:3,1:3) = Fr * P(1:3,1:3) * Fr' + Fu * Q * Fu';
    if size(P,1)>3
        P(1:3,4:end) = Fr * P(1:3,4:end);
        P(4:end,1:3) = P(1:3,4:end)';
    end    
    %% Predict robot pose
     x(1:3)= [x(1) + vtc; 
              x(2) + vts;
              piTopi(x(3)+ v*dt*sin(g)/L)];
end
    