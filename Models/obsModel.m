function [z, H] = obsModel(x, idf)
    % Inputs:
    %   x   - state vector.
    %   idf - index of observations.
    % Outputs:
    %   z   - predicted observation
    %   H   - jacobian about observation.
    %%
    Nxv  = 3;                 % Dimension of robot pose
    fpos = Nxv + idf * 2 - 1; % Order of measurements in global state vector.
    H    = zeros(2, length(x));

    % Auxiliary values
    dx  = x(fpos)  -x(1); dy  = x(fpos+1)-x(2);
    d2  = dx^2 + dy^2;    d   = sqrt(d2);
    xd  = dx/d;  yd  = dy/d;
    xd2 = dx/d2; yd2 = dy/d2;
    %% Predict measurements.
    z = [d;
         atan2(dy,dx) - x(3)];
    %% Calculate H
    H(:,1:3)         = [-xd  -yd  0; yd2  -xd2  -1];
    H(:,fpos:fpos+1) = [ xd   yd;   -yd2   xd2];
end