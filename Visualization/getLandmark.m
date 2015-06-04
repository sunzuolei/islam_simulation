function [z,idz]= getLandmark(x, lm, rmax, W)
    % For getting the features within robot's visual range.
    % Adapted from the codes of Tim Bailey.
    % Inputs:
    %    x   - robot pose [x;y;phi].
    %    lm  - set of all landmarks.
    %   rmax - maximum range of range-bearing sensor. 
    %    W   - observations' covariances.
    % Outputs:
    %    z   - observed landmarks.
    %   idz  - ID tag for each observed landmarks.
    %% Select landmarks within robot's semi-circular field-of-view.
    dx  = lm(1,:) - x(1);
    dy  = lm(2,:) - x(2);
    phi = x(3);
    % Return the position of eligible landmarks.
    ii = find(abs(dx) < rmax & abs(dy) < rmax ... % Bounding box.
          & (dx*cos(phi) + dy*sin(phi)) > 0 ...   % Bounding line.
          & (dx.^2 + dy.^2) < rmax^2);            % Bounding circle.
    % Note: the bounding box test is unnecessary but illustrates a
    % possible speedup technique as it quickly eliminates distant points. 
    % Ordering the landmark set would make this operation
    % O(logN) rather that O(N).
    %% Store observations in vision.
    obs = lm(:,ii);
    idz = ii; % observations ID number in 'lm'.
    %% Observations' true position.
    dx  = obs(1,:) - x(1);
    dy  = obs(2,:) - x(2);
    phi = x(3);
    z   = [sqrt(dx.^2 + dy.^2);
           atan2(dy,dx) - phi];
           len= size(z,2);
    %% Add noise to observations.   
    if len > 0
     z(1,:) = z(1,:) + randn(1,len) * sqrt(W(1,1));
     z(2,:) = z(2,:) + randn(1,len) * sqrt(W(2,2));
    end
end