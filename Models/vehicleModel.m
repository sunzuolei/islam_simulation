function [xTrue, U]= vehicleModel(x, u, wb, wp, minD, maxRate, G, dt)
    % INPUTS:
    %   x, u, wb - true position, control vector and wheelbase.
    %     wp     - waypoints.
    %    minD    - minimal distance to current waypoint before switching to the next one.
    %    rateG   - max steering rate (rad/s).
    %    maxG    - max steering angle (rad).
    %     dt     - timestep.
    % OUTPUTS:
    %   G   - new current steering angle.
    %  iwp  - new current waypoint.
    %   x   - new vehicle pose.
    %%
    iwp   = 1; % Index for the first waypoint.
    i     = 1;
    xTrue = x;
    U     = u;
    v     = u(1);
    g     = u(2);
    %% Allocate memory
    while iwp ~= 0
        %% Determine whether the current waypoint reach or not.
        cwp = wp(:,iwp);
        d2  = (cwp(1)-x(1))^2 + (cwp(2)-x(2))^2;
        if d2 < minD^2
            iwp = iwp + 1;      % Switch to the next waypoint.
            if iwp > size(wp,2) % Reach the final waypoint, flag and return.
                return;
            end    
            cwp = wp(:,iwp);    % next waypoint.
        end
        %%
        delta    = piTopi(atan2(cwp(2) - x(2), cwp(1) - x(1)) - x(3) - g);
        % limit rate
        maxDelta = maxRate*dt;
        if abs(delta) > maxDelta
            delta     = sign(delta) * maxDelta;
        end
        % limit angle
        g = g + delta;
        if abs(g) > G
            g = sign(g)*G;
        end
        %% true robot position
        x = [x(1) + v*dt*cos(g+x(3,:)); 
             x(2) + v*dt*sin(g+x(3,:));
             piTopi(x(3) + v*dt*sin(g)/wb)];
        i = i+1;
        U(:,i)     = [v;g];
        xTrue(:,i) = x;      
    end
