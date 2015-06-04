function [x, P] = augmentState(x, P, z, W)
    % Inputs:
    %   z, W - range-bearing observations(new) and its covariance.
    % Outputs:
    %   x, P - global state and covariance.
    %%
    for i = 1:size(z,2)
        lenx = length(x);
        r = z(1,i);      b = z(2,i);
        s = sin(x(3)+b); c = cos(x(3)+b);
       %% Jacobians
        Gv = [1 0 -r*s;
              0 1  r*c];
        Gz = [c   -r*s;
              s    r*c];
       %% Augment x
         x = [x;
              x(1) + r*c;
              x(2) + r*s];
       %% Augment covariance
        rng        = lenx+1:lenx+2;
        P(rng,rng) = Gv * P(1:3,1:3) * Gv' + Gz * W * Gz'; % Features' covariance
        P(rng,1:3) = Gv * P(1:3,1:3);   % Covariance between robot and features.
        P(1:3,rng) = P(rng,1:3)';
        if lenx>3
            rnm        = 4:lenx;
            P(rng,rnm) = Gv * P(1:3,rnm);
            P(rnm,rng) = P(rng,rnm)';
        end
    end
end
