function [x, P] = updateEKF(x, P, z, W, idf, openChol)
    % Inputs:
    %   z, W     - range-bearing measurements(old) and their covariances.
    %   idf      - feature ID for each z.
    %   openChol - switch to specify cholesky decomposition or simple decomposition.
    % Outputs:
    %   x, P     - the posterior state and covariance.
    %%
    lenz  = size(z,2);
    for i = 1:lenz
        [zv,H] = obsModel(x, idf(i));
         Y     = [z(1,i)-zv(1);                % Innovation
                  piTopi(z(2,i)-zv(2))];
        if openChol ~= 0
            [x, P]  = cholesky(x, P, Y, W, H); % Cholesky update.
        else
            [x, P]  = simple(x, P, Y, W, H);   % Simple update.
        end
    end 
    end

    function [x, P] = cholesky(x, P, v, R, H)
    % The result is calculated using Cholesky factorisation, which
    % is more numerically stable than a naive implementation.
    %%
    PH    = (H * P)';
    S     = H * PH + R;
    S     = (S+S')*0.5;
    Schol = chol(S);
    Si    = inv(Schol);
    W1    = PH / Schol;
    W     = W1 * Si';
    x     = x + W*v;
    P     = P - W1*W1';
    end

    function [x, P] = simple(x, P, v, R, H)
    %%
    S  = H * P * H' + R;
    W  = P * H' / S;
    x  = x + W * v; 
    P  = P - W * S * W';
    end
