function [x, P] = updateIEKF(x, P, z, R, idf, N)
    % Inputs:
    %   z, R - range-bearing measurements(old) and their covariances.
    %   idf  - feature ID for each z.
    %   N    - iterative numbers of the IEKF update.
    %
    % Outputs:
    %   x, P - the posterior state and covariance.
    %%
    if isempty(idf), return; end
    lenz     = size(z,2);
    RIterate = zeros(2 * lenz); 
    zIterate = zeros(2 * lenz, 1);
    for i = 1:lenz
        j             = 2 * i + (-1:0);    
        zIterate(j)   = z(:,i);
        RIterate(j,j) = R;
    end
    [x,P] = iterate(x, P, zIterate, RIterate, ...
        @hmodel, @hjacobian, idf, N); 
end
%%
function v = hmodel(x, z, idf)
    lenz = length(idf);
    v    = zeros(2 * lenz, 1);
    for i= 1:lenz
        j       = 2 * i + (-1:0);    
        [zp,H]  = obsModel(x, idf(i));
        v(j)    = z(j)-zp;
        v(j(2)) = piTopi(v(j(2)));
    end
end
%%
function H = hjacobian(x, idf)
    lenz = length(idf);
    lenx = length(x);
    H    = zeros(2 * lenz, lenx);

    for i = 1:lenz
        j           = 2 * i + (-1:0);
        [zp,H(j,:)] = obsModel(x, idf(i));
    end
end