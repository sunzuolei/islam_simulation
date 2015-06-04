function un = conNoise(u, Q)
    % Add random noise to nominal control values. 
    % We assume Q is diagonal.
    % Inputs:
    %   u - control vector.
    %   Q - control noises covariance for filter.
    %
    % Outputs:
    %   Un - control variance with noise
    %%
    un(1) = u(1) + randn(1) * sqrt(Q(1,1));
    un(2) = u(2) + randn(1) * sqrt(Q(2,2));
end