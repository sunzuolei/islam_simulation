function zcov = makeFcov(x, P, nsig, n)
    % Make features covariance
    % Inputs:
    %   nsig - number of sigmas for ellipsoid bound.
    %    n   - number of lines in polyline.
    % Outputs:
    %   zcov - sigma covariance.
    %%
    lenx = length(x);
    lenf = (lenx-3)/2;
    zcov = zeros(2,lenf*(n+2)); % Feature covariance
    ctr  = 1;
    for i=1:lenf
        ii         = ctr:(ctr+n+1);
        jj         = 2+2*i; jj = jj:jj+1;
        k          = i*2-1; k  =  k:k+1;
        zcov(:,ii) = getSigmaEllipse(x(jj), P(1:2,k), nsig, n);
        ctr        = ctr + n + 2;
    end
end
