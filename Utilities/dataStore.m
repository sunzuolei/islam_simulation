function data = dataStore(data, x, P, z, i)
    %% Store online data
    data.path(:,i) = x(1:3);
    data.pos(i).x  = x;          % Robot pose
    data.cov(i).Pr = P(1:3,1:3); % Robot covariance.
    data.pos(i).z  = z;          % Feature pose.
    lenf           = (length(x)-3)/2;
    fcov           = [];
    for k = 1: lenf
        j    = 3 + k*2;
        jj   = j-1:j;
        fcov =[fcov P(jj,jj)];
    end
    data.cov(i).Pf  = fcov;       % Feature covariance.
end   