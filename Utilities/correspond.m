function [zf, idf, zn, cList] = correspond(x, z, idz, cList)
    % Maintains a measurement corresponding list. 
    % Adapted from code by Tim Bailey
    % Inputs:
    %    x    - global state covarianc.
    %    z    - observations.
    %   idz   - observations' ID.
    %   cList - corresponding list recoding the observations appearance order.
    % Outputs:
    %  zf,idf - old observations(already add to the list) and its ID.
    %     zn  - new observations.
    %   cList - updated corresponding list.
    %% Store old observations, new observations and their index.
    % Old observations used to update. New observations added to global augment state.
    zf  = []; zn  = [];
    idf = []; idn = [];
    %% Distinguish old and new observations.
    for i  = 1:length(idz)
        ii = idz(i);
        if cList(ii) == 0         % New observations.
            zn  = [zn z(:,i)];
            idn = [idn ii];       % ID.
        else                      % Old observations.
            zf  = [zf z(:,i)];
            idf = [idf cList(ii)];% Index to observation.
        end
    end
    %% add new features order to corresponding list
    Nxv        = 3;                   
    Nf         = (length(x) - Nxv)/2; % Observations number in map.
    cList(idn) = Nf + (1:size(zn,2)); % Add new observations' index
end