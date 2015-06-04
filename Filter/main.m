%% Mainloop for simulation
%% True path simulation
[xTrue, U] = vehicleModel(xInit, UInit, wheelbase, wp, ...
                      convert, maxRate, maxUs, dt); 
step       = size(U,2);
%% Initialise data( struct 'data' for storing data.)
data.path      = zeros(3, step); % Robot estimated path.
data.path(:,1) = pos;
data.pos(1).x  = pos;            % Global state.
data.pos(1).z  = zeros(2,1);     % Obsevations in every step.
data.cov(1).Pr = cov;            % Robot self-covariance
data.cov(1).Pf = zeros(2,2);     % Feature self-covariance
%% Filtering
tic;
for i = 2:step
      un        = conNoise(U(:,i), simQ); % Add noise to control vector.
     %% Predict
     [pos, cov] = predictEKF(pos, cov, un, Q, wheelbase, dt);
     %% Observe every step 
     [z,idz] = getLandmark(xTrue(:,i), lm, maxRange, simW);
     %% Corresponding.
     [zf, idf, zn, idList] = correspond(pos, z, idz, idList);
     %% Update 
     if openIEKF ~= 0
          [pos, cov] = updateIEKF(pos, cov, zf, W, idf, openIEKF);
     else
          [pos, cov] = updateEKF(pos, cov, zf, W, idf, openChol);
     end 
   
          [pos, cov] = augmentState(pos, cov, zn, W);
     %%
     data  = dataStore(data, pos, cov, z, i);
end
toc;


        
        