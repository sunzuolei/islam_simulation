dbstop if error;
clear all; close all;
path(path, genpath('../EKF and IEKF Feature-based SLAM'));
load 'Data/rollerCoaster.mat';
rng(23); % Generate random seed
dtsum = 0;
%% Switches
visualize= 1;   % If zero, there is no demo.
openChol = 0;   % If non-zero open cholesky update
openIEKF = 0;   % If zero, open EKF-SLAM; otherwise,open
                % IEKF-SLAM, and the value is the iterative number.
%% Parametres configuration
xInit    = [0;0;(-2/pi)-(2/pi)];  % Initialize true pose
pos      = xInit;                 % Initialize estimated pose.
cov      = zeros(3);              % Initialize estimated covariance.
z        = zeros(2,1);            % Observations.
idList   = zeros(1,size(lm,2));   % Index table for each observations.
%% Control parametersve
UInit    = [4; 0];    % Control vector, inculding velocity and steering.
                      % units are (m/s) and (radians), respectively.
convert  = 1.0;       % (metres). Distance from current waypoint at 
                      % which to switch to the next waypoint
maxUs    = 30*pi/180; % (radians). Maximal steering angle (-angel < Us < angle).
maxRate  = 20*pi/180; % (rad/s). Maximal palstance.
wheelbase= 4;         % (m). The length of wheelbase.
dt       = 0.025*4;   % (s). Time interval of control signals.
%% Measurement parameters
maxRange = 30;        % (metres). Maximal measurement range for a sensor.
%% For data association
gate1 = 4.0;  % Maximal distance for association
gate2 = 25.0; % Minimal distance for a new feature
%
%% Turn noises
conNoiseScalar = 1.0; % Tune process noise scalar.
obsNoiseScalar = 1.0; % Tune observation noise scalar.
%% Process noise for filter
sigmaUvNoise = 0.7;
sigmaUsNoise = (3*pi/180);
Q = (conNoiseScalar * diag([sigmaUvNoise,sigmaUsNoise])).^2;
%% Measurements noises for filter
sigmaRNoise = 0.3;
sigmaBNoise = (4*pi/180);
W  = (obsNoiseScalar * diag([sigmaRNoise, sigmaBNoise])).^2;
%% Noise for simulating truths
simQ = diag([sigmaUvNoise, sigmaUsNoise]).^2;
simW = diag([sigmaRNoise, sigmaBNoise]).^2;
%
%% Main loop
main;    
%% Animation is implemented by a function
vis(lm, wp, xTrue, data, step, visualize);