%% Set initial conditions
clear all;
dt = 1/100;
duration = 10;
indexLimit = round(duration/dt);
statesLog = zeros(10,indexLimit);
quatLog   = zeros(4,indexLimit);
velInnovLog = zeros(3,indexLimit);
decInnovLog = zeros(1,indexLimit);
velInnovVarLog = velInnovLog;
decInnovVarLog = decInnovLog;
% Use a random initial orientation
quatTruth = [rand;randn;randn;randn];
quatLength = sqrt(quatTruth(1)^2 + quatTruth(2)^2 + quatTruth(3)^2 + quatTruth(4)^2);
quatTruth = quatTruth / quatLength;
TbnTruth = Quat2Tbn(quatTruth);
% initialise the filter to level
quat = [1;0;0;0];
states = zeros(10,1);
Tsn = Quat2Tbn(quat);
% define the earths truth magnetic field
magEarthTruth = [0.3;0.1;-0.5];
% define the initial declination using the truth field plus location
% variation
decInit = atan2(magEarthTruth(2),magEarthTruth(1)) + 2*pi/180*randn;
states(10) = decInit;
% define the magnetometer bias errors
magMeasBias = 0.02*[randn;randn;randn];
% define the state covariances with the exception of the quaternion covariances
Sigma_velNED = 0.5; % 1 sigma uncertainty in horizontal velocity components
Sigma_dAngBias  = 1*pi/180*dt; % 1 Sigma uncertainty in delta angle bias
Sigma_angErr = 1; % 1 Sigma uncertainty in angular misalignment (rad)
Sigma_decErr = 0.1; % 1 sigma uncertainty in declination (rad)
covariance   = single(diag([Sigma_angErr*[1;1;1];Sigma_velNED*[1;1;1];Sigma_dAngBias*[1;1;1];Sigma_decErr^2]).^2);
% Define initial gimbal angles
gPhi = 0;
gTheta = 0;
gPsi = 0;
%% Main Loop
headingAligned=0;
for index = 1:indexLimit
    % synthesise IMU measurements
    angRate = 0*[randn;randn;randn];
    accel = 0*[randn;randn;randn] + transpose(TbnTruth)*[0;0;-9.81];
    % predict states
    [quat, states, Tsn, delAng, delVel]  = PredictStates(quat,states,angRate,accel,dt);
    statesLog(:,index) = states;
    quatLog(:,index) = quat;
    % predict covariance matrix
    covariance  = PredictCovariance(delAng,delVel,quat,states,covariance,dt);
    % synthesise velocity measurements
    measVel = [0;0;0];
    % fuse velocity measurements
    [quat,states,angErr,covariance,velInnov,velInnovVar] = FuseVelocity(quat,states,covariance,measVel);
    velInnovLog(:,index) = velInnov;
    velInnovVarLog(:,index) = velInnovVar;
    % synthesise magnetometer measurements adding sensor bias
    magBody = transpose(TbnTruth)*magEarthTruth + magMeasBias;
    % fuse magnetometer measurements
    if (index > 500 && headingAligned==0 && angErr < 1e-4)
        % calculate the initial heading using magnetometer, gimbal,
        % estimated tilt and declination
        quat = AlignHeading(gPhi,gPsi,gTheta,Tsn,magBody,quat,decInit);
        headingAligned = 1;
    end
    if (headingAligned == 1)
        [quat,states,covariance,decInnov,decInnovVar] = FuseMagnetometer(quat,states,covariance,magBody,gPhi,gPsi,gTheta);
        decInnovLog(:,index) = decInnov;
        decInnovVarLog(:,index) = decInnovVar;
    end
    
end