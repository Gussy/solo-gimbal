%% Set initial conditions
clear all;
dt = 1/100;
duration = 10;
indexLimit = round(duration/dt);
time = zeros(1,indexLimit);
statesLog = zeros(10,indexLimit);
quatLog   = zeros(4,indexLimit);
eulLog = zeros(3,indexLimit);
eulErrLog = eulLog;
velInnovLog = zeros(3,indexLimit);
decInnovLog = zeros(1,indexLimit);
velInnovVarLog = velInnovLog;
decInnovVarLog = decInnovLog;
% Use a random initial truth orientation
quatTruth = [1;0.05*randn;0.05*randn;2*(rand-0.5)];
quatLength = sqrt(quatTruth(1)^2 + quatTruth(2)^2 + quatTruth(3)^2 + quatTruth(4)^2);
quatTruth = quatTruth / quatLength;
TsnTruth = Quat2Tbn(quatTruth);
% initialise the filter to level (let it find its own attitude)
quat = [1;0;0;0];
states = zeros(10,1);
Tsn = Quat2Tbn(quat);
% define the earths truth magnetic field
magEarthTruth = [0.3;0.1;-0.5];
% define the initial declination using the truth field plus uncertainty
decInit = atan2(magEarthTruth(2),magEarthTruth(1)) + 2*pi/180*randn;
states(10) = decInit;
% define the magnetometer bias errors
magMeasBias = 0.02*[randn;randn;randn];
% define the state covariances with the exception of the quaternion covariances
Sigma_velNED = 0.5; % 1 sigma uncertainty in horizontal velocity components
Sigma_dAngBias  = 1*pi/180*dt; % 1 Sigma uncertainty in delta angle bias
Sigma_angErr = 1; % 1 Sigma uncertainty in angular misalignment (rad)
Sigma_decErr = 2*pi/180; % 1 sigma uncertainty in declination (rad)
covariance   = single(diag([Sigma_angErr*[1;1;1];Sigma_velNED*[1;1;1];Sigma_dAngBias*[1;1;1];Sigma_decErr^2]).^2);
%% Main Loop
headingAligned=0;
for index = 1:indexLimit

    % calculate truth quantities - need to replace this static model
    % with a dynamic model or test data
    time(index) = dt*index;
    % synthesise IMU measurements
    angRate = 0.01*[randn;randn;randn];
    accel   = 0.05*[randn;randn;randn] + transpose(TsnTruth)*[0;0;-9.81];
    % synthesise velocity measurements
    measVel = [0;0;0];
    % synthesise gimbal angles
    gPhi = 0;
    gTheta = 0;
    gPsi = 20*pi/180;
    % calculate rotation from magnetometer to sensor 
    % Define rotation from magnetometer to yaw gimbal
    T3 = [ cos(gPsi)  sin(gPsi)   0; ...
            -sin(gPsi)  cos(gPsi)   0; ...
             0          0           1];
    % Define rotation from yaw gimbal to roll gimbal
    T1 = [ 1          0           0; ...
             0          cos(gPhi)   sin(gPhi); ...
             0         -sin(gPhi)   cos(gPhi)];
    % Define rotation from roll gimbal to pitch gimbal
    T2 = [ cos(gTheta)    0      -sin(gTheta); ...
             0              1       0; ...
             sin(gTheta)    0       cos(gTheta)];
    % Define rotation from magnetometer to sensor using a 312 rotation sequence
    TmsTruth = T2*T1*T3;
    % calculate rotation from NED to magnetometer axes Tnm = Tsm * Tns
    TnmTruth = transpose(TmsTruth) * transpose(TsnTruth);
    % synthesise magnetometer measurements adding sensor bias
    magBody = TnmTruth*magEarthTruth + magMeasBias;

    % predict states
    [quat, states, Tsn, delAng, delVel]  = PredictStates(quat,states,angRate,accel,dt);
    statesLog(:,index) = states;
    quatLog(:,index) = quat;
    eulLog(:,index) = QuatToEul(quat);
    eulErrLog(:,index) = eulLog(:,index) - QuatToEul(quatTruth);

    % predict covariance matrix
    covariance  = PredictCovariance(delAng,delVel,quat,states,covariance,dt);
    % fuse velocity measurements
    [quat,states,angErr,covariance,velInnov,velInnovVar] = FuseVelocity(quat,states,covariance,measVel);
    velInnovLog(:,index) = velInnov;
    velInnovVarLog(:,index) = velInnovVar;

    % Align the heading once there has been enough time for the filter to
    % settle and the error estimate has dropped below the threshold
    if (index > 500 && headingAligned==0 && angErr < 1e-4)
        % calculate the initial heading using magnetometer, gimbal,
        % estimated tilt and declination
        quat = AlignHeading(gPhi,gPsi,gTheta,Tsn,magBody,quat,decInit);
        headingAligned = 1;
    end
    
    % fuse magnetometer measurements
    if (headingAligned == 1)
        [quat,states,covariance,decInnov,decInnovVar] = FuseMagnetometer(quat,states,covariance,magBody,gPhi,gPsi,gTheta);
        decInnovLog(:,index) = decInnov;
        decInnovVarLog(:,index) = decInnovVar;
    end
    
end
%% Generate Plots
close all;
PlotData;