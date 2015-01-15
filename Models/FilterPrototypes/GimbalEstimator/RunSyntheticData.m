%% Set initial conditions
clear all;
dt = 1/50;
duration = 60;
indexLimit = round(duration/dt);
time = zeros(1,indexLimit);
statesLog = zeros(9,indexLimit);
quatLog   = zeros(4,indexLimit);
eulLog = zeros(3,indexLimit);
eulErrLog = eulLog;
tiltCorrLog = zeros(1,indexLimit);
velInnovLog = zeros(3,indexLimit);
decInnovLog = zeros(1,indexLimit);
velInnovVarLog = velInnovLog;
decInnovVarLog = decInnovLog;
% Use a random initial truth orientation
phiInit = 0.1*randn;
thetaInit = 0.1*randn;
psiInit = 2*pi*rand - pi;
quatTruth = EulToQuat([phiInit,thetaInit,psiInit]);% [1;0.05*randn;0.05*randn;2*(rand-0.5)];
quatLength = sqrt(quatTruth(1)^2 + quatTruth(2)^2 + quatTruth(3)^2 + quatTruth(4)^2);
quatTruth = quatTruth / quatLength;
TsnTruth = Quat2Tbn(quatTruth);

% initialise the filter to level (let it find its own attitude)
quat = [1;0;0;0];
states = zeros(9,1);
Tsn = Quat2Tbn(quat);

% define the earths truth magnetic field
declTruth = 10*pi/180;
magEarthTruth = [0.25*cos(declTruth);0.25*sin(declTruth);-0.5];

% define the declination parameter assuming 2deg RMS error - this would be
% obtained from the main EKF to take advantage of in-flight learning
declParam = declTruth + 2*pi/180*randn;

% define the magnetometer bias errors
magMeasBias = 0.02*[randn;randn;randn];

% Define IMU bias errors and noise
gyroBias = 1*pi/180*[randn;randn;randn];
accBias = 0.05*[randn;randn;randn];
gyroNoise = 0.01;
accNoise = 0.05;

% define the state covariances with the exception of the quaternion covariances
Sigma_velNED = 0.5; % 1 sigma uncertainty in horizontal velocity components
Sigma_dAngBias  = 1*pi/180*dt; % 1 Sigma uncertainty in delta angle bias
Sigma_angErr = 1; % 1 Sigma uncertainty in angular misalignment (rad)
covariance   = single(diag([Sigma_angErr*[1;1;1];Sigma_velNED*[1;1;1];Sigma_dAngBias*[1;1;1]]).^2);

% Initialise truth trajectory variables
% fly a CCW circle with constant gimbal angles
gPsiInit = 20*pi/180; % gimbal yaw
gThetaInit = 0; % gimbal pitch
gPhiInit = 0; % gimbal roll
psiTruth = psiInit;
radius = 20;
gndSpd = 5;
trackAngTruth = -pi;
centripAccelMag = gndSpd/radius*gndSpd;
gravAccel = [0;0;-9.81];

%% Main Loop
headingAligned=0;
for index = 1:indexLimit
    %% Calculate Truth Data
    % Need to replace this with a full kinematic model or test data
    time(index) = dt*index;
    % calculate truth angular rates - we don't start maneouvring until 
    % heading alignment is complete
    psiRateTruth = gndSpd/radius*headingAligned;
    angRateTruth = [0;0;psiRateTruth]; % constant yaw rate
    % calculate yaw and track angles
    psiTruth = psiTruth + psiRateTruth*dt;
    trackAngTruth = trackAngTruth + psiRateTruth*dt;
    % Cacluate truth quternion
    quatTruth = EulToQuat([phiInit,thetaInit,psiTruth]);
    % Calculate truth rotaton from sensor to NED
    TsnTruth = Quat2Tbn(quatTruth);
    % calculate truth accel vector
    centripAccel = centripAccelMag*[-sin(trackAngTruth);cos(trackAngTruth);0];
    accelTruth = transpose(TsnTruth)*(gravAccel + centripAccel);
    % calculate truth velocity vector
    truthVel = gndSpd*[cos(trackAngTruth);sin(trackAngTruth);0];
    
    %% synthesise sensor measurements
    % Synthesise IMU measurements, adding bias and noise
    angRateMeas = angRateTruth + gyroBias + gyroNoise*[randn;randn;randn];
    accelMeas   = accelTruth + accBias + accNoise*[randn;randn;randn];
    % synthesise velocity measurements
    measVel = truthVel;
    % synthesise gimbal angles
    gPhi = 0;
    gTheta = 0;
    gPsi = gPsiInit;
    % Define rotation from magnetometer to sensor using a 312 rotation sequence
    TmsTruth = calcTms(gPhi,gPsi,gTheta);
    % calculate rotation from NED to magnetometer axes Tnm = Tsm * Tns
    TnmTruth = transpose(TmsTruth) * transpose(TsnTruth);
    % synthesise magnetometer measurements adding sensor bias
    magMeas = TnmTruth*magEarthTruth + magMeasBias;

    %% Run Filter
    % predict states
    [quat, states, Tsn, delAng, delVel]  = PredictStates(quat,states,angRateMeas,accelMeas,dt);
    
    % log state prediciton data
    statesLog(:,index) = states;
    quatLog(:,index) = quat;
    eulLog(:,index) = QuatToEul(quat);
    if (headingAligned)
        eulErrLog(:,index) = eulLog(:,index) - QuatToEul(quatTruth);
        if (eulErrLog(3,index) > pi)
            eulErrLog(3,index) = eulErrLog(3,index) - 2*pi;
        elseif (eulErrLog(3,index) < -pi)
            eulErrLog(3,index) = eulErrLog(3,index) + 2*pi;
        end
    else
        eulErrLog(:,index) = [NaN;NaN;NaN];
    end

    % predict covariance matrix
    covariance = PredictCovarianceOptimised(delAng,delVel,quat,states,covariance,dt);
    
    % fuse velocity measurements
    [quat,states,tiltCorrection,covariance,velInnov,velInnovVar] = FuseVelocity(quat,states,covariance,measVel);
    
    % log velocity fusion data
    velInnovLog(:,index) = velInnov;
    velInnovVarLog(:,index) = velInnovVar;
    tiltCorrLog(1,index) = tiltCorrection;

    % Align the heading once there has been enough time for the filter to
    % settle and the tilt corrections have dropped below a threshold
    if (((time(index) > 5.0 && tiltCorrection < 1e-4) || (time(index) > 30.0)) && headingAligned==0)
        % calculate the initial heading using magnetometer, gimbal,
        % estimated tilt and declination
        quat = AlignHeading(gPhi,gPsi,gTheta,Tsn,magMeas,quat,declParam);
        headingAligned = 1;
    end
    
    % fuse magnetometer measurements and log fusion data
    if (headingAligned == 1)
        [quat,states,covariance,decInnov,decInnovVar] = FuseMagnetometer(quat,states,covariance,magMeas,declParam,gPhi,gPsi,gTheta);
        decInnovLog(:,index) = decInnov;
        decInnovVarLog(:,index) = decInnovVar;
    end
    %%
end

%% Generate Plots
close all;
PlotData;