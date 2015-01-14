%% plot gyro bias estimates
figure;
plot(time,statesLog(7:9,:)/dt*180/pi);
grid on;
ylabel('Gyro Bias Estimate (deg/sec)');
xlabel('time (sec)');

%% plot velocity
figure;
plot(time,statesLog(4:6,:));
grid on;
ylabel('Velocity (m/sec)');
xlabel('time (sec)');

%% calculate and plot tilt correction magnitude
figure;
angErrLog = sqrt(statesLog(1,:).^2 + statesLog(2,:).^2 + statesLog(3,:).^2)*180/pi;
plot(time,angErrLog);
grid on;
ylabel('Tilt correction magnitude (deg)');
xlabel('time (sec)');

%% plot Euler angle estimates
figure;
plot(time,eulLog*180/pi);
grid on;
ylabel('Euler Angle Estimates (deg)');
xlabel('time (sec)');

%% plot Euler angle error estimates
figure;
plot(time,eulErrLog*180/pi);
grid on;
ylabel('Euler Angle Errors (deg)');
xlabel('time (sec)');