%% plot gyro bias estimates
figure;
plot(time,statesLog(7:9,:)/dt*180/pi);
grid on;
ylabel('Gyro Bias Estimate (deg/sec)');
xlabel('time (sec)');
hold on;
plot([min(time),max(time)],[gyroBias,gyroBias]*180/pi);
hold off;

%% plot velocity
figure;
plot(time,statesLog(4:6,:));
grid on;
ylabel('Velocity (m/sec)');
xlabel('time (sec)');

%% calculate and plot tilt correction magnitude
figure;
plot(time,tiltCorrLog);
grid on;
ylabel('Tilt Correction Magnitude (rad)');
xlabel('time (sec)');

%% plot Euler angle estimates
figure;
plot(time,eulLog*180/pi);
grid on;
ylabel('Euler Angle Estimates (deg)');
xlabel('time (sec)');

%% plot Euler angle error estimates
figure;
subplot(3,1,1);plot(time,eulErrLog(1,:)*180/pi);
ylabel('Roll Error (deg)');grid on;
subplot(3,1,2);plot(time,eulErrLog(2,:)*180/pi);
ylabel('Pitch Error (deg)');grid on;
subplot(3,1,3);plot(time,eulErrLog(3,:)*180/pi);
ylabel('Yaw Error (deg)');
grid on;
xlabel('time (sec)');