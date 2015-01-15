%% plot gyro bias estimates
figure;
plot(EKFlogs.time,EKFlogs.states(7:9,:)/dtSlow*180/pi);
grid on;
ylabel('Gyro Bias Estimate (deg/sec)');
xlabel('time (sec)');
hold on;
plot([min(time),max(time)],[gyroBias,gyroBias]*180/pi);
hold off;

%% plot velocity
figure;
plot(EKFlogs.time,EKFlogs.states(4:6,:));
grid on;
ylabel('Velocity (m/sec)');
xlabel('time (sec)');

%% calculate and plot tilt correction magnitude
figure;
plot(EKFlogs.time,EKFlogs.tiltCorr);
grid on;
ylabel('Tilt Correction Magnitude (rad)');
xlabel('time (sec)');

%% plot Euler angle estimates
figure;
plot(EKFlogs.time,EKFlogs.euler*180/pi);
grid on;
ylabel('EKF Euler Angle Estimates (deg)');
xlabel('time (sec)');

%% plot Euler angle error estimates
figure;
subplot(3,1,1);plot(EKFlogs.time,EKFlogs.eulErr(1,:)*180/pi);
ylabel('Roll Error (deg)');grid on;
subplot(3,1,2);plot(EKFlogs.time,EKFlogs.eulErr(2,:)*180/pi);
ylabel('Pitch Error (deg)');grid on;
subplot(3,1,3);plot(EKFlogs.time,EKFlogs.eulErr(3,:)*180/pi);
ylabel('Yaw Error (deg)');
grid on;
xlabel('time (sec)');

%% plot high rate Euler angles
figure;
plot(timeFast,eulLogFast*180/pi);
grid on;
ylabel('Gimbal Euler Angle Estimates (deg)');
xlabel('time (sec)');