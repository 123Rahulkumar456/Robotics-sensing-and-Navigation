% Loading the Five Minute IMU data
load('fiveMinImu.mat')

% Extracting the desired data
Data1 = fiveminimudata(:,3);
Data2 = fiveminimudata(:,10:13);
Data3 = fiveminimudata(:,15:17);
Data4 = fiveminimudata(:,19:21);
Data5 = fiveminimudata(:,27:29);

% Converting the table to array for analysis
time = table2array(Data1);
quaternion = table2array(Data2);
angVel = table2array(Data3);
linearAccln = table2array(Data4);
magneticField= table2array(Data5);

% Converting the quaternion to euler angle
quat_wxyz = [quaternion(:,4) quaternion(:,1:3)];
eulerAngle = quat2eul(quat_wxyz) * (180/pi);

% Finding the mean and standard deviation of data on each axis
eulerAngle_mean = mean(eulerAngle);
eulerAngle_std = std(eulerAngle);
angVel_mean = mean(angVel);
angVel_std = std(angVel);
linearAccln_mean = mean(linearAccln);
linearAccln_std = std(linearAccln);
magneticField_mean = mean(magneticField);
magneticField_std = std(magneticField);

% Printing the mean of data on each axis
fprintf("\n Mean of Euler Angles (Yaw,Pitch,Roll):\n")
disp(eulerAngle_mean)
fprintf("\n Mean of Angular Velocity (X,Y,Z):\n")
disp(angVel_mean)
fprintf("\n Mean of Linear Acceleration (X,Y,Z):\n")
disp(linearAccln_mean)
fprintf("\n Mean of Magnetic Field (X,Y,Z):\n")
disp(magneticField_mean)

% Printing the standard deviation of data on each axis
fprintf("\n Standard deviation of Euler Angles (Yaw,Pitch,Roll):\n")
disp(eulerAngle_std)
fprintf("\n Standard deviation of Angular Velocity (X,Y,Z):\n")
disp(angVel_std)
fprintf("\n Standard deviation of Linear Acceleration (X,Y,Z):\n")
disp(linearAccln_std)
fprintf("\n Standard deviation of Magnetic Field (X,Y,Z):\n")
disp(magneticField_std)

% Plotting timeseries of each axis
figure(1)
ts1 = timeseries(angVel(:,1));
ts1.Name = "Angular Velocity X";
%plot(ts1,'b')
hold on;
plot([1:size(angVel,1)],angVel(:,1),'r')
plot([1:size(angVel,1)],angVel_mean(1,1)*ones(size(angVel,1),1))
plot([1:size(angVel,1)],(angVel_mean(1,1)+angVel_std(1,1))*ones(size(angVel,1),1),'k')
plot([1:size(angVel,1)],(angVel_mean(1,1)-angVel_std(1,1))*ones(size(angVel,1),1),'k')
title("Time Series Plot:Angular Velocity X")
xlabel("Time (sec) * (1/40)")
ylabel("Angular Velocity (rad/s)")
legend("Angular Velocity","Mean","+- Standard deviation")
hold off

figure(2)
ts2 = timeseries(angVel(:,2));
ts2.Name = "Angular Velocity Y";
%plot(ts2,'r')
hold on;
plot([1:size(angVel,1)],angVel(:,2),'g')
plot([1:size(angVel,1)],angVel_mean(1,2)*ones(size(angVel,1),1))
plot([1:size(angVel,1)],(angVel_mean(1,2)+angVel_std(1,2))*ones(size(angVel,1),1),'k')
plot([1:size(angVel,1)],(angVel_mean(1,2)-angVel_std(1,2))*ones(size(angVel,1),1),'k')
title("Time Series Plot:Angular Velocity Y")
xlabel("Time (sec) * (1/40)")
ylabel("Angular Velocity (rad/s)")
legend("Angular Velocity","Mean","+- Standard deviation")
hold off

figure(3)
ts3 = timeseries(angVel(:,3));
ts3.Name = "Angular Velocity Z";
%plot(ts3,'g')
hold on;
plot([1:size(angVel,1)],angVel(:,3),'b')
plot([1:size(angVel,1)],angVel_mean(1,3)*ones(size(angVel,1),1))
plot([1:size(angVel,1)],(angVel_mean(1,3)+angVel_std(1,3))*ones(size(angVel,1),1),'k')
plot([1:size(angVel,1)],(angVel_mean(1,3)-angVel_std(1,3))*ones(size(angVel,1),1),'k')
title("Time Series Plot:Angular Velocity Z")
xlabel("Time (sec) * (1/40)")
ylabel("Angular Velocity (rad/s)")
legend("Angular Velocity","Mean","+- Standard deviation")
hold off

figure(4)
ts4 = timeseries(linearAccln(:,1));
ts4.Name = "Linear Acceleration X";
%plot(ts4,'b')
hold on;
plot([1:size(linearAccln,1)],linearAccln(:,1),'r')
plot([1:size(linearAccln,1)],linearAccln_mean(1,1)*ones(size(linearAccln,1),1))
plot([1:size(linearAccln,1)],(linearAccln_mean(1,1)+linearAccln_std(1,1))*ones(size(linearAccln,1),1),'k')
plot([1:size(linearAccln,1)],(linearAccln_mean(1,1)-linearAccln_std(1,1))*ones(size(linearAccln,1),1),'k')
title("Time Series Plot:Linear Acceleration X")
xlabel("Time (sec) * (1/40)")
ylabel("Linear Acceleration (m/s^2)")
legend("Linear Acceleration","Mean","+- Standard deviation")
hold off

figure(5)
ts5 = timeseries(linearAccln(:,2));
ts5.Name = "Linear Acceleration Y";
%plot(ts5,'r')
hold on;
plot([1:size(linearAccln,1)],linearAccln(:,2),'g')
plot([1:size(linearAccln,1)],linearAccln_mean(1,2)*ones(size(linearAccln,1),1))
plot([1:size(linearAccln,1)],(linearAccln_mean(1,2)+linearAccln_std(1,2))*ones(size(linearAccln,1),1),'k')
plot([1:size(linearAccln,1)],(linearAccln_mean(1,2)-linearAccln_std(1,2))*ones(size(linearAccln,1),1),'k')
title("Time Series Plot:Linear Acceleration Y")
xlabel("Time (sec) * (1/40)")
ylabel("Linear Acceleration (m/s^2)")
legend("Linear Acceleration","Mean","+- Standard deviation")
hold off

figure(6)
ts6 = timeseries(linearAccln(:,3));
ts6.Name = "Linear Acceleration Z";
%plot(ts6,'g')
hold on;
plot([1:size(linearAccln,1)],linearAccln(:,3),'b')
plot([1:size(linearAccln,1)],linearAccln_mean(1,3)*ones(size(linearAccln,1),1))
plot([1:size(linearAccln,1)],(linearAccln_mean(1,3)+linearAccln_std(1,3))*ones(size(linearAccln,1),1),'k')
plot([1:size(linearAccln,1)],(linearAccln_mean(1,3)-linearAccln_std(1,3))*ones(size(linearAccln,1),1),'k')
title("Time Series Plot:Linear Acceleration Z")
xlabel("Time (sec) * (1/40)")
ylabel("Linear Acceleration (m/s^2)")
legend("Linear Acceleration","Mean","+- Standard deviation")
hold off

figure(7)
ts7 = timeseries(magneticField(:,1));
ts7.Name = "Magnetic Field X";
%plot(ts7,'b')
hold on;
plot([1:size(magneticField,1)],magneticField(:,1),'r')
plot([1:size(magneticField,1)],magneticField_mean(1,1)*ones(size(magneticField,1),1))
plot([1:size(magneticField,1)],(magneticField_mean(1,1)+magneticField_std(1,1))*ones(size(magneticField,1),1),'k')
plot([1:size(magneticField,1)],(magneticField_mean(1,1)-magneticField_std(1,1))*ones(size(magneticField,1),1),'k')
title("Time Series Plot:Magnetic Field X")
xlabel("Time (sec) * (1/40)")
ylabel("Magnetic Field (gauss)")
legend("Magnetic Field","Mean","+- Standard deviation")
hold off

figure(8)
ts8 = timeseries(magneticField(:,2));
ts8.Name = "Magnetic Field Y";
% plot(ts8,'r')
hold on;
plot([1:size(magneticField,1)],magneticField(:,2),'g')
plot([1:size(magneticField,1)],magneticField_mean(1,2)*ones(size(magneticField,1),1))
plot([1:size(magneticField,1)],(magneticField_mean(1,2)+magneticField_std(1,2))*ones(size(magneticField,1),1),'k')
plot([1:size(magneticField,1)],(magneticField_mean(1,2)-magneticField_std(1,2))*ones(size(magneticField,1),1),'k')
title("Time Series Plot:Magnetic Field Y")
xlabel("Time (sec) * (1/40)")
ylabel("Magnetic Field (gauss)")
legend("Magnetic Field","Mean","+- Standard deviation")
hold off

figure(9)
ts9 = timeseries(magneticField(:,3));
ts9.Name = "Magnetic Field Z";
% plot(ts9,'g')
hold on;
plot([1:size(magneticField,1)],magneticField(:,3),'b')
plot([1:size(magneticField,1)],magneticField_mean(1,3)*ones(size(magneticField,1),1))
plot([1:size(magneticField,1)],(magneticField_mean(1,3)+magneticField_std(1,3))*ones(size(magneticField,1),1),'k')
plot([1:size(magneticField,1)],(magneticField_mean(1,3)-magneticField_std(1,3))*ones(size(magneticField,1),1),'k')
title("Time Series Plot:Magnetic Field Z")
xlabel("Time (sec) * (1/40)")
ylabel("Magnetic Field (gauss)")
legend("Magnetic Field","Mean","+- Standard deviation")
hold off

figure(10)
ts10 = timeseries(eulerAngle(:,1));
ts10.Name = "Yaw";
% plot(ts10,'b')
hold on;
plot([1:size(eulerAngle,1)],eulerAngle(:,1),'r')
plot([1:size(eulerAngle,1)],eulerAngle_mean(1,1)*ones(size(eulerAngle,1),1))
plot([1:size(eulerAngle,1)],(eulerAngle_mean(1,1)+eulerAngle_std(1,1))*ones(size(eulerAngle,1),1),'k')
plot([1:size(eulerAngle,1)],(eulerAngle_mean(1,1)-eulerAngle_std(1,1))*ones(size(eulerAngle,1),1),'k')
title("Time Series Plot:Yaw")
xlabel("Time (sec) * (1/40)")
ylabel("Yaw (degrees)")
legend("Yaw","Mean","+- Standard deviation")
hold off

figure(11)
ts11 = timeseries(eulerAngle(:,2));
ts11.Name = "Pitch";
%plot(ts11,'r')
hold on;
plot([1:size(eulerAngle,1)],eulerAngle(:,2),'g')
plot([1:size(eulerAngle,1)],eulerAngle_mean(1,2)*ones(size(eulerAngle,1),1))
plot([1:size(eulerAngle,1)],(eulerAngle_mean(1,2)+eulerAngle_std(1,2))*ones(size(eulerAngle,1),1),'k')
plot([1:size(eulerAngle,1)],(eulerAngle_mean(1,2)-eulerAngle_std(1,2))*ones(size(eulerAngle,1),1),'k')
title("Time Series Plot:Pitch")
xlabel("Time (sec) * (1/40)")
ylabel("Pitch (degrees)")
legend("Pitch","Mean","+- Standard deviation")
hold off

figure(12)
ts12 = timeseries(eulerAngle(:,3));
ts12.Name = "Roll";
%plot(ts12,'g')
hold on;
plot([1:size(eulerAngle,1)],eulerAngle(:,3),'b')
plot([1:size(eulerAngle,1)],eulerAngle_mean(1,3)*ones(size(eulerAngle,1),1))
plot([1:size(eulerAngle,1)],(eulerAngle_mean(1,3)+eulerAngle_std(1,3))*ones(size(eulerAngle,1),1),'k')
plot([1:size(eulerAngle,1)],(eulerAngle_mean(1,3)-eulerAngle_std(1,3))*ones(size(eulerAngle,1),1),'k')
title("Time Series Plot:Roll")
xlabel("Time (sec) * (1/40)")
ylabel("Roll (degrees)")
legend("Roll","Mean","+- Standard deviation")
hold off


% Plotting the histogram of each axis
figure(13)
histogram(angVel(:,1))
title("Histogram Plot:Angular Velocity X")
xlabel("Angular Velocity (rad/s)")
ylabel("Frequency")

figure(14)
histogram(angVel(:,2))
title("Histogram Plot:Angular Velocity Y")
xlabel("Angular Velocity (rad/s)")
ylabel("Frequency")

figure(15)
histogram(angVel(:,3))
title("Histogram Plot:Angular Velocity Z")
xlabel("Angular Velocity (rad/s)")
ylabel("Frequency")

figure(16)
histogram(linearAccln(:,1))
title("Histogram Plot:Linear Acceleration X")
xlabel("Linear Acceleration (m/s^2)")
ylabel("Frequency")

figure(17)
histogram(linearAccln(:,2))
title("Histogram Plot:Linear Acceleration Y")
xlabel("Linear Acceleration (m/s^2)")
ylabel("Frequency")

figure(18)
histogram(linearAccln(:,3))
title("Histogram Plot:Linear Acceleration Z")
xlabel("Linear Acceleration (m/s^2)")
ylabel("Frequency")

figure(19)
histogram(magneticField(:,1))
title("Histogram Plot:Magnetic Field X")
xlabel("Magnetic Field (gauss)")
ylabel("Frequency")

figure(20)
histogram(magneticField(:,2))
title("Histogram Plot:Magnetic Field Y")
xlabel("Magnetic Field (gauss)")
ylabel("Frequency")

figure(21)
histogram(magneticField(:,3))
title("Histogram Plot:Magnetic Field Z")
xlabel("Magnetic Field (gauss)")
ylabel("Frequency")

figure(22)
histogram(eulerAngle(:,1))
title("Histogram Plot:Yaw")
xlabel("Yaw (degrees)")
ylabel("Frequency")

figure(23)
histogram(eulerAngle(:,2))
title("Histogram Plot:Pitch")
xlabel("Pitch (degrees)")
ylabel("Frequency")

figure(24)
histogram(eulerAngle(:,3))
title("Histogram Plot:Roll")
xlabel("Roll (degrees)")
ylabel("Frequency")