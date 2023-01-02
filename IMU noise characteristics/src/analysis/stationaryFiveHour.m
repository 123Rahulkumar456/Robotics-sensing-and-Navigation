% Loading the Five Hour IMU data
load('fiveHourImu.mat')

% Extracting the desired data
Data1 = fivehourimudataimu(:,3);
Data2 = fivehourimudataimu(:,10:13);
Data3 = fivehourimudataimu(:,15:17);
Data4 = fivehourimudataimu(:,19:21);
Data5 = fivehourimudataimu(:,27:29);

% Converting the table to array for analysis
time = table2array(Data1);
quaternion = table2array(Data2);
angVel = table2array(Data3);
linearAccln = table2array(Data4);
magneticField= table2array(Data5);
quat_wxyz = [quaternion(:,4) quaternion(:,1:3)];
eulerAngle = quat2eul(quat_wxyz);

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
fprintf("\n Mean of Angular Velocity (X,Y,Z):\n")
disp(angVel_mean)
fprintf("\n Mean of Linear Acceleration (X,Y,Z):\n")
disp(linearAccln_mean)


% Printing the standard deviation of data on each axis
fprintf("\n Standard deviation of Angular Velocity (X,Y,Z):\n")
disp(angVel_std)
fprintf("\n Standard deviation of Linear Acceleration (X,Y,Z):\n")
disp(linearAccln_std)

% Plotting Allan deviation plot for Gyro Angular Rate X
Fs = 40;
t0 = 1/Fs;
theta = cumsum(angVel(:,1),1)*t0;
maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2)); 
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m);
m = unique(m);

[avar, tau] = allanvar(angVel(:,1), m, Fs);
adev = sqrt(avar);

% Angle Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN;
tauN = 1;
lineN = N ./ sqrt(tau);

% Rate Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK;
tauK = 3;
lineK = K .* sqrt(tau/3);

% Bias Instability
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB;
tauB = tau(i);
lineB = B * scfB * ones(size(tau));

% Now that all the noise parameters have been calculated, plot the Allan
% deviation with all of the lines used for quantifying the parameters.
tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')
title('Allan Deviation with Noise Parameters for Angular Velocity X')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma (rad/s)$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
    '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal

N = N*(180/pi);
B = B*(180/pi)*3600;
K = K*(180/pi)*3600*60;

fprintf("\n\nPrinting the parameters for Gyro Angular Rate X:\n")
fprintf("Angle Random Walk:%f(deg/s^(1/2))",N)
fprintf("\nBias Instability:%f(deg/hr)",B)
fprintf("\nRate Random Walk:%f(deg/hr/hr^(1/2))\n",K)

% Plotting Allan deviation plot for Gyro Angular Rate Y
Fs = 40;
t0 = 1/Fs;
theta = cumsum(angVel(:,2),1)*t0;
maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2)); 
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m);
m = unique(m);

[avar, tau] = allanvar(angVel(:,2), m, Fs);
adev = sqrt(avar);

% Angle Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN;
tauN = 1;
lineN = N ./ sqrt(tau);

% Rate Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK;
tauK = 3;
lineK = K .* sqrt(tau/3);

% Bias Instability
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB;
tauB = tau(i);
lineB = B * scfB * ones(size(tau));

% Now that all the noise parameters have been calculated, plot the Allan
% deviation with all of the lines used for quantifying the parameters.
tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')
title('Allan Deviation with Noise Parameters for Angular Velocity Y')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma (rad/s)$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
    '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal

N = N*(180/pi);
B = B*(180/pi)*3600;
K = K*(180/pi)*3600*60;


fprintf("\n\nPrinting the parameters for Gyro Angular Rate Y:\n")
fprintf("Angle Random Walk:%f(deg/s^(1/2))",N)
fprintf("\nBias Instability:%f(deg/hr)",B)
fprintf("\nRate Random Walk:%f(deg/hr/hr^(1/2))\n",K)


% Plotting Allan deviation plot for Gyro Angular Rate Z
Fs = 40;
t0 = 1/Fs;
theta = cumsum(angVel(:,3),1)*t0;
maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2)); 
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m);
m = unique(m);

[avar, tau] = allanvar(angVel(:,3), m, Fs);
adev = sqrt(avar);

% Angle Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN;
tauN = 1;
lineN = N ./ sqrt(tau);

% Rate Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK;
tauK = 3;
lineK = K .* sqrt(tau/3);

% Bias Instability
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB;
tauB = tau(i);
lineB = B * scfB * ones(size(tau));

% Now that all the noise parameters have been calculated, plot the Allan
% deviation with all of the lines used for quantifying the parameters.
tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')
title('Allan Deviation with Noise Parameters for Angular Velocity Z')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma (rad/s)$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
    '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal

N = N*(180/pi);
B = B*(180/pi)*3600;
K = K*(180/pi)*3600*60;

fprintf("\n\nPrinting the parameters for Gyro Angular Rate Z:\n")
fprintf("Angle Random Walk:%f(deg/s^(1/2))",N)
fprintf("\nBias Instability:%f(deg/hr)",B)
fprintf("\nRate Random Walk:%f(deg/hr/hr^(1/2))\n",K)

% Plotting Allan deviation plot for Linear Acceleration X
Fs = 40;
t0 = 1/Fs;
theta = cumsum(linearAccln(:,1),1)*t0;
maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2)); 
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m);
m = unique(m);

[avar, tau] = allanvar(linearAccln(:,1), m, Fs);
adev = sqrt(avar);

% Velocity Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN;
tauN = 1;
lineN = N ./ sqrt(tau);

% Rate Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK;
tauK = 3;
lineK = K .* sqrt(tau/3);

% Bias Instability
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB;
tauB = tau(i);
lineB = B * scfB * ones(size(tau));

% Now that all the noise parameters have been calculated, plot the Allan
% deviation with all of the lines used for quantifying the parameters.
tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')
title('Allan Deviation with Noise Parameters for Linear Acceleration X')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma (m/s^2)$', '$\sigma_N ((m/s^2)/\sqrt{Hz})$', ...
    '$\sigma_K ((m/s^2)\sqrt{Hz})$', '$\sigma_B (m/s^2)$', 'Interpreter', 'latex')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal

B = B*1000/9.80665;
K = K*60;

fprintf("\n\nPrinting the parameters for Linear acceleration X:\n")
fprintf("Velocity Random Walk:%f(m/s/s^(1/2))",N)
fprintf("\nBias Instability:%f(mg)",B)
fprintf("\nRate Random Walk:%f(m/s^2/hr^(1/2))\n",K)

% Plotting Allan deviation plot for Linear Acceleration Y
Fs = 40;
t0 = 1/Fs;
theta = cumsum(linearAccln(:,2),1)*t0;
maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2)); 
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m);
m = unique(m);

[avar, tau] = allanvar(linearAccln(:,2), m, Fs);
adev = sqrt(avar);

% Velocity Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN;
tauN = 1;
lineN = N ./ sqrt(tau);

% Rate Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK;
tauK = 3;
lineK = K .* sqrt(tau/3);

% Bias Instability
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB;
tauB = tau(i);
lineB = B * scfB * ones(size(tau));

% Now that all the noise parameters have been calculated, plot the Allan
% deviation with all of the lines used for quantifying the parameters.
tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')
title('Allan Deviation with Noise Parameters for Linear Acceleration Y')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma (m/s^2)$', '$\sigma_N ((m/s^2)/\sqrt{Hz})$', ...
    '$\sigma_K ((m/s^2)\sqrt{Hz})$', '$\sigma_B (m/s^2)$', 'Interpreter', 'latex')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal

B = B*1000/9.80665;
K = K*60;

fprintf("\n\nPrinting the parameters for Linear acceleration Y:\n")
fprintf("Velocity Random Walk:%f(m/s/s^(1/2))",N)
fprintf("\nBias Instability:%f(mg)",B)
fprintf("\nRate Random Walk:%f(m/s^2/hr^(1/2))\n",K)

% Plotting Allan deviation plot for Linear Acceleration Z
Fs = 40;
t0 = 1/Fs;
theta = cumsum(linearAccln(:,3),1)*t0;
maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2)); 
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m);
m = unique(m);

[avar, tau] = allanvar(linearAccln(:,3), m, Fs);
adev = sqrt(avar);

% Velocity Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN;
tauN = 1;
lineN = N ./ sqrt(tau);

% Rate Random Walk
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK;
tauK = 3;
lineK = K .* sqrt(tau/3);

% Bias Instability
% Find the index where the slope of the log-scaled Allan deviation is equal to the slope specified.
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB;
tauB = tau(i);
lineB = B * scfB * ones(size(tau));

% Now that all the noise parameters have been calculated, plot the Allan
% deviation with all of the lines used for quantifying the parameters.
tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')
title('Allan Deviation with Noise Parameters for Linear Acceleration Z')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma (m/s^2)$', '$\sigma_N ((m/s^2)/\sqrt{Hz})$', ...
    '$\sigma_K ((m/s^2)\sqrt{Hz})$', '$\sigma_B (m/s^2)$', 'Interpreter', 'latex')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal

B = B*1000/9.80665;
K = K*60;

fprintf("\n\nPrinting the parameters for Linear acceleration Z:\n")
fprintf("Velocity Random Walk:%f(m/s/s^(1/2))",N)
fprintf("\nBias Instability:%f(mg)",B)
fprintf("\nRate Random Walk:%f(m/s^2/hr^(1/2))\n",K)