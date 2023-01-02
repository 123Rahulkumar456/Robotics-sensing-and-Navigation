load('stationary.mat')

Data1 = stationarydatagps(:,6:10);
Data2 = stationarydatagps(:,3);

Data_arr = table2array(Data1);
time = table2array(Data2);

lat = Data_arr(:,1);
lon = Data_arr(:,2);
Alt = Data_arr(:,3);
utm_x = Data_arr(:,4);
utm_y = Data_arr(:,5);

true_lat = 42.337150;
true_lon = -71.090521;
true_x = 327781;
true_y = 4689327;

Error_x = utm_x - (true_x*ones(size(utm_x)));
Error_y = utm_y - (true_y*ones(size(utm_y)));
Error_Pos = (Error_x.^2+Error_y.^2).^0.5;

x_offset = mean(utm_x);
y_offset = mean(utm_y);
z_offset = mean(Alt);
t_offset = min(time);

X = utm_x - (x_offset*ones(900,1));
Y = utm_y - (y_offset*ones(900,1));
Z = Alt - (z_offset*ones(900,1));
T = time - (t_offset*ones(900,1));
Actual_x = true_x - x_offset;
Actual_y = true_y - y_offset;

std_x = std(utm_x);
std_y = std(utm_y);
std_z = std(Alt);

X_error = median(utm_x) - true_x;
Y_error = median(utm_y) - true_y;

fprintf('%d m is error in X direction wrt Median\n', X_error );
fprintf('%d m is error in Y direction wrt Median\n', Y_error );


figure(1)
plot(X,Y,'r.')
xlabel('X position(m)  (Offset by 327779m)')
ylabel('Y postion(m) (Offset by 4689326m)')
title('East(X) vs North(Y)')

figure(2)
plot(T,X,'c.')
hold on;
axis([0,900,-8,6])
plot(T, std_x*ones(size(T)))
plot(T, -std_x*ones(size(T)))
hold off;
xlabel('Time(s)  (Offset by 54247s)')
ylabel('X position(m)  (Offset by 327779m)')
title('Time vs X Pos')
legend('UTM X Position', '+- Std deviation')

figure(3)
plot(T,Y,'b.')
hold on;
plot(T, std_y*ones(size(T)))
plot(T, -std_y*ones(size(T)))
hold off;
xlabel('Time(s)  (Offset by 54247s)')
ylabel('Y postion(m) (Offset by 4689326m)')
title('Time vs Y Pos')
legend('UTM Y Position', '+- Std deviation')

figure(4)
plot(T,Z,'g.')
hold on;
plot(T, std_z*ones(size(T)))
plot(T, -std_z*ones(size(T)))
hold off;
xlabel('Time(s)  (Offset by 54247s)')
ylabel('Y postion(m) (Offset by 33m)')
title('Time vs Z Pos')
legend('Altitude', '+- Std deviation')

figure(5)
plot3(X,Y,T,'go')
xlabel('X pos(m)')
ylabel('Y pos(m)')
zlabel('Time(s)')
title('XY plot wrt Time')

figure(6)
plot3(X,Y,Z,'co')
xlabel('X pos(m)')
ylabel('Y pos(m)')
zlabel('Z pos(m)')
title('Position Scatter Plot')

figure(7)
histogram(X)
title('X Position')
figure(8)
histogram(Y)
title('Y Position')
figure(9)
histogram(Z)
title('Z Position')

figure(10)
plot(T, Error_x,'-r')
xlabel('Time(s)')
ylabel('X Error (m)')
title('X Error Plot')

figure(11)
plot(T, Error_y,'-r')
xlabel('Time(s)')
ylabel('Y Error (m)')
title('Y Error Plot')

figure(12)
plot(T, Error_Pos,'-r')
xlabel('Time(s)')
ylabel('Pos Error (m)')
title('Position Error Plot')
