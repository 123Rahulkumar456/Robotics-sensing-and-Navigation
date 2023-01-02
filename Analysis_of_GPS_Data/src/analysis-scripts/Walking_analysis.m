load('walking.mat')

Data1 = walkingdatagps(:,6:10);
Data2 = walkingdatagps(:,3);

Data_arr = table2array(Data1);
time = table2array(Data2);

lat = Data_arr(:,1);
lon = Data_arr(:,2);
Alt = Data_arr(:,3);
utm_x = Data_arr(:,4);
utm_y = Data_arr(:,5);

x_offset = min(utm_x);
y_offset = min(utm_y);
z_offset = min(Alt);
t_offset = min(time);

X = utm_x - (x_offset*ones(374,1));
Y = utm_y - (y_offset*ones(374,1));
Z = Alt - (z_offset*ones(374,1));
T = time - (t_offset*ones(374,1));

figure(1)
plot(X,Y,'r.')
xlabel('X position(m)  (Offset by 327476m)')
ylabel('Y postion(m) (Offset by 4688700m)')
title('East(X) vs North(Y)')

figure(2)
plot(T,X,'c.')
xlabel('Time(s)  (Offset by 70053s)')
ylabel('X position(m)  (Offset by 327476m)')
title('Time vs X Pos')

figure(3)
plot(T,Y,'b.')
xlabel('Time(s)  (Offset by 70053s)')
ylabel('Y postion(m) (Offset by 4688700m)')
title('Time vs Y Pos')

figure(4)
plot(T,Z,'g.-')
xlabel('Time(s)  (Offset by 70053s)')
ylabel('Y postion(m) (Offset by 17m)')
title('Time vs Z Pos')

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