% Loading the data
load('movingBlocked.mat')

% Extracting the desired data from the table
Data1 = movingblocked2Isaacgps(:,6:11);
Data2 = movingblocked2Isaacgps(:,3);

% Converting the table to array
Data_arr = table2array(Data1);
time = table2array(Data2);
q = Data_arr(:,6);
lat = Data_arr(:,1);
lon = Data_arr(:,2);
Alt = Data_arr(:,3);
utm_x = Data_arr(:,4);
utm_y = Data_arr(:,5);

% Offsetting the data
x_offset = mean(utm_x);
y_offset = mean(utm_y);
z_offset = mean(Alt);
t_offset = min(time);

X = utm_x - (x_offset*ones(size(utm_x)));
Y = utm_y - (y_offset*ones(size(utm_y)));
Z = Alt - (z_offset*ones(size(Alt)));
T = time - (t_offset*ones(size(time)));

figure(1)
plot(T,Z,'g.-')
xlabel('Time(s)  (Offset by 68265s)')
ylabel('Z postion(m) (Offset by 14m)')
title('Time vs Z Pos')

figure(2)
plot3(X,Y,T,'go')
xlabel('X pos(m)')
ylabel('Y pos(m)')
zlabel('Time(s)')
title('XY plot wrt Time')

figure(3)
plot3(X,Y,Z,'co')
xlabel('X pos(m)')
ylabel('Y pos(m)')
zlabel('Z pos(m)')
title('Position Scatter Plot')

% Plotting the scatter plot
figure(4)
hold on
p1 = polyfit(X(1:32,1),Y(1:32,1),1);
y_fit1 = polyval(p1,X(1:32,1));
plot(X(1:32,1),y_fit1,'-r',LineWidth=2)

p2 = polyfit(X(33:52,1),Y(33:52,1),1);
y_fit2 = polyval(p2,X(33:52,1));
plot(X(33:52,1),y_fit2,'-g',Linewidth=2)
hold on;

p3 = polyfit(X(53:76,1),Y(53:76,1),1);
y_fit3 = polyval(p3,X(53:76,1));
plot(X(53:76,1),y_fit3,'-y',LineWidth=2)
hold on;

p4 = polyfit(X(77:96,1),Y(77:96,1),1);
y_fit4 = polyval(p4,X(77:96,1));
plot(X(77:96,1),y_fit4,'-k',LineWidth=2)

rmse1 = sqrt(sum(((y_fit1-Y(1:32,1)).^2))/length(Y(1:32,1)));
rmse2 = sqrt(sum(((y_fit2-Y(33:52,1)).^2))/length(Y(33:52,1)));
rmse3 = sqrt(sum(((y_fit3-Y(53:76,1)).^2))/length(Y(53:76,1)));
rmse4 = sqrt(sum(((y_fit4-Y(77:96,1)).^2))/length(Y(77:96,1)));

fprintf("\nPrinting RMSE:\n");
fprintf("RMSE1: %f\n", rmse1);
fprintf("RMSE2: %f\n", rmse2);
fprintf("RMSE3: %f\n", rmse3);
fprintf("RMSE4: %f\n", rmse4);
fprintf("Avg RMSE: %f\n",(rmse1+rmse2+rmse3+rmse4)/4);

movingblocked2Isaacgps2 = movingblocked2Isaacgps(1:96,:);
movingblocked2Isaacgps1 = sortrows(movingblocked2Isaacgps2,11);
Data1 = movingblocked2Isaacgps1(1:96,6:11);
Data_arrs = table2array(Data1);
utm_x1 = Data_arrs(:,4);
utm_y1 = Data_arrs(:,5);
quality = Data_arrs(:,6);
x_offset1 = mean(utm_x1);
y_offset1 = mean(utm_y1);
X1 = utm_x1 - (x_offset1*ones(size(utm_x1)));
Y1 = utm_y1 - (y_offset1*ones(size(utm_y1)));

for i = 1:length(X1)
    if quality(i) == 4
        fixpos = i;
        break;
    end
end

for i = 1:length(X1)
    if quality(i) == 5
        floatpos = i;
        break;
    else
        floatpos = length(X1) + 1;
    end
end

plot(X1(fixpos:floatpos-1),Y1(fixpos:floatpos-1),'b^')
plot(X1(floatpos:length(X1)),Y1(floatpos:length(Y1)),'ko')
plot(X1(1:fixpos-1),Y1(1:fixpos-1),'rx')

xlabel('X position(m)  (Offset by 328095 m)')
ylabel('Y postion(m) (Offset by 4689331 m)')
title('Scatter Plot - East(X) vs North(Y)')
axis([-20,20,-20,35])
legend("RMSE1","RMSE2","RMSE3","RMSE4","RTK Fix","RTK Float","Uncorrected")
hold off

% Calculating the percentage of fix and float
fix = 0;
float = 0;
rest = 0;
for i = 1:length(X)
    if quality(i,1) == 4
        fix = fix + 1;
    elseif quality(i,1) == 5
        float = float + 1;
    else
        rest = rest + 1;
    end
end    

fprintf("\nPrinting RTK availability:\n")
fprintf('RTK Fix: %f %%\n',fix*100/(fix+float+rest))
fprintf('RTK Float: %f %%\n',float*100/(fix+float+rest))
fprintf('Uncorrected: %f %%\n',rest*100/(fix+float+rest))
