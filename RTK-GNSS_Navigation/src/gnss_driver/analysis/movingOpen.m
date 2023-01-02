% Loading the data
load('movingOpen.mat')
movingfieldgps = sortrows(movingfieldgps,11);

% Extracting the desired data from the table
Data1 = movingfieldgps(:,6:10);
Data2 = movingfieldgps(:,3);
Data3 = movingfieldgps(:,11);

% Converting the table to array
Data_arr = table2array(Data1);
time = table2array(Data2);
quality = table2array(Data3);

lat = Data_arr(:,1);
lon = Data_arr(:,2);
Alt = Data_arr(:,3);
utm_x = Data_arr(:,4);
utm_y = Data_arr(:,5);

% Offsetting the data
x_offset = min(utm_x);
y_offset = min(utm_y);
z_offset = min(Alt);
t_offset = min(time);

X = utm_x - (x_offset*ones(205,1));
Y = utm_y - (y_offset*ones(205,1));
Z = Alt - (z_offset*ones(205,1));
T = time - (t_offset*ones(205,1));

% Plotting the scatter plot
for i = 1:length(X)
    if quality(i) == 4
        fixpos = i;
        break;
    end
end

for i = 1:length(X)
    if quality(i) == 5
        floatpos = i;
        break;
    else
        floatpos = length(X) + 1;
    end
end

figure(1)
hold on
plot(X(fixpos:floatpos-1),Y(fixpos:floatpos-1),'b^')
plot(X(floatpos:length(X)),Y(floatpos:length(Y)),'ko')
plot(X(1:fixpos-1),Y(1:fixpos-1),'rx')
xlabel('X position(m)  (Offset by 326775 m)')
ylabel('Y postion(m) (Offset by 4688342 m)')
title('Scatter Plot - East(X) vs North(Y)')
axis([-10,90,-10,90])

p1 = polyfit(X(1:60,1),Y(1:60,1),1);
y_fit1 = polyval(p1,X(1:60,1));
plot(X(1:60,1),y_fit1,'-r',LineWidth=2)

p2 = polyfit(X(61:100,1),Y(61:100,1),1);
y_fit2 = polyval(p2,X(61:100,1));
plot(X(61:100,1),y_fit2,'-g',LineWidth=2)

p3 = polyfit(X(101:165,1),Y(101:165,1),1);
y_fit3 = polyval(p3,X(101:165,1));
plot(X(101:165,1),y_fit3,'-y',LineWidth=2)

p4 = polyfit(X(166:205,1),Y(166:205,1),1);
y_fit4 = polyval(p4,X(166:205,1));
plot(X(166:205,1),y_fit4,'-k',LineWidth=2)
legend("RTK Fix", "RMSE1", "RMSE2","RMSE3","RMSE4")
hold off

rmse1 = sqrt(sum(((y_fit1-Y(1:60,1)).^2))/length(Y(1:60,1)));
rmse2 = sqrt(sum(((y_fit2-Y(61:100,1)).^2))/length(Y(61:100,1)));
rmse3 = sqrt(sum(((y_fit3-Y(101:165,1)).^2))/length(Y(101:165,1)));
rmse4 = sqrt(sum(((y_fit4-Y(166:205,1)).^2))/length(Y(166:205,1)));

fprintf("\nPrinting RMSE:\n");
fprintf("RMSE1: %f\n", rmse1);
fprintf("RMSE2: %f\n", rmse2);
fprintf("RMSE3: %f\n", rmse3);
fprintf("RMSE4: %f\n", rmse4);
fprintf("Avg RMSE: %f\n",(rmse1+rmse2+rmse3+rmse4)/4);

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

figure(2)
plot(T,Z,'g.-')
xlabel('Time(s)  (Offset by 57452s)')
ylabel('Z postion(m) (Offset by 56m)')
title('Time vs Z Pos')

figure(3)
plot3(X,Y,T,'go')
xlabel('X pos(m)')
ylabel('Y pos(m)')
zlabel('Time(s)')
title('XY plot wrt Time')

figure(4)
plot3(X,Y,Z,'co')
xlabel('X pos(m)')
ylabel('Y pos(m)')
zlabel('Z pos(m)')
title('Position Scatter Plot')