% Loading the data
load('stationaryBlocked.mat')
stationarydatablockedgps = sortrows(stationarydatablockedgps,11);

% Extracting the desired data from the table
Data1 = stationarydatablockedgps(:,6:10);
Data2 = stationarydatablockedgps(:,3);
Data3 = stationarydatablockedgps(:,11);

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
x_offset = mean(utm_x);
y_offset = mean(utm_y);
z_offset = mean(Alt);
t_offset = min(time);

X = utm_x - (x_offset*ones(size(utm_x)));
Y = utm_y - (y_offset*ones(size(utm_y)));
Z = Alt - (z_offset*ones(size(Alt)));
T = time - (t_offset*ones(size(time)));

Error_x = utm_x - (mean(utm_x)*ones(size(utm_x)));
Error_y = utm_y - (mean(utm_y)*ones(size(utm_y)));
Error_Pos = (Error_x.^2+Error_y.^2).^0.5;

% Calculating the standard deviation
std_x = std(utm_x);
std_y = std(utm_y);
std_z = std(Alt);

fprintf("\nPrinting Standard deviation:\n")
fprintf("Standard deviation in X(East): %f m\n", std_x)
fprintf("Standard deviation in Y(North): %f m\n", std_y)
fprintf("Standard deviation in Z(Altitude): %f m\n", std_z)

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

% Calculating the CEP and 2DRMS
sigma_x = std(X);
sigma_y = std(Y);
CEP = 0.59 * (sigma_x+sigma_y);
RMS_2D = 2 * sqrt(sigma_x^2 + sigma_y^2);

fprintf("\nPrinting CEP and 2DRMS values:\n");
fprintf('CEP: %f m\n',CEP);
fprintf('2DRMS: %f m\n',RMS_2D);

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
plot(X(floatpos:length(X)),Y(floatpos:length(Y)),'mo')
plot(X(1:fixpos-1),Y(1:fixpos-1),'rx')
plot((mean(utm_x)-x_offset),(mean(utm_y)-y_offset),'g+',MarkerSize=10)
xlabel('X position(m)  (Offset by 328086.74 m)')
ylabel('Y postion(m) (Offset by 4689321.95 m)')
title('Scatter Plot - East(X) vs North(Y)')
%legend("RTK Fix", "RTK Float", "Uncorrected")
axis([-0.8, 0.8, -0.8, 0.8])

th = 0:pi/50:2*pi;
xCEP = CEP * cos(th);
yCEP = CEP * sin(th) ;
plot(xCEP, yCEP,'r');
axis equal

th = 0:pi/50:2*pi;
xDRMS = RMS_2D * cos(th);
yDRMS = RMS_2D * sin(th) ;
plot(xDRMS, yDRMS,'k');
axis equal

legend("RTK Fix Pos","RTK Float", "Uncorrected","Mean Pos","CEP (50%)","2DRMS (95%)")
hold off;

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
