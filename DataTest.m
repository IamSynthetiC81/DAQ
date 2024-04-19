clear; clc; close all;
%% Data parse
filename = 'putty.txt';

fid = fopen(filename, 'r');
% Check if file exists
if fid == -1
    error('File not found');
end

lines = readlines("putty.txt");

% Initialize the variables
data = [];

% Initialize arrays to store accelerometer data
num_lines = length(lines);
accX = zeros(num_lines,1);
accY = zeros(num_lines,1);
accZ = zeros(num_lines,1);
gyrX = zeros(num_lines,1);
gyrY = zeros(num_lines,1);
gyrZ = zeros(num_lines,1);
time = zeros(num_lines, 1);

% Parse data from each line
for i = 1:num_lines
    % disp(lines{0,0}{i});
    line_data = sscanf(lines(i), "%d,%d,%d,%d,%d,%d,%f,%f,%d,%f,%d,%d");

    % ax = typecast(uint8(buffer(1:2)), 'int16');
    % ay = typecast(uint8(buffer(3:4)), 'int16');
    % az = typecast(uint8(buffer(5:6)), 'int16');
    % gx = typecast(uint8(buffer(9:10)), 'int16');
    % gy = typecast(uint8(buffer(11:12)), 'int16');
    % gz = typecast(uint8(buffer(13:14)), 'int16');

    accX(i) = (line_data(1)-2^15)/16384.0;
    accY(i) = (line_data(2)-2^15)/16384.0;
    accZ(i) = (line_data(3)-2^15)/16384.0;
   
    gyrX(i) = line_data(3)/ 131.0;
    gyrY(i) = line_data(4)/ 131.0;
    gyrZ(i) = line_data(5)/ 131.0;

    time(i) = line_data(12)/1000;
end


fclose(fid);

% Extract the data
% uint8_data = data(:, 1:64);
% float1_data = data(:, 65);
% float2_data = data(:, 66);
% int1_data = data(:, 67);
% float3_data = data(:, 68);
% int2_data = data(:, 69);
% int3_data = data(:, 70);

% time = time-time(1);

acc_xy = sqrt(accX.^2+accY.^2);

% plot(time, acc_xy);



%% Sampling Rate 
figure('Name','Sampling Rate Data','NumberTitle','off');

dt_t = diff(time);

Fs_t = zeros(floor(time(end)),1);
sec_v = (1:floor(time(end)));

dt = mean(diff(time));
dt_e = std(diff(time));
Fs=1/dt;

for i = (1:floor(time(end)))
    Fs_t(i) = sum(time>(i-1) & time<i);
end


Fs_v = ones(floor(time(end))+1,1)*Fs;

stem(sec_v,Fs_t); hold on;
plot((0:floor(time(end))),Fs_v); grid minor;
ylabel("Samples-per-seconds");

yyaxis right;

plot(time(2:end),dt_t,':');

xlabel("Time (s)");
ylabel("Delay between samples");
title("System Response");
legend("Samples per Seconds","Mean", "Delay between packets");


fprintf("Sampling Frequency = %1.1d \x00B1 %1.1d\n",Fs,1/dt_e);
%% Raw Data
figure('Name','Raw Data','NumberTitle','off');

G=9.81;

acc_xy = sqrt(accX.^2+accY.^2);

acc_xy_km_h = acc_xy/12959.99997097;
hours = time/3600;

speed_xy = cumtrapz(acc_xy_km_h,hours);

plot(time,acc_xy/G); hold on;
ylabel('XY-Acceleration (G)');

yyaxis right;
plot(time,speed_xy); grid minor;
title("Calculated XY Speed");
legend("Acceleration","Speed");
xlabel("Time (s)");
ylabel('Calculated speed (Km/h)');

%% Spectral-Analysis on Compensated Data
figure('Name','Spectral Analysis','NumberTitle','off');

[y,f] = FFT(acc_xy,num_lines,Fs);

fft_result_db = 20*log10(y);

plot(f,fft_result_db) ; grid minor;
ylim([-60,100]);
title('Frequency Domain Plot');
xlabel('Frequency')
ylabel('Power (dB)')

%% Applying a Low-Pass Filter
figure('Name','Low Pass Filter','NumberTitle','off');

acc_lpf = lowpass(acc_xy_km_h,5,Fs);
speed_lpf = cumtrapz(acc_lpf,hours);
plot(time,speed_xy, ":"); hold on; 
plot(time,speed_lpf); grid on;

title("Speed through Low-Pass Filter");
legend("Original","LPF");
xlabel("Time (s)");
ylabel('Speed (Km/h)');
grid minor;

figure('Name','LPF + Moving-Average Filter','NumberTitle','off');
subplot(2,1,1);
for length = (1:6)
    F = filter(1/length * ones(length ,1),1,speed_lpf);
    
    plot(time,F); hold on; grid minor;
end
legend("Length = 10^1","Length = 10^2","Length = 10^3","Length = 10^4","Length = 10^5","Length = 10^6");

title("LPF + AMW Speed Calculations");
xlabel("Time (s)");
ylabel('Calculated speed (Km/h)');
grid minor;

subplot(2,1,2);
for length = (1:6)
    F = filter(1/length * ones(length ,1),1,speed_xy);
    subplot(2,1,2);
    plot(time,F); hold on; grid minor;   
end
legend("Length = 10^1","Length = 10^2","Length = 10^3","Length = 10^4","Length = 10^5","Length = 10^6");

title("MAW + Speed Calculations");
xlabel("Time (s)");
ylabel('Calculated speed (Km/h)');
grid minor;

%% Applying Moving-Average Filter
figure('Name','Moving Average Filter','NumberTitle','off');

subplot(2,1,1);
plot(time,speed_xy); grid minor;
legend("Acceleration on X-Y","Speed");

xlabel("Time (s)");
ylabel('Calculated speed (Km/h)');

for length = logspace(1,6,6)
    F = filter(1/length * ones(length ,1),1,speed_xy);
    subplot(2,1,2);
    plot(time,F); hold on;

    
end
legend("Length = 10^1","Length = 10^2","Length = 10^3","Length = 10^4","Length = 10^5","Length = 10^6");

title("Filtered X-Y Measurements");
xlabel("Time (s)");
ylabel('Calculated speed (Km/h)');
grid minor;



%%

function [power0,f0] = FFT(y,num_lines,Fs)
    f = (0:num_lines-1)*(Fs/num_lines);
    y = fft(y);
    power = abs(y).^2/num_lines;
    
    y0 = fftshift(y);                                       % shift y values
    f0 = (-num_lines/2:num_lines/2-1)*(Fs/num_lines);       % 0-centered frequency range
    power0 = abs(y0).^2/num_lines;                          % 0-centered power

end

% function [ax, ay, az, gx, gy, gz] = getMotion6(devAddr, buffer)
%     % Read bytes from the device
%     buffer = I2Cdev.readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, I2Cdev.readTimeout, wireObj);
% 
%     % Extract acceleration and gyroscope data
% 
% end
