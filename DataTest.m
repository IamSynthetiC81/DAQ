clear; clc; close all;
%% Data parse

file_path = 'putty.txt';

% Read all lines from the text file
fid = fopen(file_path, 'r');
lines = textscan(fid, '%s', 'Delimiter', '\n');

fclose(fid);

% Initialize arrays to store accelerometer data
num_lines = length(lines{1});
acc_x = zeros(num_lines, 1);
acc_y = zeros(num_lines, 1);
acc_z = zeros(num_lines, 1);
time = zeros(num_lines, 1);

G = 9.81;

% Parse data from each line
for i = 1:num_lines
    % disp(lines{0,0}{i});
    line_data = sscanf(lines{1,1}{i}, '%f,%f,%f,%f,%f,%f,%d,%d');
    acc_x(i) = G*line_data(1)/16384.0;
    acc_y(i) = G*line_data(2)/16384.0;
    acc_z(i) = G*line_data(3)/16384.0;
    time(i) = line_data(8)/1000;
end

time = time-time(1);

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



acc_xy = sqrt(acc_x.^2+acc_y.^2);

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

%%

function [power0,f0] = FFT(y,num_lines,Fs)
    f = (0:num_lines-1)*(Fs/num_lines);
    y = fft(y);
    power = abs(y).^2/num_lines;
    
    y0 = fftshift(y);                                       % shift y values
    f0 = (-num_lines/2:num_lines/2-1)*(Fs/num_lines);       % 0-centered frequency range
    power0 = abs(y0).^2/num_lines;                          % 0-centered power

end
