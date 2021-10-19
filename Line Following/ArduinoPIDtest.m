% ArduinoPIDtest.m
% Prof. Stephen Mascaro
% 10/11/2021

% This program uses serial comms to send a set of PID control parameters to
% an Arduino and receive back an experimental data set from the Arduino

clear, clc

% use the Arduino IDE to identify the serial port for your Arduino
COM = 'com3';  % serial comm port of Arduino
BaudRate = 57600;  % this must match the BaudRate setting in your Arduino sketch

% define control parameters
Kp=100;
Kd=0;
Ki=0;
BaseSpeed = 100;

s1 = serialport(COM,BaudRate)  % open serial port
readline(s1)  % wait for ready command from Arduino

% Put control parameters in a string
STR = sprintf('%4.2f %4.2f %4.2f %4.2f \n',[Kp,Kd,Ki,BaseSpeed])
writeline(s1,STR)   % Transmit string of control parameters to Arduino

% Read data
pause(3)  % Give the Arduino time to complete the experiment
i = 1;
while s1.NumBytesAvailable > 0    
    % receive data from Arduino one row at a time
    data(i,:) = str2num(readline(s1));  
    i = 1 + i;
end
disp(data)  % display the data
t = data(:,1); % first column of data is typically time
y = data(:,2:end); % other columns are variables of interest
N = size(y,2);  % # of variables to plot
S = {'error','motor 1 command','motor 2 command'};  % name of variables to plot
for k=1:N
    subplot(N,1,k)
    plot(t,y(:,k),'o')
    xlabel('Time')
    ylabel(S(k));
end

clear s1  % close serial port