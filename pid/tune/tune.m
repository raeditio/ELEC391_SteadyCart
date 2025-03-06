clc; clear; close all;

% Define system parameters
M = 2.0;    % Cart mass (kg)
m = 0.5;    % Pendulum mass (kg)
l = 0.3;    % Pendulum length (m)
g = 9.81;   % Gravity (m/s²)

% Defien transfer function G(s) = 1 / (s² - g/l)
num = [1];
den = [M + m - (m^2 * l^2) / (M + m), 0, -g / l];

% Create the transfer function
G = tf(num, den);

%Atuomatically tune the PID controller
C = pidtune(G, 'PID');

% Display tuned PID parameters
Kp = C.Kp;
Ki = C.Ki;
Kd = C.Kd;
disp(['Tuned PID: Dp = ', num2str(Kp), ', Ki = ', num2str(Ki), ', Kd = ', num2str(Kd)]);

% Closed-loop system
G_cl = feedback(C * G, 1);

% Simulate step response
t = 0:0.01:5;   % Time vector
[y, t] = step(G_cl, t);

% Plot step response
figure;
plot(t, y, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pendulum Angle');
title('Step Response of PID-Controlled Inverted Pendulum');
grid on;