close all;
clear all;
clc

syms k1 k2 k3 k12 k23 kin kin2 real

A = [-(k1 + k12),      k12,                 0;
      k12,            -(k12 + k23 + k2),    k23;
      0,               k23,               -(k23 + k3)];

B = [kin; 
    0; 
    0];


C = [0, 0, 1];

D = 0;

% Plug in value
A_num = double(subs(A, [k1 k12 k2 k23 k3], [0.002 0.077 0.004 0.081 0.006]));
B_num = double(subs(B, [kin], [0.034]));

sys = ss(A_num, B_num, C, D);

% b) Eigenvalues of the 3-tank system
eigen = eig(A_num);

% Initial conditions
t = 0:1:1250; % Time vector
x0_c = [0.6; 0.5; 0.4];
% c) Unforced state responses to an initial condition x0 = (60% 50% 40%)T
[~, ~, xc] = initial(sys, x0_c, t);
figure;
plot(t, xc(:,1), 'r', t, xc(:,2), 'g', t, xc(:,3), 'b');
title('Unforced Response to x_0c = [60%; 50%; 40%]');
xlabel('Time');
ylabel('State Variables');
legend('h_1', 'h_2', 'h_3');
grid on;

% d) Unforced state responses to an initial condition x0 = (0% 0% 40%)T
% Initial conditions
x0_d = [0; 0; 0.4];
[~, ~, xd] = initial(sys, x0_d, t);
figure;
plot(t, xd(:,1), 'r', t, xd(:,2), 'g', t, xd(:,3), 'b');
title('Unforced Response to x_0d = [0%; 0%; 40%]');
xlabel('Time');
ylabel('State Variables');
legend('h_1', 'h_2', 'h_3');
grid on;

% e) Three-dimensional state trajectories of the unforced responses 
figure;
plot3(xc(:,1), xc(:,2), xc(:,3), 'b'); hold on;
plot3(xd(:,1), xd(:,2), xd(:,3), 'r', 'LineWidth', 1.5);
grid on;
xlabel('h_1');
ylabel('h_2');
zlabel('h_3');
title('3D State Trajection of Unforced Responses');
legend('x_0c = [60%; 50%; 40%]', 'x0_d=[0%; 0%; 40%]');

% f 
kz1 = 0.06;
kz2 = 0.06;

% Disturbance Matrix
F = [kz1   0;
      0   kz2;
      0    0];

% Input Vector: [Time, q1_percent]
 u = [  0,       10;  % Starts at 5% for both
        1999.9,  19;  % Holds until just before 2000s
        2000,    25;  % Steps to 10% and 15%
        10000,   10];  % End of simulation

% z for f) and h)
% Disturbance Vector: [Time, Leak_Tank1, Leak_Tank2]
z = [ 0,       0,    0;   % Start: no leaks
      3999.9,  0,    0;   % Just before t=4000
      4000,   -2,    0;   % t=4000: Tank 1 leak of -2%
      5999.9, -2,    0;   % Just before t=6000
      6000,   -2,   -5;   % t=6000: Tank 2 leak of -5%
      10000,  -2, -5 ];   % End of simulation

% g
B2 = [kin,  0; 
    0,     0; 
    0, kin2];
B_num2 = double(subs(B2, [kin kin2], [0.034 0.034]));

% h 
% to run h, unblock this and change simulink to B_num to B_num2 
% Input Vector: [Time, q1_percent, q2_percent]
 % u = [  0,       5,    5;  % Starts at 5% for both
 %        1999.9,  5,    5;  % Holds until just before 2000s
 %        2000,    10,  15;  % Steps to 10% and 15%
 %        10000,   10, 15];  % End of simulation