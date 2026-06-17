%%  3-Tank System: Linearized State-Space Model
clear; clc; close all;

%% Task a: state-space model of the 3-tank system
k1  = 0.002;
k12 = 0.077;
k2  = 0.004;
k23 = 0.081;
k3  = 0.006;
kin = 0.034;

% State-space matrices
A = [ -(k1 + k12),      k12,            0;
        k12,      -(k12 + k2 + k23),   k23;
         0,             k23,      -(k23 + k3) ];

b = [kin; 0; 0];

C = [0 0 1];
D = 0;

sys = ss(A, b, C, D);
sys

%% Task b: Determine the eigenvalues of the 3-tank system. 
% Eigenvalues
lambda = eig(A)

lambda_slow = max(real(lambda));     
Ts_99 = 4.6 / abs(lambda_slow)

% All eigenvalues of the state matrix A lie strictly in the left half-plane
% (negative real axis).
% Therefore, the 3‑tank system is asymptotically stable.

%% Task b (Additional 1): Pole-Zero Map

figure;
pzmap(sys);
title('Pole-Zero Map of 3-Level RC Network');
grid on;

%% Task b (Additional 2): Step Response
% Time vector
t = 0:0.1:2000;

% Step input ue = 1
u = ones(size(t));

% Simulate system response
[y, t, x] = lsim(sys, u, t);

% Plot
figure;
plot(t, x(:,1)); hold on;
plot(t, x(:,2));
plot(t, x(:,3));
plot(t, u);   % input u

grid on;
xlabel('Time (s)');
ylabel('level of tank (%)');
title('Step Response of 3-tank system (u = 1)');
legend('x_1(t)', 'x_2(t)', 'x_3(t)', 'u(t)');

%% Task c: Unforced response of the 3-tank system
% Initial condition x0 = (60%, 50%, 40%)^T

t = 0:1:2000;          % simulation time (s)
x0 = [0.60; 0.50; 0.40];   % initial liquid levels (normalized)
u = zeros(size(t));        % unforced: u(t) = 0

% Simulate
[y_free, t_free, x_free] = lsim(sys, u, t, x0);

% Plot
figure;
plot(t_free, x_free(:,1)); hold on;
plot(t_free, x_free(:,2));
plot(t_free, x_free(:,3));

grid on;
xlabel('Time (s)');
ylabel('Liquid Level (%)');
title('Unforced Response of the 3-Tank System (x_0 = [60%, 50%, 40%]^T)');
legend('h_1(t)', 'h_2(t)', 'h_3(t)');

%% Task d: Unforced response of the 3-tank system
% Initial condition x0 = (0 ,0 , 40%)^T

t = 0:1:2000;          % simulation time (s)
x0 = [0; 0; 0.40];   % initial liquid levels (normalized)
u = zeros(size(t));        % unforced: u(t) = 0

% Simulate
[y_free, t_free, x_free] = lsim(sys, u, t, x0);

% Plot
figure;
plot(t_free, x_free(:,1)); hold on;
plot(t_free, x_free(:,2));
plot(t_free, x_free(:,3));

grid on;
xlabel('Time (s)');
ylabel('Liquid Level (%)');
title('Unforced Response of the 3-Tank System (x_0 = [60%, 50%, 40%]^T)');
legend('h_1(t)', 'h_2(t)', 'h_3(t)');


%% Task e: 3D State Trajectories of the 3-Tank System
% Unforced responses from initial conditions:
% (c) x0 = [0.60; 0.50; 0.40]
% (d) x0 = [0; 0; 0.40]

t = 0:1:2000;          % simulation time
u = zeros(size(t));    % unforced input

% Initial conditions
x0_c = [0.60; 0.50; 0.40];
x0_d = [0; 0; 0.40];

% Simulate both trajectories
[~, t_c, x_c] = lsim(sys, u, t, x0_c);
[~, t_d, x_d] = lsim(sys, u, t, x0_d);

% Plot 3D trajectories
figure;
plot3(x_c(:,1), x_c(:,2), x_c(:,3)); hold on;
plot3(x_d(:,1), x_d(:,2), x_d(:,3));

grid on;
xlabel('h_1(t)');
ylabel('h_2(t)');
zlabel('h_3(t)');
title('3D State Trajectories of the 3-Tank System (Unforced Responses)');
legend('Trajectory from [0.60, 0.50, 0.40]^T', ...
       'Trajectory from [0, 0, 0.40]^T');
view(45, 25);   % nicer 3D angle

%% Task f: Implement the state-space model of the 3-tank system in Simulink

time = out.tout; 

% Directly extract raw numeric arrays from timeseries objects
x_matrix = out.x_out.Data; 
u_matrix = out.u_out.Data; 
z_matrix = out.z_out.Data; 

z1_signal = z_matrix(:, 1); 
z2_signal = z_matrix(:, 2); 

figure('Color', 'w', 'Name', 'Task f: System Responses');

% Subplot 1: State Responses (x1, x2, x3)
subplot(2, 1, 1);
plot(time, x_matrix(:, 1), 'r-'); hold on;
plot(time, x_matrix(:, 2), 'g-');
plot(time, x_matrix(:, 3), 'b-');
title('Task f: 3-Tank System State Responses with Leaks');
xlabel('Time t (s)');
ylabel('Liquid Level x_i (m)'); 
legend('x_1 (Tank 1)', 'x_2 (Tank 2)', 'x_3 (Tank 3)', 'Location', 'Best');
xlim([0, 10000]);
grid on;

% Subplot 2: Control Input and Disturbances (u, z1, z2)
subplot(2, 1, 2);
plot(time, u_matrix, 'k-'); hold on;
plot(time, z1_signal, 'm--');
plot(time, z2_signal, 'c-.');
title('Control Input u(t) and Disturbance Signals z(t)');
xlabel('Time t (s)');
ylabel('Signal Amplitude');
legend('Control Input u(t)', 'Disturbance z_1(t) (Tank 1)', 'Disturbance z_2(t) (Tank 2)', 'Location', 'Best');
xlim([0, 10000]);
grid on;

%% Task g

kin1 = 0.034; 
kin2 = 0.034;

B = [ kin1,   0;
       0,     0;
       0,   kin2 ];

D = [0 0];

%% Task g+h: Plot responses of modified 3-tank system (two inputs + leaks)

time = out.tout;

% Extract numeric arrays from timeseries
x_matrix = out.x_out.Data;      % [x1 x2 x3]
u_matrix = out.u_out.Data;      % [u1 u2]
z_matrix = out.z_out.Data;      % [z1 z2]

u1_signal = u_matrix(:, 1);
u2_signal = u_matrix(:, 2);

z1_signal = z_matrix(:, 1);
z2_signal = z_matrix(:, 2);

figure('Color', 'w', 'Name', 'Task g+h: Modified System Responses');

% Subplot 1: State Responses (x1, x2, x3)
subplot(2, 1, 1);
plot(time, x_matrix(:, 1), 'r-'); hold on;
plot(time, x_matrix(:, 2), 'g-');
plot(time, x_matrix(:, 3), 'b-');

title('Task g+h: State Responses of Modified 3-Tank System');
xlabel('Time t (s)');
ylabel('Liquid Level x_i (m)');
legend('x_1 (Tank 1)', 'x_2 (Tank 2)', 'x_3 (Tank 3)', 'Location', 'Best');
xlim([0, 10000]);
grid on;

% Subplot 2: Inputs (u1, u2) and Disturbances (z1, z2)
subplot(2, 1, 2);
plot(time, u1_signal, 'k-'); hold on;
plot(time, u2_signal, 'm-');
plot(time, z1_signal, 'r--');
plot(time, z2_signal, 'b--');

title('Inputs u(t) and Disturbances z(t)');
xlabel('Time t (s)');
ylabel('Signal Amplitude');
legend('u_1(t) (Tank 1 inflow)', ...
       'u_2(t) (Tank 3 inflow)', ...
       'z_1(t) (Leak Tank 1)', ...
       'z_2(t) (Leak Tank 2)', ...
       'Location', 'Best');
xlim([0, 10000]);
grid on;
