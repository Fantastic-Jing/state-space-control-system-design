%% Prep
clc
clear
close all

% 2.1
% a)
% Static gain
Km = 1;
% Damping ratio 
zeta = 0.59;
% Time constant
T0 = 0.057;

% b)
% Ball gain
Kb = 0.028;   

% 2.2 Simulink Model
% Disturbances to Zero
z1 = 0; 
z2 = 0;
z3 = 0;

%%
% 2.3 2.4
% a) Implement the state-space model of the plan
A =      [  0   1     0          0;
            0   0     Kb         0;
            0   0     0          1;        
            0   0  -1/T0^2  -2*zeta/T0  ];

B = [0; 
     0; 
     0; 
     Km/(T0^2)];

C = [1, 0, 0, 0];

D = 0;
% Create the state-space system object
sys = ss(A, B, C, D);

% b) Determine the eigenvalues of the system
sys_eigenvalues = eig(A)

% c) Analyze the controllability and observability of the system
%  Controllability Matrix 
Qr = ctrb(A, B);
rank_Qr = rank(Qr);

% Observability Matrix
Qo = obsv(A, C);
rank_Qo = rank(Qo);

% number of states
n = size(A, 1);  

if rank_Qr == n 
    disp('The system is controllable.');
else
    disp('The system is not controllable.');
end

if rank_Qo == n
    disp('The system is observable.');
else
    disp('The system is not observable.');
end

% d) Simulate the state responses

% Create a high-resolution time array for the open-loop test
dt = (0:0.005:10)'; 

% Step input signal u(t): from 0 to 10
u_signal = zeros(size(dt));
u_signal(dt >= 0) = 10; 
u = [dt, u_signal];

% Run the open-loop simulation
out = sim("BallBalancingwoFb.slx", 'StopTime', '15'); 

% Time vector and plotting limits
t_plot_ol = out.P_b.Time;
x_limits_ol = [0 10];

%% Create one figure with 5 subplots
figure(21); clf;

subplot(5,1,1);
plot(t_plot_ol, out.P_b.Data, 'b-', 'LineWidth', 1.5);
title('Open-Loop Step Response: Ball Position P_b(t)');
xlabel('Time [s]'); ylabel('Position [m]');
xlim(x_limits_ol); legend('P_b(t)'); grid on;

subplot(5,1,2);
plot(t_plot_ol, out.V_b.Data, 'Color', [0 0.5 0], 'LineWidth', 1.5);
title('Open-Loop Step Response: Ball Velocity V_b(t)');
xlabel('Time [s]'); ylabel('Velocity [m/s]');
xlim(x_limits_ol); legend('V_b(t)'); grid on;

subplot(5,1,3);
plot(t_plot_ol, out.P_alpha.Data, 'm-', 'LineWidth', 1.5);
title('Open-Loop Step Response: Motor Crank Angle \alpha(t)');
xlabel('Time [s]'); ylabel('Angle [deg]');
xlim(x_limits_ol); legend('\alpha(t) (Stable Subsystem)'); grid on;

subplot(5,1,4);
plot(t_plot_ol, out.V_alpha.Data, 'c-', 'LineWidth', 1.5);
title('Open-Loop Step Response: Motor Crank Angular Velocity v_\alpha(t)');
xlabel('Time [s]'); ylabel('Angular Velocity [deg/s]');
xlim(x_limits_ol); legend('v_\alpha(t)'); grid on;

subplot(5,1,5);
u_plot = zeros(size(t_plot_ol));
u_plot(t_plot_ol >= 0) = 10;
plot(t_plot_ol, u_plot, 'r--', 'LineWidth', 1.5);
title('Open-Loop Step Input Signal u(t) (\alpha_{ref})');
xlabel('Time [s]'); ylabel('Input Angle [deg]');
xlim(x_limits_ol); legend('Input Step to 10^\circ'); grid on;

%% Main
clc
clear
close all
% 2.1
% a)
% Static gain
Km = 1;
% Damping ratio 
zeta = 0.59;
% Time constant
T0 = 0.057;

% b)
% Ball gain
Kb = 0.028;   

% 2.2 Simulink Model
% Disturbances to Zero
z1 = 0; 
z2 = 0;
z3 = 0;

% 2.3 2.4
% a) Implement the state-space model of the plan
A = [0, 1, 0,                  0;
     0, 0, Kb,                 0;
     0, 0, 0,                  1;
     0, 0, -1/(T0^2), -2*zeta/T0];

B = [0; 
     0; 
     0; 
     Km/(T0^2)];

C = [1, 0, 0, 0];

D = 0;

%% 3.1 Model of the PI state-feedback control system
% a) Define Extended Matrices (Size 5x5 and 5x1)
Ae = [ A,         zeros(4,1);
      -C,         0         ];

Be = [ B;
       0 ];

Bw = [ zeros(4,1);
       1          ];

disp('Extended Matrix:');
disp(Ae);

% b) Simulink model

% 3.2 Design of a PI state-feedback controller by pole placement

design = struct('poles', {}, 'K', {});

% desired_poles = [-1 + 1.0487i, -1 - 1.0487i, -5, -5.01, -5.02];
% a) Poles for ts = 4s, no overshoot
ts = 4;
zeta_d = 1;          % no overshoot -> critically damped
wn_d   = 4 / (zeta_d*ts);
re = -zeta_d*wn_d;
im = 0;

p_dom = [re, re];    % repeated real poles
p_far = 5*re * ones(1,3);

design(1).poles = [p_dom, p_far];

Kp = 1;

% d) Poles for ts = 4s, overshoot ~4-5%
ts = 4;
Mp = 0.04;
zeta_d = abs(log(Mp)) / sqrt(pi^2 + log(Mp)^2);
wn_d   = 4 / (zeta_d*ts);

re = -zeta_d*wn_d;
im =  wn_d*sqrt(1-zeta_d^2);
p_dom = [re+1i*im, re-1i*im];
p_far = 5*re * ones(1,3);
design(2).poles = [p_dom, p_far];

% b)
% Time Array (0 to 60 seconds)
dt2 = (0:0.002:60)';

% Reference input w(t): Step from 0 to 0.2m at t = 0
w_signal = zeros(size(dt2));
w_signal(dt2 >= 0) = 0.2; 
w = timeseries(w_signal, dt2); % Map to From Workspace block 'w'

% Disturbance z1: Step from 0 to 1 deg at t = 15s (Crank Angle)
z1_signal = zeros(size(dt2));
z1_signal(dt2 >= 15) = 1.0; 
z1 = timeseries(z1_signal, dt2);

% Disturbance z2: Step from 0 to 0.1 m/s at t = 30s (Ball Velocity)
z2_signal = zeros(size(dt2));
z2_signal(dt2 >= 30) = 0.1; 
z2 = timeseries(z2_signal, dt2);

% Disturbance z3: Step from 0 to 0.05 m at t = 45s (Ball Position)
z3_signal = zeros(size(dt2));
z3_signal(dt2 >= 45) = 0.05; 
z3 = timeseries(z3_signal, dt2);

for i = 1:2
    % Compute the extended gain vector Ke
    K_tilde = acker(Ae, Be, design(i).poles);
    
    % K_tilde = [k1, k2, k3, k4, kI]
    K1 = K_tilde(1)- Kp*C(1); % Total coefficient assigned to position p_B
    K2 = K_tilde(2) - Kp*C(2); % Ball velocity gain
    K3 = K_tilde(3) - Kp*C(3); % Crank angle gain
    K4 = K_tilde(4) - Kp*C(4); % Crank angular velocity gain
    Ki = -K_tilde(5);   % Gain for the integral channel
    
    design(i).K = [K1 K2 K3 K4 Ki];
    % Print results like the sample code
    fprintf('Pole placement design K case %d :\n', i);
    disp(design(i).K);
    
    out2 = sim("BallBalancingwFb.slx", 'StopTime', '60');
    
    t_plot = out2.P_b.Time;
    
    % Extract signals from simulation output structure
    x1_pB   = out2.P_b.Data;         % Ball Position
    x2_vB   = out2.V_b.Data;         % Ball Velocity
    x3_al   = out2.P_alpha.Data;     % Crank Angle
    x4_va   = out2.V_alpha.Data;     % Crank Angular Velocity
    
    % Common time limits to ensure identical horizontal scaling
    x_limits = [0 60];
    
    % --- Figure frint out ---
    figure;
    subplot(5,1,1);
    plot(t_plot, x1_pB, 'b', 'LineWidth', 1.5);
    hold on;
    w_plot = zeros(size(t_plot));
    w_plot(t_plot >= 0) = 0.2; 
    plot(t_plot, w_plot, 'r--', 'LineWidth', 1.2);
    title('Ball Position x_1(t) Response');
    xlabel('Time [s]'); ylabel('Position [m]');
    xlim(x_limits); legend('Actual Position p_B', 'Reference w'); grid on;
    
    subplot(5,1,2);
    plot(t_plot, x2_vB, 'Color', [0 0.5 0], 'LineWidth', 1.5);
    title('Ball Velocity x_2(t) Response');
    xlabel('Time [s]'); ylabel('Velocity [m/s]');
    xlim(x_limits); legend('v_B(t)'); grid on;
    
    subplot(5,1,3);
    plot(t_plot, x3_al, 'm', 'LineWidth', 1.5);
    title('Motor Crank Angle x_3(t) Response');
    xlabel('Time [s]'); ylabel('Angle [deg]');
    xlim(x_limits); legend('\alpha(t)'); grid on;
    
    subplot(5,1,4);
    plot(t_plot, x4_va, 'c', 'LineWidth', 1.5);
    title('Motor Crank Angular Velocity x_4(t) Response');
    xlabel('Time [s]'); ylabel('Angular Velocity [deg/s]');
    xlim(x_limits); legend('v_\alpha(t)'); grid on;
    
    subplot(5,1,5);
    plot(t_plot, out2.u.Data, 'r', 'LineWidth', 1.5);
    title('Actuating Control Input u(t) (\alpha_{ref})');
    xlabel('Time [s]'); ylabel('Control Input [deg]');
    xlim(x_limits); grid on;

end 

%% 3.3 Design of a PI state-feedback controller by LQR

design = struct('Q', {});
% 1. Define the exact Q and R matrices
% Q = [1,   0,   0,    0,       0;
%      0,   1,   0,    0,       0;
%      0,   0,   0.1,  0,       0;
%      0,   0,   0,    0.01,    0;
%      0,   0,   0,    0,    1000];

design(1).Q = diag([1, 1, 0.1, 0.01, 1000]);
design(2).Q = diag([1, 1, 0.1, 0.01, 10]);
R = 1;

for i = 1:2
    % 2. Compute the optimal feedback gain vector via LQR
    Q = design(i).Q;
    [Ke_lqr, S, P] = lqr(Ae, Be, Q, R);
    
    % 3. Extract and isolate the individual paths using KP = 1
    Kp = 1; 
    K1 = Ke_lqr(1)- Kp*C(1); % Total coefficient assigned to position p_B
    K2 = Ke_lqr(2) - Kp*C(2); % Ball velocity gain
    K3 = Ke_lqr(3) - Kp*C(3); % Crank angle gain
    K4 = Ke_lqr(4) - Kp*C(4); % Crank angular velocity gain
    Ki = -Ke_lqr(5);   % Gain for the integral channel
    
    K_tilde = real(Ke_lqr);
    
    fprintf('LQR design K case %d :\n', i);
    disp(K_tilde);

    out3 = sim("BallBalancingwFb.slx", 'StopTime', '60');
    
    t_plot = out3.P_b.Time;
    
    % Extract signals from LQR simulation output structure
    x1_pB   = out3.P_b.Data;         % Ball Position
    x2_vB   = out3.V_b.Data;         % Ball Velocity
    x3_al   = out3.P_alpha.Data;     % Crank Angle
    x4_va   = out3.V_alpha.Data;     % Crank Angular Velocity
    
    % Common time limits for consistent horizontal scaling
    x_limits = [0 60];
    
    % --- Figure 31: Ball Position ---
    figure; 
    subplot(5,1,1);
    plot(t_plot, x1_pB, 'b-', 'LineWidth', 1.5); hold on;
    w_plot = zeros(size(t_plot)); w_plot(t_plot >= 0) = 0.2; % Reference line
    plot(t_plot, w_plot, 'r--', 'LineWidth', 1.2);
    title('LQR Response: Ball Position x_1(t) vs Reference w(t)');
    xlabel('Time [s]'); ylabel('Position [m]');
    xlim(x_limits); legend('Actual Position p_B', 'Reference w'); grid on;
    
    % --- Figure 32: Ball Velocity ---
    subplot(5,1,2);
    plot(t_plot, x2_vB, 'Color', [0 0.5 0], 'LineWidth', 1.5);
    title('LQR Response: Ball Velocity x_2(t)');
    xlabel('Time [s]'); ylabel('Velocity [m/s]');
    xlim(x_limits); legend('v_B(t)'); grid on;
    
    % --- Figure 33: Crank Angle ---
    subplot(5,1,3);
    plot(t_plot, x3_al, 'm-', 'LineWidth', 1.5);
    title('LQR Response: Motor Crank Angle x_3(t)');
    xlabel('Time [s]'); ylabel('Angle [deg]');
    xlim(x_limits); legend('\alpha(t)'); grid on;
    
    % --- Figure 34: Crank Angular Velocity ---
    subplot(5,1,4);
    plot(t_plot, x4_va, 'c-', 'LineWidth', 1.5);
    title('LQR Response: Motor Crank Angular Velocity x_4(t)');
    xlabel('Time [s]'); ylabel('Angular Velocity [deg/s]');
    xlim(x_limits); legend('v_\alpha(t)'); grid on;
    
    % --- Figure 35: Actuating Signal (Control Input) ---
    subplot(5,1,5);
    plot(t_plot, out3.u.Data, 'r-', 'LineWidth', 1.5);
    title('LQR Response: Actuating Control Input u(t) (\alpha_{ref})');
    xlabel('Time [s]'); ylabel('Control Input [deg]');
    xlim(x_limits); legend('\alpha_{ref}'); grid on;
end