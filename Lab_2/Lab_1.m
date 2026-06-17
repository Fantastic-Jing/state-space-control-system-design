clc, clear, close;

%% T1.1 Determination of the state-space model of the system  
J1 = 2e-3;
J2 = 0.4e-3;
J3 = 0.3e-3;

d1 = 8e-3;
d2 = 7.5e-3;
d3 = 7.5e-3;

k12 = 1.5;
k23 = 1.5;

A = [
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    -k12/J1   k12/J1     0      -d1/J1   0        0;
     k12/J2 -(k12+k23)/J2 k23/J2  0     -d2/J2    0;
     0       k23/J3    -k23/J3   0       0      -d3/J3
];


B = [0; 0; 0; 1/J1; 0; 0];

C = [0 0 1 0 0 0];   % output = φ3
D = 0;

sys = ss(A, B, C, D)

%% T1.2 Analysis of the structural properties of the system 

% Calculate the eigenvalues of the system
eigenvalues = eig(A)

% Calculate the controllability matrix
controllabilityMatrix = ctrb(A, B);

% Check the rank of the controllability matrix
rankControllability = rank(controllabilityMatrix)

% Calculate the observability matrix
observabilityMatrix = obsv(A, C);

% Check the rank of the observability matrix
rankObservability = rank(observabilityMatrix)

%% T1.3 Analysis of the dynamical behavior of the system

% a) state responses to initial conditions
t = 0:0.001:1;

X0 = [
    0.05  0     0     0 0 0;   % Case 1
    0     0.05  0     0 0 0;   % Case 2
    0     0     0.05  0 0 0    % Case 3
];

for i = 1:3
    x0 = X0(i,:)';

    % Simulate free response
    [y, t_out, x] = initial(sys, x0, t);

    % angular displacements
    figure;
    plot(t_out, x(:,1)); hold on;
    plot(t_out, x(:,2));
    plot(t_out, x(:,3));
    xlabel('Time [s]');
    ylabel('Angular displacement [rad]');
    title(['Case ', num2str(i), ': Angular displacements']);
    legend('\phi_1','\phi_2','\phi_3');
    grid on;

    % angular speeds
    figure;
    plot(t_out, x(:,4)); hold on;
    plot(t_out, x(:,5));
    plot(t_out, x(:,6));
    xlabel('Time [s]');
    ylabel('Angular speed [rad/s]');
    title(['Case ', num2str(i), ': Angular speeds']);
    legend('\omega_1','\omega_2','\omega_3');
    grid on;
end

%% b) state responses to a step change u = 10Nm
u = 10 * ones(size(t));

x0 = zeros(6,1);

[y, t_out, x] = lsim(sys, u, t, x0);

% angular x1, x2, x3
figure;
xlim([0 2]);
plot(t_out, x(:,1)); hold on;
plot(t_out, x(:,2));
plot(t_out, x(:,3));
xlabel('Time [s]');
ylabel('Angular displacement [rad]');
title('Angular displacements: \phi_1, \phi_2, \phi_3');
legend('\phi_1','\phi_2','\phi_3');
grid on;

% angular velocity x4, x5, x6
figure;
xlim([0 2]);
plot(t, x(:,4)); hold on;
plot(t, x(:,5));
plot(t, x(:,6));
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
title('Angular velocity: \omega_1, \omega_2, \omega_3');
legend('\omega_1','\omega_2','\omega_3');
grid on;

%% T1.4 Simulink model of the plant 

% a) Disturbance matrix F
F = [
    0      0      0;
    0      0      0;
    0      0      0;
    1/J1   0      0;
    0      1/J2   0;
    0      0      1/J3
];

t = 0:0.001:2;    
n = numel(t);
u = timeseries(zeros(n,1),t);             % input u = 0

%% b1) Disturbance case 1: Tz2 = 1 Nm step, others 0

Tz1 = zeros(n,1);          % 0
Tz2 = ones(n,1);           % 1 Nm step
Tz3 = zeros(n,1);          % 0

Z1 = timeseries([Tz1 Tz2 Tz3], t);  % n x 3 disturbance matrix

% Run simulink case 1
% sim('spring_mass_damper_simu.slx');  


%% b1) plot
x = out.x_out.signals.values;  % Simulink state matrix (N × 6)

figure;
plot(out.tout, x(:,1)); hold on;
plot(out.tout, x(:,2));
plot(out.tout, x(:,3));
xlabel('Time [s]');
ylabel('Angular displacement [rad]');
title('Case 1: Step disturbance Tz2 = 1 Nm');
legend('\phi_1','\phi_2','\phi_3');
grid on;

figure;
plot(out.tout, x(:,4)); hold on;
plot(out.tout, x(:,5));
plot(out.tout, x(:,6));
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
title('Case 1: Angular velocities');
legend('\omega_1','\omega_2','\omega_3');
grid 

%% b2) Disturbance case 2: Tz3 = 1 Nm step, others 0
Tz1 = zeros(n,1);          % 0
Tz2 = zeros(n,1);          % 0
Tz3 = ones(n,1);           % 1 Nm step

Z2 = timeseries([Tz1 Tz2 Tz3], t);  % n x 3 disturbance matrix

% Run simulink case 2
% sim('spring_mass_damper_simu.slx');  

%% b1) plot
x = out.x_out.signals.values;  % Simulink state matrix (N × 6)

figure;
plot(out.tout, x(:,1)); hold on;
plot(out.tout, x(:,2));
plot(out.tout, x(:,3));
xlabel('Time [s]');
ylabel('Angular displacement [rad]');
title('Case 2: Step disturbance Tz3 = 1 Nm');
legend('\phi_1','\phi_2','\phi_3');
grid on;

figure;
plot(out.tout, x(:,4)); hold on;
plot(out.tout, x(:,5));
plot(out.tout, x(:,6));
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
title('Case 1: Angular velocities');
legend('\omega_1','\omega_2','\omega_3');
grid 

%% 2.2 Design of a state feedback control 

p = [-8, -10, -30, -40, -50, -60];   % desired closed-loop poles
K = place(A, B, p);

Acl = A - B*K;
V = -1 / (C * (Acl \ B));

t = (0:0.001:2)';    
n = length(t);
w = timeseries(ones(n,1), t);  

% Pre-allocate disturbance signals
zeroSig = zeros(n,1);

%%
cases = {
    % Case 1: reference step w=50, no disturbances
    struct('name','Case 1: w=50 rad, no disturbances', ...
           'w', 50*ones(n,1), ...
           'Tz1', zeroSig, 'Tz2', zeroSig, 'Tz3', zeroSig)

    % Case 2: Tz1 step
    struct('name','Case 2: Tz1=1 Nm', ...
           'w', zeroSig, ...
           'Tz1', ones(n,1), 'Tz2', zeroSig, 'Tz3', zeroSig)

    % Case 3: Tz2 step
    struct('name','Case 3: Tz2=1 Nm', ...
           'w', zeroSig, ...
           'Tz1', zeroSig, 'Tz2', ones(n,1), 'Tz3', zeroSig)

    % Case 4: Tz3 step
    struct('name','Case 4: Tz3=1 Nm', ...
           'w', zeroSig, ...
           'Tz1', zeroSig, 'Tz2', zeroSig, 'Tz3', ones(n,1))

    % Case 5: Tz2=0.5, Tz3=1
    struct('name','Case 5: Tz2=0.5 Nm, Tz3=1 Nm', ...
           'w', zeroSig, ...
           'Tz1', zeroSig, 'Tz2', 0.5*ones(n,1), 'Tz3', ones(n,1))
};

%% 
for k = 1:length(cases)
    Ck = cases{k};

    % --- Prepare signals for Simulink ---
    w_in  = timeseries(Ck.w,  t);
    Z_in  = timeseries([Ck.Tz1 Ck.Tz2 Ck.Tz3], t);

    % --- Run Simulink ---
    sim('spring_mass_damper_simu.slx');

    % --- Extract outputs ---
    x = out.x_out.signals.values;   % Nx6
    u = out.u_out.signals.values;   % Nx1
    tt = out.tout;

    % --- Plotting ---
    figure('Name',Ck.name,'NumberTitle','off');

    % Subplot 1: angular displacements
    subplot(3,1,1);
    plot(tt, x(:,1), tt, x(:,2), tt, x(:,3));
    ylabel('\phi [rad]');
    title([Ck.name ' — Angular Displacements']);
    legend('\phi_1','\phi_2','\phi_3');
    grid on;

    % Subplot 2: angular velocities
    subplot(3,1,2);
    plot(tt, x(:,4), tt, x(:,5), tt, x(:,6));
    ylabel('\omega [rad/s]');
    title('Angular Velocities');
    legend('\omega_1','\omega_2','\omega_3');
    grid on;

    % Subplot 3: control input u(t)
    subplot(3,1,3);
    plot(tt, u);
    xlabel('Time [s]');
    ylabel('u(t) [Nm]');
    title('Control Input');
    grid on;
end

