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
plot(t, x(:,4)); hold on;
plot(t, x(:,5));
plot(t, x(:,6));
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
title('Angular velocity: \omega_1, \omega_2, \omega_3');
legend('\omega_1','\omega_2','\omega_3');
grid on;

%% T1.4 Simulink model of the plant 





