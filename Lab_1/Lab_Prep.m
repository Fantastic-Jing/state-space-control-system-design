%% Task b state-space model
R1 = 4;  R2 = 4;  R3 = 4;
C1 = 1;  C2 = 1;  C3 = 1;

A = [ -(1/(R1*C1) + 1/(R2*C1)),   1/(R2*C1),                 0;
       1/(R2*C2),               -(1/(R2*C2) + 1/(R3*C2)),    1/(R3*C2);
       0,                         1/(R3*C3),               -(1/(R3*C3)) ];

b = [1/(R1*C1); 0; 0]; 

C = [0, 0, 1];

d = 0;

sys = ss(A, b, C, d);

%% Task c Pole-Zero Map

figure;
pzmap(sys);
title('Pole-Zero Map of 3-Level RC Network');
grid on;

%% Task d Step response (ue = 1)

% Time vector
t = 0:0.1:50;

% Step input ue = 1
u = ones(size(t));

% Simulate system response
[y, t, x] = lsim(sys, u, t);

% Plot
figure;
plot(t, x(:,1)); hold on;
plot(t, x(:,2));
plot(t, x(:,3));
plot(t, u);   % input ue

grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Step Response of 3-Level RC Network (ue = 1)');
legend('u_1(t)', 'u_2(t)', 'u_3(t)', 'u_e(t)');

%% Unforced response for initial condition x0 = (0.5, 0, 0)^T

t = 0:0.1:50;          % simulation time
x0 = [0.5; 0; 0];      % initial condition
u = zeros(size(t));    % no input (unforced)

[y_free, t_free, x_free] = lsim(sys, u, t, x0);

figure;
plot(t_free, x_free(:,1)); hold on;
plot(t_free, x_free(:,2));
plot(t_free, x_free(:,3));

grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Unforced Response of 3-Level RC Network (x_0 = [0.5, 0, 0]^T)');
legend('u_1(t)', 'u_2(t)', 'u_3(t)');

%% Unforced response for initial condition x0 = (0, 0.5, 0)^T

t = 0:0.1:50;          % simulation time
x0 = [0; 0.5; 0];      % initial condition
u = zeros(size(t));    % no input (unforced)

[y_free, t_free, x_free] = lsim(sys, u, t, x0);

figure;
plot(t_free, x_free(:,1)); hold on;
plot(t_free, x_free(:,2));
plot(t_free, x_free(:,3));

grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Unforced Response of 3-Level RC Network (x_0 = [0.5, 0, 0]^T)');
legend('u_1(t)', 'u_2(t)', 'u_3(t)');

%% Unforced response for initial condition x0 = (0, 00, 0.5)^T

t = 0:0.1:50;          % simulation time
x0 = [0; 0; 0.5];      % initial condition
u = zeros(size(t));    % no input (unforced)

[y_free, t_free, x_free] = lsim(sys, u, t, x0);

figure;
plot(t_free, x_free(:,1)); hold on;
plot(t_free, x_free(:,2));
plot(t_free, x_free(:,3));

grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Unforced Response of 3-Level RC Network (x_0 = [0.5, 0, 0]^T)');
legend('u_1(t)', 'u_2(t)', 'u_3(t)');

