syms R1 R2 R3 C1 C2 C3 real

A = [ -1/C1*(1/R1 + 1/R2),   1/(R2*C1),              0;
       1/(R2*C2),           -1/C2*(1/R2 + 1/R3),    1/(R3*C2);
       0,                    1/(R3*C3),            -1/(R3*C3) ];

B = [1/(R1*C1); 
    0; 
    0];

C = [0, 0, 1];

D = 0;

% Plug in value
A_num = double(subs(A, [R1 R2 R3 C1 C2 C3], [4 4 4 1 1 1]));
B_num = double(subs(B, [R1 C1], [4 1]));

%% pole-zero map
sys = ss(A_num, B_num, C, D);
pzmap(sys)
grid on

%% Plot the step response of the system for a step input ue = 1
t = 0:0.1:100; % Time vector
[y, t, x] = step(sys, t);

figure;
plot(t, x(:,1), 'r', t, x(:,2), 'g', t, x(:,3), 'b', t, ones(size(t)), 'k--');
title('Step Response (u_e = 1)');
xlabel('Time');
ylabel('State Variables / Input');
legend('u_1', 'u_2', 'u_3', 'u_e (input)');
grid on;

t = 0:0.1:100; % Time vector
x0_e = [0.5; 0; 0]; 
x0_f = [0; 0.5; 0]; 
x0_g = [0; 0; 0.5]; 

%% e) Unforced response for x0 = [0.5; 0; 0]
[~, ~, x1] = initial(sys, x0_e, t);
figure;
plot(t, x1(:,1), 'r', t, x1(:,2), 'g', t, x1(:,3), 'b');
title('Unforced Response to x_0 = [0.5; 0; 0]');
xlabel('Time');
ylabel('State Variables');
legend('u_1', 'u_2', 'u_3');
grid on;

%% f) Unforced response for x0 = [0; 0.5; 0]
[~, ~, x2] = initial(sys, x0_f, t);
figure;
plot(t, x2(:,1), 'r', t, x2(:,2), 'g', t, x2(:,3), 'b');
title('Unforced Response to x_0 = [0; 0.5; 0]');
xlabel('Time');
ylabel('State Variables');
legend('u_1', 'u_2', 'u_3');
grid on;

%% g) Unforced response for x0 = [0; 0; 0.5]
[~, ~, x3] = initial(sys, x0_g, t);
figure;
plot(t, x3(:,1), 'r', t, x3(:,2), 'g', t, x3(:,3), 'b');
title('Unforced Response to x_0 = [0; 0; 0.5]');
xlabel('Time');
ylabel('State Variables');
legend('u_1', 'u_2', 'u_3');
grid on;