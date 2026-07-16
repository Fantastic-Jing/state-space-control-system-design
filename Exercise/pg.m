%% ================================
%  System definition (a = 1)
% ================================
A = [ 0   1   0;
      0  -2   2;
     -4  -3  -1 ];

B = [0; 0; 1];     % a = 1
C = [3 0 0];
D = 0;

%% ================================
%  Desired closed-loop poles
% ================================
p_des = [-2 -15 -15.01];   % dominant pole + two fast poles

%% ================================
%  State feedback K (pole placement)
% ================================
K = place(A, B, p_des);

%% ================================
%  (d) Closed-loop transfer function G_closed(s)
%      w → y  (with prefilter V added later)
% ================================
% Closed-loop system without prefilter
Acl = A - B*K;
Bcl = B;   % reference enters through B (prefilter added later)

sys_cl = ss(Acl, Bcl, C, D);
G_cl = tf(sys_cl)

disp('Closed-loop poles:')
disp(pole(sys_cl))

%% ================================
%  (e) Prefilter V to remove steady-state error
% ================================
dc_gain = dcgain(G_cl);
V = 1/dc_gain;     % prefilter

disp('Prefilter V:')
disp(V)

% Final closed-loop transfer function with prefilter
G_closed = V * G_cl

%% ================================
%  (f) Observer poles
% ================================
L = [49; 1728.3; 9505.2];   % given observer gain

Aobs = A - L*C;
p_obs = eig(Aobs);

disp('Observer poles:')
disp(p_obs)

%% Reasonableness check
disp('Observer pole placement is reasonable if poles are much faster (more negative)')
disp('than closed-loop poles (-2, -15, -15).')
