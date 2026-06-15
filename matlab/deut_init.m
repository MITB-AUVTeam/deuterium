clear; clc;

%% ================= PHYSICAL PARAMETERS =================
Ix = 0.2025;
Iy = 0.04457;

xf = 0.22;
xb = 0.28;
y1 = 0.225;

m  = 20;
g  = 9.8;
Fb = 500;

dphi   = 0.02;
dtheta = 0.03;
dz     = 1;

%% ================= INITIAL CONDITIONS =================
phi_init   = pi/4;
wx_init    = 1;
theta_init = pi/4;
wy_init    = 1;
z_init     = 1;
dz_init    = 1;

Ts = 0.01;   % 100 Hz

%% ================= THRUSTER MAPPING =================
% [tau_phi; tau_theta; Fz] = FtoX * [F1; F2; F3]
FtoX = [ 0   -y1   y1;
        -xf   xb   xb;
         1    1    1];

XtoF = inv(FtoX);

%% ================= FULL STATE-SPACE MODEL =================
% x = [phi; wx; theta; wy; z; dz]

A = [ 0      1      0      0      0      0;
      0  -dphi/Ix   0      0      0      0;
      0      0      0      1      0      0;
      0      0      0  -dtheta/Iy 0      0;
      0      0      0      0      0      1;
      0      0      0      0      0  -dz/m ];

B = zeros(6,3);
B(2,:) = (1/Ix) * FtoX(1,:);   % roll
B(4,:) = (1/Iy) * FtoX(2,:);   % pitch
B(6,:) = (1/m)  * FtoX(3,:);   % vertical

C = eye(6);
D = zeros(6,3);

%% ================= INNER LQR (TORQUE SPACE) =================
% x = [wx; wy]
% u = [tau_phi; tau_theta]

A_red = [ -dphi/Ix   0;
           0      -dtheta/Iy ];

B_tau = [1/Ix   0;
         0   1/Iy];

Q_red = diag([10 10]);
R_red = diag([100 100]);

K_tau = lqr(A_red, B_tau, Q_red, R_red);

%% ================= LIMITS =================
TAU_MAX = 2.0;      % torque saturation
FZ_MAX  = 200;      % vertical force limit

THRUST_SCALE = 150;

%% ================= TRIM (HOVER CONDITION) =================
Fz_eq = m*g - Fb;

% convert equilibrium force to thrusters
u_trim = XtoF * [0; 0; Fz_eq];

%% ================= FILTERING =================
beta = 0.2;   % smoothing for LQR output

%% ================= DISPLAY =================
disp('A matrix:'); disp(A);
disp('B matrix:'); disp(B);

disp('Torque LQR gain (K_tau):');
disp(K_tau);

disp('Trim thrust (u_trim):');
disp(u_trim);