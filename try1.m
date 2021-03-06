%% Adaptive Project - Quadcopter
addpath(genpath('trajectories'));
%% Drone Parameter Values
%close all; clear all;
global Ka Km m Ix Iy Iz g l;
Ct = 0.0107;
Cq = Ct*sqrt(Ct/2);
Rr = 33/1000;    % rotor radius
RA = pi*Rr^2;     % rotor radius
rho = 1.184;    % density of air
Ka = Ct*rho*RA*Rr^2;
Km = Cq*rho*RA*Rr^3;
g = 9.81;
Ix = 0.0686e-3;
Iy = 0.092e-3;
Iz = 0.1366e-3;
l = 0.0624;        %Distance from rotor to the center of Drone
m = 0.068;

%% Linearized System arround the equilibrium point (Hover)
global K A B;

O6 = zeros(6);
I6 = eye(6);
Psi = 0;    % Phi angule chosen as Eq point at the hover.

Phi = [0, 0, 0, -g*sin(Psi), -g*cos(Psi), 0;
       0, 0, 0, g*cos(Psi), -g*sin(Psi), 0;
       0, 0, 0, 0,           0,          0;
       0, 0, 0, 0,           0,          0;
       0, 0, 0, 0,           0,          0;
       0, 0, 0, 0,           0,          0];
   
A = [O6,  I6;       % Jacobian Matrix A
     Phi, O6];
 
Ac = [Phi, O6];     % Part of A that is Controllable

O84 = zeros(8,4);

Delta = [Ka/m,      Ka/m,           Ka/m,       Ka/m;
         0,         -Ka*l/Ix,       0,          Ka*l/Ix;
         Ka*l/Iy,   0,              -Ka*l/Iy,   0;
         Km/Iz,     -Km/Iz,         Km/Iz,      -Km/Iz];
       
B = [O84;       % Jacobian Matrix B
     Delta];
global m_act l_act 
%% Model Mismatch (the actual plant)
global A_act B_act
A_act = A;
m_act = 1.5 * m;
l_act = 1.5 * l;
Delta_act = [   Ka/m_act,     Ka/m_act,     Ka/m_act,    Ka/m_act;
               0, -Ka*l_act/Ix,        0, Ka*l_act/Ix;
         Ka*l_act/Iy,        0, -Ka*l_act/Iy,       0;
           Km/Iz,   -Km/Iz,    Km/Iz,  -Km/Iz];

B_act = [O84;
         Delta_act];
 
%% Pole Placement
global Kr_lin_ctr;

desired_poles = -linspace(1,12,12);
K = place(A,B,desired_poles);
Kr_lin_ctr = B\(A-B*K);

% Our calculated Gain K to stabilize the system.
array2table(K)

% Eigenvalues of the closed loop system.
EigenValues = array2table(eig(A - B*K))

%% Set the desired State Values (Reference Signal that the 12 states should track)
global r;
r = [1,1,1,0,0,0,0,0,0,0,0,0]';     % the desired state values



%% Nonlinear Adaptive Controller & Nonlinear Plant
global lambda gamma_x gamma_r gamma_alpha;
lambda = 0.6; % the parameter in the coord transform

% Some Preparation
global A_ref B_ref K_ref Kr_nonlin_ctr r_pos Am Bm P;
A_ref = [zeros(3,3) , eye(3) ; zeros(3,3) , zeros(3,3)];
B_ref = zeros(6,4);
B_ref(4,1) = -lambda*Ka*l/Iy;
B_ref(4,3) = lambda*Ka*l/Iy;
B_ref(5,2) = -lambda*Ka*l/Ix;
B_ref(5,4) = lambda*Ka*l/Ix;
B_ref(6,:) = [-Ka/m , -Ka/m, -Ka/m, -Ka/m];
desired_poles = [-1,-2,-2,-1,-2,-1];       %linspace(5,10,6);
K_ref = place(A_ref,B_ref,desired_poles);
Kr_nonlin_ctr = B_ref\(A_ref-B_ref*K_ref);

% the reference model
Am = A_ref - B_ref*K_ref;
Bm = B_ref;
% matrix Q , P
Q = 500*eye(6);
P = lyap(Am',Q);
% adaptation rates
gamma_x = [100*eye(3) 100*eye(3); 500*eye(3) 500*eye(3)];
gamma_r = 100*eye(4);
gamma_alpha = [1000*eye(3) 1000*eye(3); 5000*eye(3) 5000*eye(3)];

% The desired values for the 6 position states
% x1,x2,x3,x7,x8,x9 (in the original coords)
r_pos = [1;1;1;0;0;0];

% start the simulation
tspan = [0,25];
x0 = zeros(88,1);
% initial conditions: x(0) = xm(0) = 0
% convert xm(0) to the new transformed coordinates
x0(15) = -lambda;
x0(85) = -lambda;
% use the position state dynamics in the new coords to obtain kx(0) and kr(0)
temp_K = -K_ref';
x0(19:42) = [temp_K(1,:) temp_K(2,:) temp_K(3,:) temp_K(4,:) temp_K(5,:) temp_K(6,:)]';
x0(43) = 1;
x0(48) = 1;
x0(53) = 1;
x0(58) = 1;

% calculate alpha_hat(0)
temp1 = [-Ka/m , -Ka/m , -Ka/m, -Ka/m;
         0 , -lambda*Ka*l/Ix , 0 , lambda*Ka*l/Ix;
         -lambda*Ka*l/Iy , 0 , lambda*Ka*l/Iy , 0];
temp2 = [lambda , g , 0 , 0 , 0 , 0;
         0 , 0 , lambda*(-Ix+Iy-Iz)/Ix , g , 0 , 0;
         0 , 0 , 0 , 0 , -lambda*(-Ix+Iy+Iz)/Iy , -g];
alpha_star = (temp1\temp2)';
x0(59:82) = [alpha_star(1,:) , alpha_star(2,:) , alpha_star(3,:) , alpha_star(4,:) , alpha_star(5,:) , alpha_star(6,:)]';
%%
[t, x] = ode45(@AC_NonlinearModel, tspan, x0);

%% Plots
xt = x(:,1:12);
xm = x(:,13:18);
x_prime = x(:,83:88);

figs(1)  = figure ;

l = tiledlayout('flow');
r = '';
s = '';
r = 'r_{rsteps}';
a = 'adaptive';

title(l,['Quadcopter Trajectory for Non Linear system  (',r,s,')'])
                
l = nexttile;
title(l,'Position')
hold on
plot(t,x(:,83));
plot(t,x(:,84));
plot(t,x(:,85));
plot(t,x(:,13),'--');
plot(t,x(:,14),'--');
plot(t,x(:,15),'--');
ylabel('Magnitude (m)');
xlabel('Time (s)');
hold off
grid on
legend('x','y','z','x_m','y_m','z_m','Interperter','latex');

l = nexttile;
title(l,'Position Error')
hold on
plot(t,x(:,83)-x(:,13));
plot(t,x(:,84)-x(:,14));
plot(t,x(:,85)-x(:,15));
ylabel('Magnitude (m)');
xlabel('Time (s)');
hold off
grid on
legend('x-x_m','y-y_m','z-z_m','Interperter','latex');



l = nexttile;
title(l,'Kx')
hold on
plot(t,x(:,19:42));

ylabel('Magnitude');
xlabel('Time (s)');
hold off
grid on


l = nexttile;
title(l,'Kr')
hold on
plot(t,x(:,43:48));
ylabel('Magnitude');
xlabel('Time (s)');
hold off
grid on


figure;
hold on;
plot3(x(:,83),x(:,84),x(:,85),'r')
plot3(x(:,13),x(:,14),x(:,15),'b--')
plot3(x(end,83),x(end,84),x(end,85),'r*')
plot3(x(end,13),x(end,14),x(end,15),'b*')

title(['Quadcopter Trajectory for non linear system (',r,s,')'])
legend('Mismatched','Model','Mismatched Final Position','Model Final Position')
view(3)
zlabel('y (m)','Interpreter','latex');
ylabel('z (m)','Interpreter','latex');
xlabel('x (m)','Interpreter','latex');
hold off;

                








%% Nonlinear Adaptive Controller (MRAC MIMO, nonlinear plant)
function xdot = AC_NonlinearModel(t,x)
disp(t);
% x: [x:12*1 xm:6*1 kx:6*4 kr:4*4 alpha:6*4 x_prime:6*1]
global lambda Am Bm P Kr_nonlin_ctr gamma_x gamma_r gamma_alpha r_pos Ka Km m Ix Iy Iz g l m_act l_act;
% l = l_act;
% m = m_act;

x = reshape(x,[1,88]);
x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4); x5 = x(5); x6 = x(6);
x7 = x(7); x8 = x(8); x9 = x(9); x10 = x(10); x11 = x(11); x12 = x(12);
x_prime = reshape(x(83:88),[6,1]);

% transform the desired ref states r from the original
% coordinates to the new transformed coordinates (prime)

r_prime = [4*cos(3*t), -sin(t), 5*sin(2*t),0,0,0]';
R = rsteps(t)
r_prime = [3, R(1) , 2,0,R(2),0]';
% r_prime = [5, 4 , 1,0,0,0]'
rt_prime = -Kr_nonlin_ctr * r_prime;
%rt = -Kr_lin_ctr * r;

% The Phi(x) matrix that parametrizes the nonlinearity
phi_x = [x10*x10 + x11*x11;
         cos(x4) * cos(x5);
         x11 * x12;
         sin(x4) * cos(x5);
         x10 * x12;
         sin(x5)];

xt = reshape(x(1:12),[12,1]);
xm = reshape(x(13:18),[6,1]);
Kx = [x(19:22); x(23:26); x(27:30); x(31:34); x(35:38); x(39:42)];
Kr = [x(43:46); x(47:50); x(51:54); x(55:58)];
alpha_hat = [x(59:62); x(63:66); x(67:70); x(71:74) ; x(75:78) ; x(79:82)];

e_prime = x_prime - xm;

% The B_prime matrix for the plant in the transformed coords
R = [cos(x4)*sin(x5)*cos(x6)+sin(x4)*sin(x6) , sin(x4)*sin(x5)*cos(x6)-cos(x4)*sin(x6) , cos(x5)*cos(x6);
     cos(x4)*sin(x5)*sin(x6)-sin(x4)*cos(x6) , sin(x4)*sin(x5)*sin(x6)+cos(x4)*cos(x6) , cos(x5)*sin(x6);
     cos(x4)*cos(x5) , sin(x4)*cos(x5) , -sin(x5)];
Ra = R(1,1); Rb = R(1,2); Rc = R(1,3); Rd = R(2,1); Re = R(2,2); Rf = R(2,3); Rg = R(3,1); Rh = R(3,2); Ri = R(3,3); 
Rstar = [-Ka*Ra/m - lambda*Ka*l*Rc/Iy , -Ka*Ra/m - lambda*Ka*l*Rb/Ix , -Ka*Ra/m + lambda*Ka*l*Rc/Iy , -Ka*Ra/m + lambda*Ka*l*Rb/Ix;
         -Ka*Rd/m - lambda*Ka*l*Rf/Iy , -Ka*Rd/m - lambda*Ka*l*Re/Ix , -Ka*Rd/m + lambda*Ka*l*Rf/Iy , -Ka*Rd/m + lambda*Ka*l*Re/Ix;
         -Ka*Rg/m - lambda*Ka*l*Ri/Iy , -Ka*Rg/m - lambda*Ka*l*Rh/Ix , -Ka*Rg/m + lambda*Ka*l*Ri/Iy , -Ka*Rg/m + lambda*Ka*l*Rh/Ix];
B_prime = [zeros(3,4) ; Rstar];

% The control output u(t)
ut = Kx' * x_prime + Kr' * rt_prime - alpha_hat' * phi_x;
% The adaptation laws
Kx_dot = -gamma_x * x_prime * e_prime' * P * B_prime;
Kr_dot = -gamma_r * rt_prime * e_prime' * P * B_prime;
Kalpha_dot = gamma_alpha * phi_x * e_prime' * P * B_prime;

xdot = zeros(88,1);
% state update for xm (in the transformed coords)
xdot(13:18) = Am*xm + Bm*rt_prime;
% state update for Kx_prime
xdot(19:42) = [Kx_dot(1,:) , Kx_dot(2,:) , Kx_dot(3,:) , Kx_dot(4,:) , Kx_dot(5,:) , Kx_dot(6,:)]';
xdot(43:58) = [Kr_dot(1,:) , Kr_dot(2,:) , Kr_dot(3,:) , Kr_dot(4,:)]';
xdot(59:82) = [Kalpha_dot(1,:) , Kalpha_dot(2,:) , Kalpha_dot(3,:) , Kalpha_dot(4,:) , Kalpha_dot(5,:) , Kalpha_dot(6,:)]';

% The actual nonlinear dynamics for updating x(t) (in the original coords)
T = Ka * sum(ut);
tau_x = Ka*l * (ut(4) - ut(2));
tau_y = Ka*l * (ut(1) - ut(3));
tau_z = Km * (ut(1) - ut(2) + ut(3) - ut(4));
xdot(1) = x7;
xdot(2) = x8;
xdot(3) = x9;
xdot(4) = x10 + sin(x4)*tan(x5)*x11 + cos(x4)*tan(x5)*x12;
xdot(5) = cos(x4)*x11 - sin(x4)*x12;
xdot(6) = sin(x4)*x11/cos(x5) + cos(x4)*x12/cos(x5);
xdot(7) = -T * (cos(x4)*sin(x5)*cos(x6) + sin(x4)*sin(x6)) / m;
xdot(8) = -T * (cos(x4)*sin(x5)*sin(x6) - sin(x4)*cos(x6)) / m;
xdot(9) = -T * cos(x4) * cos(x5) / m + g;
xdot(10) = (Iy - Iz)*x11*x12/Ix + tau_x/Ix;
xdot(11) = (-Ix + Iz)*x10*x12/Iy + tau_y/Iy;
xdot(12) = (Ix - Iy)*x10*x11/Iz + tau_z/Iz;

% The dynamics for x_prime
A = [zeros(3,3) , eye(3) ; zeros(3,3) , zeros(3,3)];
B = [zeros(3,3) ; R];
f = [-T/m + lambda*(x10*x10+x11*x11) + cos(x4)*cos(x5)*g;
     lambda*((-Ix+Iy-Iz)*x11*x12/Ix + tau_x/Ix) + sin(x4)*cos(x5)*g;
     -lambda*((-Ix+Iy+Iz)*x10*x12/Iy + tau_y/Iy) + sin(x5)*g];
x_prime_dot = A*x(83:88)' + B*f;
xdot(83:88) = x_prime_dot;

end