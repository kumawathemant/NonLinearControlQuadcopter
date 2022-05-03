%% Adaptive Project - Quadcopter
addpath(genpath('trajectories'));
%% Drone Parameter Values
%% Adapted from https://digitalcommons.uri.edu/theses/1130/


%% PARAMS 
global Ka Km m Ix Iy Iz g l lambda gamma_x gamma_r gamma_alpha  A_ref B_ref K_ref Kr_nonlin_ctr r_pos Am Bm P;
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
lambda = 0.6; % the parameter in the coord transform


%% Matrices for Am and start 
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
r_pos =  [5, 4 , 1,0,0,0];

% start the simulation
tspan = [0,3];
x0 = zeros(88,1);
x0(15) = -lambda;
x0(85) = -lambda;
temp_K = -K_ref';
x0(19:42) = [temp_K(1,:) temp_K(2,:) temp_K(3,:) temp_K(4,:) temp_K(5,:) temp_K(6,:)]';
x0(43) = 1;
x0(48) = 1;
x0(53) = 1;
x0(58) = 1;

% alpha_hat(0)
temp1 = [-Ka/m , -Ka/m , -Ka/m, -Ka/m;
         0 , -lambda*Ka*l/Ix , 0 , lambda*Ka*l/Ix;
         -lambda*Ka*l/Iy , 0 , lambda*Ka*l/Iy , 0];
temp2 = [lambda , g , 0 , 0 , 0 , 0;
         0 , 0 , lambda*(-Ix+Iy-Iz)/Ix , g , 0 , 0;
         0 , 0 , 0 , 0 , -lambda*(-Ix+Iy+Iz)/Iy , -g];
alpha_star = (temp1\temp2)';
x0(59:82) = [alpha_star(1,:) , alpha_star(2,:) , alpha_star(3,:) , alpha_star(4,:) , alpha_star(5,:) , alpha_star(6,:)]';
%%
[t, x] = ode45(@NCNS, tspan, x0);

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
