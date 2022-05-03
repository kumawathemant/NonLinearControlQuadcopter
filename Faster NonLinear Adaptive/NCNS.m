function xdot = NCNS(t,x)
disp(t);
% x: [x:12*1 xm:6*1 kx:6*4 kr:4*4 alpha:6*4 x_prime:6*1]
global lambda Am Bm P Kr_nonlin_ctr gamma_x gamma_r gamma_alpha r_pos Ka Km m Ix Iy Iz g l m_act l_act;

x = reshape(x,[1,88]);
x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4); x5 = x(5); x6 = x(6);
x7 = x(7); x8 = x(8); x9 = x(9); x10 = x(10); x11 = x(11); x12 = x(12);
x_prime = reshape(x(83:88),[6,1]);

% transform the desired ref states r from the original
% coordinates to the new transformed coordinates (prime)


R = rsteps(t)
r_prime = r_pos';
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
% The dynamics for x_prime
A = [zeros(3,3) , eye(3) ; zeros(3,3) , zeros(3,3)];
B = [zeros(3,3) ; R];
T = Ka * sum(ut);
tau_x = Ka*l * (ut(4) - ut(2));
tau_y = Ka*l * (ut(1) - ut(3));
tau_z = Km * (ut(1) - ut(2) + ut(3) - ut(4));
f = [-T/m + lambda*(x10*x10+x11*x11) + cos(x4)*cos(x5)*g;
     lambda*((-Ix+Iy-Iz)*x11*x12/Ix + tau_x/Ix) + sin(x4)*cos(x5)*g;
     -lambda*((-Ix+Iy+Iz)*x10*x12/Iy + tau_y/Iy) + sin(x5)*g];
x_prime_dot = A*x(83:88)' + B*f;
xdot(83:88) = x_prime_dot;

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



end