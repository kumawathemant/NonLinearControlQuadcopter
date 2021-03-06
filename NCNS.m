function xdot = NCNS(t,X,args)
    addpath(genpath('dynamics'))
    disp(t);
    X = X;
%% System Parameters
    A = args.A;
    B = args.B;
    Am = args.Am;
    Bm = args.Bm;
    A0 = args.A0;
    B0 = args.B0;
    P = args.P;
    K = args.K;
    m = args.Quadcopter.mass;
    I = args.Quadcopter.moments;
    Ix = I(1);
    Iy = I(2);
    Iz = I(3);

%     g = args.gamma;
       g = 9.81;
% 
%     dx = [];
%     dx_m = [];
%     dk_x = [];
%     dk_r = [];

%% Reference States
r = [0 0 0,... % Pos
     0 0 0, ... % Angle
     0 0 0,...  % Vel
     0 0 0]; % Angular Vel
    try 
        switch args.Reference
            case 1
                r(1:3) = [scalar(t,5), scalar(t,1), scalar(t,4)]; 
            case 2
                [R,Dr] = sinusoidal(t,4,3,pi/2); 
                r(1) = R;
                r(7) = Dr;
                [R,Dr] = sinusoidal(t,-1,1,0); 
                r(2) = R;
                r(8) = Dr;
                [R,Dr] = sinusoidal(t,5,2,0);
                r(3) = R;
                r(9) = Dr;
            case 3
                R = 3;
                r(1) = R;
                R = rsteps(t);
                r(2) = R(1);
                r(8) = R(2);
                R = 2;
                r(3) = R;
        end
                
    catch ERR
        display(ERR);
    end
    
%% System Initialization Params;
    lambda = args.system.env.lambda;
    A_ref = args.system.env.A_ref;
    B_ref = args.system.env.B_ref;
    K_ref = args.system.env.K_ref;
    Kr_nonlin_ctr =  args.system.env.Kr_nonlin_ctr;
    Am = args.system.env.Am;
    Bm = args.system.env.Bm;
    Q = args.system.env.Q;
    P = args.system.env.P;
    gamma_x = args.system.env.gamma_x;
    gamma_r = args.system.env.gamma_r;
    gamma_alpha = args.system.env.gamma_alpha;
    x0 = args.system.env.x0;
    l = args.system.env.l;
    m = args.system.env.m;
    Ix = args.system.env.Ix;
    Iy = args.system.env.Iy;
    Iz = args.system.env.Iz;
    Ka = args.system.env.Ka;
    Km = args.system.env.Km;

    
%% States
    x = reshape(X,[1,88]);
    x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4); x5 = x(5); x6 = x(6);
    x7 = x(7); x8 = x(8); x9 = x(9); x10 = x(10); x11 = x(11); x12 = x(12);
    xt = reshape(x(1:12),[12,1]);
    xm = reshape(x(13:18),[6,1]);
    Ka = args.system.env.Ka;
    Km = args.system.env.Km;
    Kx = [x(19:22); x(23:26); x(27:30); x(31:34); x(35:38); x(39:42)];
    Kr = [x(43:46); x(47:50); x(51:54); x(55:58)];
    alpha_hat = [x(59:62); x(63:66); x(67:70); x(71:74) ; x(75:78) ; x(79:82)];
    x_prime = reshape(x(83:88),[6,1]);
    r_prime = [5, 4 , 1,0,0,0]';
    rt_prime = -Kr_nonlin_ctr * r_prime;

    % Non Linearity
    phi_x = [x10*x10 + x11*x11;
         cos(x4) * cos(x5);
         x11 * x12;
         sin(x4) * cos(x5);
         x10 * x12;
         sin(x5)];

    % Transformed Matrices Law
    R = [cos(x4)*sin(x5)*cos(x6)+sin(x4)*sin(x6) , sin(x4)*sin(x5)*cos(x6)-cos(x4)*sin(x6) , cos(x5)*cos(x6);
     cos(x4)*sin(x5)*sin(x6)-sin(x4)*cos(x6) , sin(x4)*sin(x5)*sin(x6)+cos(x4)*cos(x6) , cos(x5)*sin(x6);
     cos(x4)*cos(x5) , sin(x4)*cos(x5) , -sin(x5)];
    Ra = R(1,1); Rb = R(1,2); Rc = R(1,3); Rd = R(2,1); Re = R(2,2); Rf = R(2,3); Rg = R(3,1); Rh = R(3,2); Ri = R(3,3); 
    Rstar = [-Ka*Ra/m - lambda*Ka*l*Rc/Iy , -Ka*Ra/m - lambda*Ka*l*Rb/Ix , -Ka*Ra/m + lambda*Ka*l*Rc/Iy , -Ka*Ra/m + lambda*Ka*l*Rb/Ix;
             -Ka*Rd/m - lambda*Ka*l*Rf/Iy , -Ka*Rd/m - lambda*Ka*l*Re/Ix , -Ka*Rd/m + lambda*Ka*l*Rf/Iy , -Ka*Rd/m + lambda*Ka*l*Re/Ix;
             -Ka*Rg/m - lambda*Ka*l*Ri/Iy , -Ka*Rg/m - lambda*Ka*l*Rh/Ix , -Ka*Rg/m + lambda*Ka*l*Ri/Iy , -Ka*Rg/m + lambda*Ka*l*Rh/Ix];
    B_prime = [zeros(3,4) ; Rstar];
    A = [zeros(3,3) , eye(3) ; zeros(3,3) , zeros(3,3)];
    B = [zeros(3,3) ; R];
    




    %% LAWS
    e_prime = x_prime - xm; 
    ut = Kx' * x_prime + Kr' * rt_prime - alpha_hat' * phi_x;
    Kx_dot = -gamma_x * x_prime * e_prime' * P * B_prime;
    Kr_dot = -gamma_r * rt_prime * e_prime' * P * B_prime;
    Kalpha_dot = gamma_alpha * phi_x * e_prime' * P * B_prime;
    T = Ka * sum(ut);
    tau_x = Ka*l * (ut(4) - ut(2));
    tau_y = Ka*l * (ut(1) - ut(3));
    tau_z = Km * (ut(1) - ut(2) + ut(3) - ut(4));
    f = [-T/m + lambda*(x10*x10+x11*x11) + cos(x4)*cos(x5)*g;
         lambda*((-Ix+Iy-Iz)*x11*x12/Ix + tau_x/Ix) + sin(x4)*cos(x5)*g;
         -lambda*((-Ix+Iy+Iz)*x10*x12/Iy + tau_y/Iy) + sin(x5)*g];
    x_prime_dot = A*x(83:88)' + B*f;

    %% xdot
    xdot = zeros(88,1);
%     xdot(1:12) = nonlinear_dyn(x,ut,args);
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
    xdot(13:18) = Am*xm + Bm*rt_prime; %% Xm
    xdot(19:42) = [Kx_dot(1,:) , Kx_dot(2,:) , Kx_dot(3,:) , Kx_dot(4,:) , Kx_dot(5,:) , Kx_dot(6,:)]'; % Kx
    xdot(43:58) = [Kr_dot(1,:) , Kr_dot(2,:) , Kr_dot(3,:) , Kr_dot(4,:)]'; %Kr
    xdot(59:82) = [Kalpha_dot(1,:) , Kalpha_dot(2,:) , Kalpha_dot(3,:) , Kalpha_dot(4,:) , Kalpha_dot(5,:) , Kalpha_dot(6,:)]';
    xdot(83:88) = x_prime_dot; %x_prime

    
    
end