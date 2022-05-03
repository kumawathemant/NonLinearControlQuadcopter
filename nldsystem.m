function [nld] = nldsystem()
    nld = struct;
    nld.env = struct;
    
    % Refer to  https://digitalcommons.uri.edu/theses/1130/
    % for more defination of variables
    lambda = 0.6;
    nld.env.lambda = 0.6; % the parameter in the coord transform 

    % Environment Variables
    % COMMENT IF YOU Wanna use original variables
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
    nld.env.Ka = Ct*rho*RA*Rr^2;
    nld.env.Km = Cq*rho*RA*Rr^3;
    nld.env.g = 9.81;
    nld.env.Ix = 0.0686e-3;
    nld.env.Iy = 0.092e-3;
    nld.env.Iz = 0.1366e-3;
    nld.env.l = 0.0624;        %Distance from rotor to the center of Drone
    nld.env.m = 0.068;

    %% Define the matrices;
    nld.env.A_ref =  [zeros(3,3) , eye(3) ; zeros(3,3) , zeros(3,3)];
    nld.env.B_ref = zeros(6,4);
    nld.env.B_ref(4,1) = -lambda*Ka*l/Iy;
    nld.env.B_ref(4,3) = lambda*Ka*l/Iy;
    nld.env.B_ref(5,2) = -lambda*Ka*l/Ix;
    nld.env.B_ref(5,4) = lambda*Ka*l/Ix;
    nld.env.B_ref(6,:) = [-Ka/m , -Ka/m, -Ka/m, -Ka/m];

    desired_poles = [-1,-2,-2,-1,-2,-1];       %linspace(5,10,6);
    nld.env.K_ref = place(nld.env.A_ref,nld.env.B_ref,desired_poles);
    nld.env.Kr_nonlin_ctr = nld.env.B_ref\(nld.env.A_ref-nld.env.B_ref*nld.env.K_ref);
    
    %% model system
    nld.env.Am = nld.env.A_ref - nld.env.B_ref*nld.env.K_ref;
    nld.env.Bm = nld.env.B_ref;
    nld.env.Q = 500*eye(6);
    nld.env.P = lyap(nld.env.Am',nld.env.Q);
    % adaptation rates
    nld.env.gamma_x = [100*eye(3) 100*eye(3); 500*eye(3) 500*eye(3)];
    nld.env.gamma_r = 100*eye(4);
    nld.env.gamma_alpha = [1000*eye(3) 1000*eye(3); 5000*eye(3) 5000*eye(3)];
    
    
    
    nld.env.x0 = zeros(88,1);
    % initial conditions: x(0) = xm(0) = 0
    % convert xm(0) to the new transformed coordinates
    nld.env.x0(15) = -lambda;
    nld.env.x0(85) = -lambda;
    % use the position state dynamics in the new coords to obtain kx(0) and kr(0)
    temp_K = -nld.env.K_ref';
    nld.env.x0(19:42) = [temp_K(1,:) temp_K(2,:) temp_K(3,:) temp_K(4,:) temp_K(5,:) temp_K(6,:)]';
    nld.env.x0(43) = 1;
    nld.env.x0(48) = 1;
    nld.env.x0(53) = 1;
    nld.env.x0(58) = 1;
    
    % calculate alpha_hat(0)
    temp1 = [-Ka/m , -Ka/m , -Ka/m, -Ka/m;
             0 , -lambda*Ka*l/Ix , 0 , lambda*Ka*l/Ix;
             -lambda*Ka*l/Iy , 0 , lambda*Ka*l/Iy , 0];
    temp2 = [lambda , g , 0 , 0 , 0 , 0;
             0 , 0 , lambda*(-Ix+Iy-Iz)/Ix , g , 0 , 0;
             0 , 0 , 0 , 0 , -lambda*(-Ix+Iy+Iz)/Iy , -g];
    alpha_star = (temp1\temp2)';
    nld.env.x0(59:82) = [alpha_star(1,:) , alpha_star(2,:) , alpha_star(3,:) , alpha_star(4,:) , alpha_star(5,:) , alpha_star(6,:)]';






end 