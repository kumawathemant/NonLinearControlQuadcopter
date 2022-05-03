function xdot = LCNS(t,X,args)
    display(t);
    X = X';
%% System Parameters
    A = args.A;
    B = args.B;
    Am = args.Am;
    Bm = args.Bm;
    A0 = args.A0;
    B0 = args.B0;
    P = args.P;
    K = args.K;

    g = args.gamma;

    dx = [];
    dx_m = [];
    dk_x = [];
    dk_r = [];
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
    r = r';
    
%% States
    m = args.Quadcopter.mass;
    I = args.Quadcopter.moments;
    Ix = I(1);
    Iy = I(2);
    Iz = I(3);
    x = X(1:12)';
    x_m = X(13:24)';
    
    u = linear(A,B,K,x,r);
    dx = A*x+B*u;
    T = args.Quadcopter.thrust *sum(u);
    tau_x = args.Quadcopter.thrust*  args.Quadcopter.arm_length * (u(4) - u(2));
    tau_y = args.Quadcopter.thrust*  args.Quadcopter.arm_length * (u(1) - u(3));
    tau_z = args.Quadcopter.thrust*  args.Quadcopter.arm_length * (u(1) - u(2) + u(3) - u(4));
    x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4); x5 = x(5); x6 = x(6);
    x7 = x(7); x8 = x(8); x9 = x(9); x10 = x(10); x11 = x(11); x12 = x(12);
    xdot = zeros(12,1)
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
    dx = xdot;
    u = linear(Am,Bm,K,x,r);
    dx_m = Am*x_m + Bm*u;
    r = K*r;
    %try
        switch args.AdaptiveLaw
            case 3
                xdot = [dx; dx_m;];
            case 4
                
                switch args.Reference
                    case 2
                        Q = eye(12)/1000;
                        P = lyap(Am',Q);
                    case 3
                        Q = eye(12)/1000;
                        P = lyap(Am',Q);
                end

                k_x = [X(25:28);X(29:32);X(33:36);X(37:40);X(41:44);X(45:48);X(49:52);X(53:56);X(57:60);X(61:64);X(65:68);X(69:72)];
                k_r = [X(73:76) ; X(77:80) ; X(81:84) ; X(85:88)];

                u = k_x'*x + k_r'*r;
                

                
                err = x-x_m;
                T = args.Quadcopter.thrust *sum(u);
                tau_x = args.Quadcopter.thrust*  args.Quadcopter.arm_length * (u(4) - u(2));
                tau_y = args.Quadcopter.thrust*  args.Quadcopter.arm_length * (u(1) - u(3));
                tau_z = args.Quadcopter.thrust*  args.Quadcopter.arm_length * (u(1) - u(2) + u(3) - u(4));
                x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4); x5 = x(5); x6 = x(6);
                x7 = x(7); x8 = x(8); x9 = x(9); x10 = x(10); x11 = x(11); x12 = x(12);
                xdot = zeros(12,1)
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

                dx = xdot;
                dx_m = Am*x_m + Bm*r;
                dk_x= -g*x*err'*P*B;
                dk_r = -g*r*err'*P*B;
                
                xdot = [dx; dx_m;reshape(dk_x,numel(dk_x),1);reshape(dk_r,numel(dk_r),1)];
                %xdot = zeros(88,1);
                %xdot(1:12) = dx;
                %xdot(13:24) = dx_m;
                %xdot(25:72) = [dk_x(1,:) dk_x(2,:) dk_x(3,:) dk_x(4,:) dk_x(5,:) dk_x(6,:) dk_x(7,:) dk_x(8,:) dk_x(9,:) dk_x(10,:) dk_x(11,:) dk_x(12,:)]';
                %xdot(73:88) = [dk_r(1,:) dk_r(2,:) dk_r(3,:) dk_r(4,:)]';        
        end 
    %catch ERR
    %    display(ERR)
    %end 
    
    
end