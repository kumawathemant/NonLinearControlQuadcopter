function xdot = controller(t,X,args)
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
    x = X(1:12)';
    x_m = X(13:24)';
    
    u = linear(A,B,K,x,r);
    dx = A*x+B*u;
    
    u = linear(Am,Bm,K,x,r);
    dx_m = Am*x_m + Bm*u;
    r = K*r;
    %try
        switch args.AdaptiveLaw
            case 1 
                xdot = [dx; dx_m;];
            case 2
                
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
                dx = A*x+B*u;
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