function [Ax,Bx,Axm,Bxm,Ay,By, Aym, Bym, Az, Bz, Azm, Bzm, Ayaw, Byaw, Ayawm, Byawm,Px,Py,Pz,Kx,Ky,Kz,Pyaw,Kyaw,x0] = decentralized(quad,args)
    Phi = @(mismatch) [0, 0, 0, -mismatch*quad.env.grav*sin(quad.init_pos(6)), -mismatch*quad.env.grav*cos(quad.init_pos(6)), 0;
        0, 0, 0, quad.env.grav*cos(quad.init_pos(6)), -mismatch*quad.env.grav*sin(quad.init_pos(6)), 0;
       0, 0, 0, 0,           0,          0;
       0, 0, 0, 0,           0,          0;
       0, 0, 0, 0,           0,          0;
       0, 0, 0, 0,           0,          0];

    Delta = @(mismatch) [quad.thrust/(quad.mass*mismatch),      quad.thrust/(quad.mass*mismatch),           quad.thrust/(quad.mass*mismatch),       quad.thrust/(quad.mass*mismatch);
         0,         -quad.thrust*(quad.arm_length*mismatch)/quad.moments(1),       0,          quad.thrust*(quad.arm_length*mismatch)/quad.moments(1),;
         quad.thrust*(quad.arm_length*mismatch)/quad.moments(2),   0,              -quad.thrust*(quad.arm_length*mismatch)/quad.moments(2),   0;
         quad.thrust/quad.moments(3),     -quad.thrust/quad.moments(3),         quad.thrust/quad.moments(3),      -quad.thrust/quad.moments(3)];
    
    %% Standadard Matricies
    A0 = [zeros(6),eye(6); Phi(1), zeros(6)];
    B0 = [zeros(8,4); Delta(1)];
    A = [zeros(6),eye(6); Phi(args.mismatch), zeros(6)];
    B = [zeros(8,4); Delta(args.mismatch)];
    Bm = B0;
    %% Pole Placement
    Q = 10000*eye(12);
    [P,~,K] = care(A0,B0,Q);
    
    Am = (A - B*K);
    %% Initial Condt
    x0 = zeros(88,1);
    x0(25:72) = (pinv(B0)*(Am - A0));
    x0(73:88) = (pinv(B0)*Bm);

    %% X subssystem 
    Ax0 = [0.0 1.0 0.0 0.0;
          0.0 0.0 g  0.0;
          0.0 0.0 0.0 1.0;
          0.0 0.0 0.0 0.0];
    Bx0 = [0.0;0.0;0.0;1/(quad.moments(1))];
    Ax = [0.0 1.0 0.0 0.0;
          0.0 0.0 g  0.0;
          0.0 0.0 0.0 1.0;
          0.0 0.0 0.0 0.0];
    Bx = [0.0;0.0;0.0;1/(quad.moments(1)*mismatch)];
    Qx = 1000*eye(4);
    [Px, L, Kx] = care(Ax0,Bx0,Qx);
    Axm = Ax0 - Bx0*K;


    %% Y subsystem 
    Ay0 = [0.0 1.0 0.0 0.0;
          0.0 0.0 -g  0.0;
          0.0 0.0 0.0 1.0;
          0.0 0.0 0.0 0.0];

    By0 = [0.0;0.0;0.0;1/(quad.moments(2))];
    Ay = [0.0 1.0 0.0 0.0;
          0.0 0.0 -g  0.0;
          0.0 0.0 0.0 1.0;
          0.0 0.0 0.0 0.0];

    By = [0.0;0.0;0.0;1/(quad.moments(2)*mismatch)];
    Qy = 1000*eye(4);
    [Py, L, Ky] = care(Ay0,By0,Qy);
    Aym = Ay0 - By0*K;
    
    %% Z subsystem
    Az0 = [0.0 1.0;
          0.0 0.0];
    Bz0 = [0.0;
          1 / (quad.mass)];
    Az = [0.0 1.0;
          0.0 0.0];
    Bz = [0.0;
          1 / (quad.mass*mismatch)];
    Qz = 1000*eye(2);
    [Pz, L, Kz] = care(Az0,Bz0,Qz);
    Azm = Az0 - Bz0*K;
    
    %% Yaw subsystem 
    Ayaw0 = [0.0 1.0;
          0.0 0.0];
    Byaw = [0.0;
          1 / (quad.moments(3)*mismatch)];
    Ayaw = [0.0 1.0;
          0.0 0.0];
    Byaw0 = [0.0;
          1 / (quad.moments(3))];
    Qyaw = 1000*eye(2);
    [Pyaw, L, Kyaw] = care(Ayaw0,Byaw0,Qyaw);
    Ayawm = Ayaw0 - Byaw0*K;
end