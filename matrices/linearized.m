function [A,B,Am,Bm,A0,B0,P,K,x0] = linearized(quad,args)
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
end