N = 50; % prediction horizon

%For a square system, we only use outputs x,y,z and psi
% If we use x,y,z and phi or x,y,z and theta, the matrix big_A is singular
big_A = [A_d - eye(nx), B_d; 
        C_d(1:3,:), D_d(1:3,:);
        C_d(6,:), D_d(6,:)];

big_b = [zeros(nx,4);
        eye(4)];
big_N = inv(big_A)*big_b;

N_x = big_N(1:nx,:); % x_ss = N_x * ref
N_u = big_N(nx+1:end,:); % u_ss = N_u * ref

T_s = 0.05;

M = 500;
refs = zeros(M,4);
Q = diag([10,10,1e4,1*ones(1,nx-3)]);
R = eye(nu);

U_vector = zeros(M,nu);
X_vector = zeros(M,nx);
Y_vector = zeros(M,ny);
r = [5 4 1 0 0 0 0 0 0 0 0 0];
% initial state
x = x0_quadcopter;

options = odeset('RelTol',1e-13,'AbsTol',1e-16);
for k=1:M
    S = Q;
    % solve the Riccati difference equation backwards in time
    for n=1:N-1
        S = riccati_diffeq(S,A_d,B_d,Q,R);
    end
    K = inv(R+B_d'*S*B_d)*B_d'*S*A_d;
    refs(k,:) = [5,4,1,0];
    r = [5,4,1,0]';
%     r = refs(k,:)'; % reference 
    u = -K*(x-N_x*r)+N_u*r;
    y = C_d*x;
    [~,X]=ode113(@(t,xt) NLDyn([xt;u+u_eq*ones(nu,1)]),[0,T_s],x,options);
    x = X(end,:)';
    
    U_vector(k,:) = u';
    X_vector(k,:) = x';
    Y_vector(k,:) = y';
end

refs = refs(:,1:3);
T=T_s*(0:M-1);



%% Our Figures; 
figs(1)  = figure ;

l = tiledlayout('flow');
r = '';
s = '';
r = 'r_{scalar}';
a = 'MPC';

title(l,['Quadcopter Trajectory with',a,' Linear Controller (',r,s,')'])
                
l = nexttile;
title(l,'Position')
hold on
plot(T,X_vector (:,1));
plot(T ,X_vector (:,2));
plot(T ,X_vector (:,3));
ylabel('Magnitude (m)');
xlabel('Time (s)');
hold off
grid on
legend('x','y','z','x_m','y_m','z_m','Interperter','latex');

l = nexttile;
title(l,'Rotation')
hold on
plot(T ,X_vector(:,4));
plot(T ,X_vector(:,5));
plot(T ,X_vector(:,6));

ylabel('Magnitude (rad)');
xlabel('Time (s)');
hold off
grid on
legend('$\phi$','$\theta$','$\psi$','$\phi_m$','$\theta_m$','$\psi_m$','Interpreter','latex');
l = nexttile;
                  
title(l,'Rotation')
hold on
plot(T ,X_vector(:,10));
plot(T ,X_vector(:,11));
plot(T ,X_vector(:,12));

ylabel('Magnitude (rad)');
xlabel('Time (s)');
hold off
grid on
legend('$p$','$q$','$r$','$\phi_m$','$\theta_m$','$\psi_m$','Interpreter','latex');
l = nexttile;
hold on;
plot(T,U_vector);
ylabel('Magnitude');
xlabel('Time (s)');
title('Control inputs');
xlabel('T [s]')
grid on;

hold off;
figure;
plot3(X_vector(:,1),X_vector(:,2),X_vector(:,3),'r')
title(['Quadcopter Trajectory with MPC'])

view(3)
zlabel('y (m)','Interpreter','latex');
ylabel('z (m)','Interpreter','latex');
xlabel('x (m)','Interpreter','latex');
hold off
grid on


