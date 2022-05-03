clear;

% Data
m = 0.5;
L = 0.25;
k = 3 * 10^(-6);
b = 1 * 10^(-7);
g = 9.81;
k_d = 0.25;
Ixx = 5 * 10^(-3);
Iyy = 5 * 10^(-3);
Izz = 1 * 10^(-2);
c_m = 1 * 10^(4);
T_s=0.05;

% System Sizes
nx = 12;    % Nb of states
nu = 4;     % Nb of inputs
ny = 6;     % Nb of outputs

x0_quadcopter = zeros(nx,1);

% Inputs at equilbrium point (all states 0)
u_eq = g*m/(4*k*c_m);

% symbolic variables
syms x y z v_x v_y v_z phi theta psi w_x w_y w_z u1 u2 u3 u4
% state vector
state = [x; y; z; v_x; v_y; v_z; phi; theta; psi; w_x; w_y; w_z];
% input vector
input = [u1; u2; u3; u4];

% The functions f:
f1 = x+T_s*v_x;
f2 = y+T_s*v_y;
f3 = z+T_s*v_z;

f4 = v_x+T_s*(-k_d/m * v_x + k*c_m/m *(sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta))*(u1+u2+u3+u4));
f5 = v_y+T_s*(-k_d/m * v_y + k*c_m/m *(cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi))*(u1+u2+u3+u4));
f6 = v_z+T_s*(-k_d/m * v_z - g + k*c_m/m *(cos(theta)*cos(phi))*(u1+u2+u3+u4));

f7 = phi+T_s*(w_x + w_y*(sin(phi)*tan(theta)) + w_z*(cos(phi)*tan(theta)));
f8 = theta+T_s*(w_y*cos(phi) - w_z*sin(phi));
f9 = psi+T_s*(sin(phi)/cos(theta) *w_y + cos(phi)/cos(theta) * w_z);

f10 = w_x+T_s*(L*k*c_m/Ixx * (u1-u3) - (Iyy-Izz)/Ixx * w_y*w_z);
f11 = w_y+T_s*(L*k*c_m/Iyy * (u2-u4) - (Izz-Ixx)/Iyy * w_x*w_z);
f12 = w_z+T_s*(b*c_m/Izz * (u1-u2+u3-u4) - (Ixx-Iyy)/Izz * w_y*w_x);

fun = [f1; f2; f3; f4; f5; f6; f7; f8; f9; f10; f11; f12];

% Deriving the functions in the state variables (Jacobian)
J = jacobian(fun, state);
 
% Evaluating the jacobian in the equilibrium values: the result is A
A = subs(J,[state; input],[zeros(nx,1); u_eq*ones(nu,1)]);
A_d = double(A);


% Deriving the functions in the input variables
J = jacobian(fun, input);

% Evaluating the derivatives in the equilibrium values: the result is B
B = subs(J, [state; input],[zeros(nx,1); u_eq*ones(nu,1) ]);
B_d = double(B);

% The output consists of states 1 to 3 and 7 to 9, so C selects these and D
% is zero
C_d = [eye(3), zeros(3,9);
     zeros(3,6), eye(3), zeros(3,3)];
D_d = zeros(ny,nu);

% Creating the continuous time system
sys = ss(A_d,B_d,C_d,D_d,T_s);


disp('Poles:')
disp(eig(A))

disp ('Controllability matrix');

CO = ctrb(A_d,B_d);

disp('Rank of the controllability matrix:');
rank(CO)
disp ('Observability matrix');

OB = obsv(A_d,C_d);

disp('Rank of the observability matrix:');
rank(OB)



