close all; clear all; %clc
addpath(genpath('trajectories'));
addpath(genpath('laws'));
addpath(genpath('matrices'));
addpath(genpath('graphs'))
%% GLOBAL
TSPAN = [0 .1];
%% Quadcopter 
quad = quadcopter();
nld = nldsystem();
%% Args 
% Test Settings
%
% args.AdaptiveLaw = 1: linear control, 2: linear adaptive, 3: nonlinear - linear bad design , 4: nonlinear - bad design adaptive, 5: projection, 6: modulation
% args.Reference = 1: scalar, 2: sinusoudal, 3: rsteps
% args.Noise = 0; % Inject noise? 1: yes 0: no

args = struct;
args.Quadcopter = quad;
args.system = nld;
args.mismatch = 1.2;
args.gamma = 0.01;

%% Linearized Case

% Args
args.Noise = 0;
[A,B,Am,Bm,A0,B0,P,K,x0] = linearized(quad,args);
args.A = A;
args.B = B;
args.A0 = A0;
args.B0 = B0;
args.Am = Am;
args.Bm = Bm;
args.P = P;
args.K = K;
args.Reference = 1;

% Linear Controller
args.AdaptiveLaw = 1;
for i = 1:3
   args.Reference = i
   [t,x] = ode45(@(t,x) controller(t,x,args),TSPAN,x0(1:24));
   parametric(t,x,args)
   position(t,x,args);
end 

%Adaptive Controller
args.AdaptiveLaw = 2;
for i = 1:3
    args.Reference = i;
    [t,x] = ode45(@(t,x) controller(t,x,args),TSPAN,x0);
    parametric(t,x,args);
    position(t,x,args);
end 

%% Nonlinear Case with Linear feedback 
args.AdaptiveLaw = 3;
for i = 1:3
    args.Reference = i;
    [t,x] = ode45(@(t,x) LCNS(t,x,args), TSPAN, x0(1:24));
    parametric(t,x,args);
    position(t,x,args);
end 

%% Nonlinear Case with Adaptive
args.AdaptiveLaw = 4;
for i = 1:3
    args.Reference = i;
    [t,x] = ode45(@(t,x) LCNS(t,x,args), TSPAN, x0);
    parametric(t,x,args);
    position(t,x,args);
end 

%% Nonlinear Case with Transformed Coordinates

%%%% THIS ONE MAY BE TOO SLOW: TRY THE ONE IN FASTER FILE  

args.AdaptiveLaw = 5;
for i = 1:1
%     args.Reference = i;
%     [t,x] = ode45(@(t,x) NCNS(t,x,args), TSPAN, args.system.env.x0);
%     parametric(t,x,args);
%     position(t,x,args);
end 

