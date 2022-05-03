function [quad] = quadcopter()
    quad = struct;
    quad.env = struct;
    
    % Environment Variables
    quad.env.grav = 9.81; %m/s^2
    quad.env.rho = 1.225;    % density of air
    
    max_rpm = 10000/60 * 2*pi; % https://www.rotordronepro.com/guide-multirotor-motors/
    
    % Geometric Variables
    quad.mass = .865; % kg
    quad.arm_length = 0.11276; % m
    quad.rotor_rad = 0.119;    % m
    quad.rotor_vel = max_rpm*quad.rotor_rad;
    quad.dim = [0.3475 0.283 0.1077]; % [ L , W , H ] of general rectangular shape
    quad.thrust = quad.mass*quad.env.grav/(4*(quad.rotor_vel)^2);
    quad.init_pos = [0 0 0 0 0 0 0 0 0 0 0 0];

    Ix = quad.mass/12*(quad.dim(2)^2 + quad.dim(3)^2);
    Iy = quad.mass/12*(quad.dim(1)^2 + quad.dim(3)^2);
    Iz = quad.mass/12*(quad.dim(1)^2 + quad.dim(2)^2);

    quad.moments = [Ix Iy Iz];
end 