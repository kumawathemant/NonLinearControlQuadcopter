function [r,dr] = sinusoidal(t,A,omega,phase)
    r = A*sin(omega*t);
    dr = omega*A*cos(omega*t+phase);
end