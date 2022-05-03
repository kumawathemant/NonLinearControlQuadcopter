function xdot = nonlinear_dyn(x, ut, args)
    xdot = zeros(12,1);
    Ka = args.system.env.Ka;
    Km = args.system.env.Km;
    l = args.system.env.l;
    m = args.system.env.m;
    g = args.system.env.g;
    Ix = args.system.env.Ix;
    Iy = args.system.env.Iy;
    Iz = args.system.env.Iz;
    x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4); x5 = x(5); x6 = x(6);
    x7 = x(7); x8 = x(8); x9 = x(9); x10 = x(10); x11 = x(11); x12 = x(12);
    
    T = Ka * sum(ut);
    tau_x = Ka*l * (ut(4) - ut(2));
    tau_y = Ka*l * (ut(1) - ut(3));
    tau_z = Km * (ut(1) - ut(2) + ut(3) - ut(4));
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
    

end 