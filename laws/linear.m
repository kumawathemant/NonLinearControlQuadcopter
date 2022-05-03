function u = linear(A,B,K,x,r)
    Kr = B\(A-B*K);
    u = -K*x - Kr*r;
end