%================================ rsteps.m ===============================
%
%  function [r, rp] = rsteps(t)
%
%  Generates a funky, periodic signal for all time.  Should be run for at
%  least 30 seconds to go past one period (a period is 25 seconds).
%
%================================ rsteps.m ===============================
function rsig = rsteps(t)

state = mod(floor(t/5),5);
tstart = floor(t/5)*5;
tau = t - tstart;

switch state
  case 0
    r  = 2;
    rp = 0;
  case 1
    r  =  cos(2*pi*tau/5) + 1;
    rp = -(2*pi/5)*sin(2*pi*tau/5);
  case 2
    r  = polyval([-16/1000 4/100 0 2], tau);
    rp = polyval([-3*16/1000 2*4/100 0], tau);
  case 3
    r  = 1;
    rp = 0;
  case 4
    r  = 1 + tau/5;
    rp = 1/5;
end

rsig = [r ; rp];

end
    

%
%================================ rsteps.m ===============================
