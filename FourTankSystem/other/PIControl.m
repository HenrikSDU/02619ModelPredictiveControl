function [u, i] = PIControl(i,r,y,us,Kc,Ti,Ts,umin,umax)
arguments
    i 
    r 
    y 
    us 
    Kc 
    Ti 
    Ts 
    umin = -inf
    umax = +inf
end
    e = r - y;
    v = us + Kc.*e + i;
    i = i + (Kc.*Ts./Ti).*e;
    u = max(umin, min(umax,v));
end