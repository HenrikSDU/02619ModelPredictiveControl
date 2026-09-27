function u = PControl(r,y,us,Kc,umin,umax)
arguments
    r 
    y 
    us 
    Kc 
    umin = -inf
    umax = +inf
end    
    e = r - y;
    v = us + Kc*e;
    u = max(umin, min(umax,v));
end