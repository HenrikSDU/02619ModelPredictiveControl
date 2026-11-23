function [Abar,Qbar] = c2dsw(A,G,Ts)

nx = size(A,1);
nx1 = nx+1;
nx2 = nx+nx;
M = [-A G*G'; zeros(nx,nx) A']*Ts;
Phi = expm(M);
Abar = Phi(nx1:nx2,nx1:nx2)';
Qbar = Abar*Phi(1:nx,nx1:nx2);