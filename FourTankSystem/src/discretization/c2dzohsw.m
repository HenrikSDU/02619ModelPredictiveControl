function [Abar,Bbar,Qbar] = c2dzohsw(A,B,G,Ts)

[Abar, Bbar] = c2dzoh(A,B,Ts);
[~, Qbar] = c2dsw(A,G,Ts);