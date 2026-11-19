function [u,Uk,Zk] = ComputeUnconstrainedMPC(mpc,x,r,ubar,uPrevious)

    % Constant references over the horizon
    Rk = repmat(r,mpc.N,1);
    Ubar = repmat(ubar,mpc.N,1);

    % Free response: no estimated process-noise contribution
    bk = mpc.Phi_x*x;

    ck = Rk - bk;

    % Linear term
    g = mpc.Mz*ck + mpc.Mu*Ubar + mpc.Mdu*uPrevious;

    % Unconstrained optimum: H*Uk + g = 0
    %[Uk,info] = qpsolver(mpc.H,g,[],[],[],[],[],[]);
    Uk = -(mpc.H \ g); 

    % First command and predicted outputs
    u = Uk(1:mpc.nu);
    Zk = bk + mpc.Gamma*Uk;

end