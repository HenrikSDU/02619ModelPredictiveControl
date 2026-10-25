function mpc = DesignUnconstrainedMPC(A,B,Cz,N,Wz,Wu,Wdu)

    nx = size(A,1);
    nu = size(B,2);
    nz = size(Cz,1);

    % Prediction: Zk = bk + Gamma*Uk

    Phi_x = zeros(N*nz,nx);
    Gamma = zeros(N*nz,N*nu);

    for j = 1:N
        rows = (j-1)*nz + (1:nz);

        Phi_x(rows,:) = Cz*A^j;

        for i = 1:j
            cols = (i-1)*nu + (1:nu);

            Gamma(rows,cols) = Cz*A^(j-i)*B;
        end
    end

    % Stacked weights

    Wz_bar = kron(eye(N),Wz);
    Wu_bar = kron(eye(N),Wu);
    Wdu_bar = kron(eye(N),Wdu);

    % Input differences: DeltaUk = Lambda*Uk - I0*uPrevious

    Lambda = eye(N*nu);

    for j = 2:N
        rows = (j-1)*nu + (1:nu);
        cols = (j-2)*nu + (1:nu);

        Lambda(rows,cols) = -eye(nu);
    end

    I0 = [eye(nu);
          zeros((N-1)*nu,nu)];

    % Cost matrices

    Hz = (Wz_bar*Gamma)'*(Wz_bar*Gamma);
    Hu = Wu_bar'*Wu_bar;
    Hdu = (Wdu_bar*Lambda)'*(Wdu_bar*Lambda);

    Mz = -(Wz_bar*Gamma)'*Wz_bar;
    Mu = -Hu;
    Mdu = -(Wdu_bar*Lambda)'*Wdu_bar*I0;

    % Store the design

    mpc.Phi_x = Phi_x;
    mpc.Gamma = Gamma;

    mpc.H = Hz + Hu + Hdu;
    mpc.Mz = Mz;
    mpc.Mu = Mu;
    mpc.Mdu = Mdu;

    mpc.N = N;
    mpc.nu = nu;

end