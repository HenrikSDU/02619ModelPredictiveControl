function [x_hat, xPredictedNext, PpredictedNext, innovation, Kfx] = KalmanFilterDynamic( ...
          xPredicted, Ppredicted, y, u, A, B, G, C, Q, R)

    % Measurement update
    innovation = y - C*xPredicted;

    Re = C*Ppredicted*C' + R;
    Kfx = (Ppredicted*C') / Re;

    x_hat = xPredicted + Kfx*innovation;

    Pfiltered = Ppredicted - Kfx*Re*Kfx';

    % Time update
    xPredictedNext = A*x_hat + B*u;
    PpredictedNext = A*Pfiltered*A' + G*Q*G';

end