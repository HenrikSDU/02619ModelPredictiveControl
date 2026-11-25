function [xFiltered, Pfiltered, innovation, Kfx] = KalmanUpdateDynamic(xPredicted, Ppredicted, y, C, R)
    % Measurement update (correction) for dynamic Kalman filter
    innovation = y - C*xPredicted;

    Re  = C*Ppredicted*C' + R;
    Kfx = (Ppredicted*C') / Re;

    xFiltered = xPredicted + Kfx*innovation;
    Pfiltered = Ppredicted - Kfx*Re*Kfx';
end