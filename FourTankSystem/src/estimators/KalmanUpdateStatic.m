function [xFiltered, innovation] = KalmanUpdateStatic(xPredicted, y, C, Kfx)
    % Measurement update (correction) for stationary Kalman filter
    innovation = y - C*xPredicted;
    xFiltered  = xPredicted + Kfx*innovation;
end