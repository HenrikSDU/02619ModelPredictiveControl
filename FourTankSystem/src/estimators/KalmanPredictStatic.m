function xPredictedNext = KalmanPredictStatic(xFiltered, u, A, B)
    % Time update (prediction) for stationary Kalman filter
    xPredictedNext = A*xFiltered + B*u;
end