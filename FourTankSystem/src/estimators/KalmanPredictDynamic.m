
function [xPredictedNext, PpredictedNext] = KalmanPredictDynamic(xFiltered, Pfiltered, u, A, B, G, Q)
    % Time update (prediction) for dynamic Kalman filter
    xPredictedNext = A*xFiltered + B*u;
    PpredictedNext = A*Pfiltered*A' + G*Q*G';
end