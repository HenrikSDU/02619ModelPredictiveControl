function [xFiltered,xPredictedNext,innovation] = KalmanFilterStationary(xPredicted,y,u,A,B,C,Kfx)

    % Measurement update
    innovation = y - C*xPredicted;
    xFiltered = xPredicted + Kfx*innovation;

    % Time update
    xPredictedNext = A*xFiltered + B*u;

end