function [Kfx,P,Re] = DesignStationaryKalmanFilter(A,G,C,Q,R)

    % Stationary prediction-error covariance
    P = dare(A', C', G*Q*G', R);

    % Innovation covariance
    Re = C*P*C' + R;

    % Measurement-update gain
    Kfx = (P*C') / Re;

end