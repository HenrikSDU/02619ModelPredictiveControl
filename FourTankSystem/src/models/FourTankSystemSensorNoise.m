function y = FourTankSystemSensorNoise(xk,p,sigma_h)
    % Measurements of tanks 1 and 2 with independent noise

    if nargin < 3
        sigma_h = 0.1; % Default standard deviation [cm]
    end

    y = FourTankSystemSensor(xk,p);
    
    w = randn(2, 1);

    if isscalar(sigma_h)
        v = sigma_h * w;
    else % R is given
        LR = chol(sigma_h,'lower');
        v = LR * w;
    end

    y = y + v;
end