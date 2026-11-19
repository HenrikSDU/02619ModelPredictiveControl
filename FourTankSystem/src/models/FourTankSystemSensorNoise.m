function y = FourTankSystemSensorNoise(xk,p,sigma_h)
    % Measurements of tanks 1 and 2 with independent noise

    if nargin < 3
        sigma_h = 0.1; % Default standard deviation [cm]
    end

    y = FourTankSystemSensor(xk,p);
    v = sigma_h * randn(2,1);

    y = y + v;
end