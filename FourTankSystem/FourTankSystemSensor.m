function y = FourTankSystemSensor(xk,p)
    % Measured levels: tanks 1 and 2 [cm]

    h = FourTankSystemLevels(xk,p);
    y = h(1:2);
end