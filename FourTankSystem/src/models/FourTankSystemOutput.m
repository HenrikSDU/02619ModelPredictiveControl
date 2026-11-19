function z = FourTankSystemOutput(xk,p)
    % Controlled levels: tanks 1 and 2 [cm]

    h = FourTankSystemLevels(xk,p);
    z = h(1:2);
end