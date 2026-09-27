function z = FourTankSystemOutput(xk,p)
    % Extract dimensions and density from parameters
    rho = p(12,1);
    A = p(5:8);
    h = xk ./ (A*rho); % Calculate the levels
    z = h(1:2,1);
end