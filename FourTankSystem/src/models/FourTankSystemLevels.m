function h = FourTankSystemLevels(xk,p)
    % All four tank levels [cm]
    % xk: column vector of four masses [g]

    rho = p(12);
    A = p(5:8);

    h = xk ./ (rho * A);
end