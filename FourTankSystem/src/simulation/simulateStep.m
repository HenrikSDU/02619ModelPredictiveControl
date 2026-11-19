function [T, X, H] = simulateStep(us, xs, p, stepSize, stepInput, tf, d)

    % Step input
    u = us;

    u(stepInput) = (1 + stepSize)*us(stepInput);

    % Nonlinear simulation
    odefun = @(t,x) ModifiedFourTankSystem(t,x,u,p,d);
    [T,X] = ode45(odefun,[0 tf],xs);

    % Extract levels
    rho = p(12);
    At = p(5:8);

    H = X ./ (rho * At');

end