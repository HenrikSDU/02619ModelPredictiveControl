function y = FourTankSystemSensorNoise(xk,p,sigma_h)
    % Extract dimensions and density from parameters
    rho = p(12,1);
    A = p(5:8);
    
    if nargin < 3
        % Standard deviation of level measurement noise
        sigma_h1 = 0.1; % [cm]
        sigma_h2 = 0.1; % [cm]
        sigma_h3 = 0.1; % [cm]
        sigma_h4 = 0.1; % [cm]
        
        R_v = diag([sigma_h1^2;
                    sigma_h2^2;
                    sigma_h3^2;
                    sigma_h4^2]);
    else
        R_v = diag([sigma_h^2;
                sigma_h^2;
                sigma_h^2;
                sigma_h^2]);
    end

    h = xk ./ (A*rho); % Calculate the levels
    v = chol(R_v,'lower') * randn(4,1);

    y = h + v;
end