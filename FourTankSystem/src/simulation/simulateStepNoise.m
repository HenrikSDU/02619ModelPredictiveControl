function [T, X, H, Y, D] = simulateStepNoise(us, xs, p, stepSize, inputIndex, tf, dMean, Ts, sigmaProcess, sigmaMeasurement)

    % Simulation time
    T = (0:Ts:tf)';
    N = length(T);

    nx = length(xs);
    ny = 2;
    nh = 4;

    % Allocate
    X = zeros(N,nx);
    H = zeros(N,nh);
    Y = zeros(N,ny);
    D = zeros(N,2);

    % Initial condition
    X(1,:) = xs';

    % Input after the step
    u = us;

    u(inputIndex) = us(inputIndex)*(1 + stepSize);

    % Initial disturbance
    D(1,:) = dMean';

    H(1,:) = FourTankSystemLevels(xs,p)';
    Y(1,:) = FourTankSystemSensorNoise(X(1,:)', p, sigmaMeasurement)';

    % Simulation
    for k = 1:N-1

        % Current disturbance
        d = D(k,:)';

        % Integrate nonlinear four-tank model
        [~,Xk] = ode15s( ...
            @(time,state) ModifiedFourTankSystem(time,state,u,p,d), ...
            [T(k) T(k+1)], ...
            X(k,:)' );

        % Store final state
        X(k+1,:) = Xk(end,:);

        D(k+1,:) = dMean' + sigmaProcess*randn(1,2); % Process noise

        xk = X(k+1,:)';

        H(k+1,:) = FourTankSystemLevels(xk,p)';
        Y(k+1,:) = FourTankSystemSensorNoise(xk,p,sigmaMeasurement)';

    end

end