function [T, X, H, Y, D] = simulateStepNoise(us, xs, p, stepSize, inputIndex, tf, dMean, Ts, sigmaProcess, sigmaMeasurement)

    % Simulation time
    t = (0:Ts:tf)';
    N = length(t);

    nx = length(xs);
    ny = 4;

    % Allocate
    X = zeros(N,nx);
    H = zeros(N,ny);
    Y = zeros(N,ny);
    D = zeros(N,2);

    % Initial condition
    X(1,:) = xs';

    % Input after the step
    u = us;

    u(inputIndex) = us(inputIndex)*(1 + stepSize);

    % Initial disturbance
    D(1,:) = dMean';

    % Measurement noise covariance
    R = sigmaMeasurement^2 * eye(ny);

    % Cholesky factor
    L = chol(R,'lower');

    % Initial measurements
    h = FourTankSystemSensor(xs,p);

    H(1,:) = h';
    Y(1,:) = FourTankSystemSensorNoise(X(1,:)', p, sigmaMeasurement)';

    % Time vector for ode solver
    T = t;

    % Simulation

    for k = 1:N-1

        % Current disturbance
        d = D(k,:)';

        % Integrate nonlinear four-tank model
        [~,Xk] = ode15s( ...
            @(time,state) ModifiedFourTankSystem(time,state,u,p,d), ...
            [t(k) t(k+1)], ...
            X(k,:)' );

        % Store final state
        X(k+1,:) = Xk(end,:);

        D(k+1,:) = dMean' + sigmaProcess*randn(1,2); % Process noise

        h = FourTankSystemSensor(X(k+1,:)',p); % True levels

        H(k+1,:) = h';

        Y(k+1,:) = FourTankSystemSensorNoise(X(k+1,:)', p, sigmaMeasurement)';

    end

end