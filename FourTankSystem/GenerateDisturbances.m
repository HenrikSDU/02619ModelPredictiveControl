function [d, F3, F4] = GenerateDisturbances(t, Ts, disturbanceType, seed)

    % Generate reproducible disturbances for the four-tank system.
    %
    % Inputs:
    %   t               - Sampling-time vector [s]
    %   Ts              - Sampling time [s]
    %   disturbanceType - 'fixed', 'piecewise', or 'langevin'
    %   seed            - Random-number seed
    %
    % Outputs:
    %   d  - 2-by-N disturbance matrix
    %   F3 - Disturbance entering tank 3
    %   F4 - Disturbance entering tank 4

    N = length(t);

    % Mean disturbance flows [cm^3/s]
    F3_mean = 20;
    F4_mean = 20;

    % Initialize random number generator
    rng(seed);

    % Initialize disturbance matrix
    d = zeros(2,N);

    switch lower(disturbanceType)
        case 'fixed'

            d(1,:) = F3_mean;
            d(2,:) = F4_mean;

        case 'piecewise'

            % Standard deviations [cm^3/s]
            sigma_F3 = 2;
            sigma_F4 = 2;

            % Generate one value per sampling interval
            F3 = F3_mean + sigma_F3*randn(1,N);
            F4 = F4_mean + sigma_F4*randn(1,N);

            % Keep disturbance flows nonnegative
            F3 = max(0,F3);
            F4 = max(0,F4);

            d = [F3; F4];

        case 'langevin'

            % Mean-reversion rates [1/s]
            aF3 = 0.01;
            aF4 = 0.01;

            % Diffusion coefficients
            sigma_F3 = 0.2;
            sigma_F4 = 0.2;

            % Initial disturbance values
            d(:,1) = [F3_mean; F4_mean];

            % Generate correlated disturbance trajectories
            for k = 1:N-1

                d(:,k+1) = LangevinDisturbance( ...
                    d(:,k), ...
                    Ts, ...
                    aF3, ...
                    aF4, ...
                    F3_mean, ...
                    F4_mean, ...
                    sigma_F3, ...
                    sigma_F4);

            end

            % Physical constraint: disturbance flows >= 0
            % d = max(d,0);

        otherwise
            error(['Unknown disturbance type. Use ', ...
                   '''fixed'', ''piecewise'', or ''langevin''.']);

    end

    F3 = d(1,:);
    F4 = d(2,:);

end