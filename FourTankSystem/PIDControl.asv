function [u, i_state, d_state] = PIDControl(i_state, d_state, r, y, us, Kp, Ti, Td, Ts, umin, umax, controllerType)

    e = r - y;
    P = Kp .* e;
    I = i_state;
    D = -Kp .* Td .* d_state;

    switch upper(controllerType)

        case 'P'
            v = us + P;

        case 'PI'
            v = us + P + I;
            Ki = Kp ./ Ti;
            i_state= i_state + Ki .* e .* Ts;

        case 'PID'
            v = us + P + I + D;
            Ki = Kp ./ Ti;
            i_state= i_state + Ki .* e .* Ts;
            d_state = (y - d_state) ./ Ts;

        otherwise
            error('Unknown controller type. Use P, PI, or PID.');
    end

    u = min(umax, max(umin, v));

end