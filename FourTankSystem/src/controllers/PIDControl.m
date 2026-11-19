function [u, i_state, y_prev] = PIDControl(i_state, y_prev, r, y, us, Kp, Ti, Td, Ts, umin, umax, controllerType)

    e = r - y;
    P = Kp .* e;
    I = i_state;
    D = -Kp .* Td .* (y - y_prev) ./ Ts;

    switch upper(controllerType)

        case 'P'
            v = us + P;

        case 'PI'
            v = us + P + I;

        case 'PID'
            v = us + P + I + D;

        otherwise
            error('Unknown controller type. Use P, PI, or PID.');
    end

    u = min(umax, max(umin, v)); % saturation
    Ki = Kp ./ Ti;
    i_state = i_state + Ts * (Ki .* e + (u - v) ./ Ti); % anti-windup

    y_prev = y;

end