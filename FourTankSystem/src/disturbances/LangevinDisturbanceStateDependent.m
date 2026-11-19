function d_next = LangevinDisturbanceStateDependent(d,Ts,aF3,aF4,F3_mean,F4_mean,sigma_F3,sigma_F4)

    F3 = d(1);
    F4 = d(2);

    % Euler-Maruyama discretization of the Ornstein-Uhlenbeck process
    F3_next = F3 + aF3*(F3_mean - F3)*Ts + sigma_F3*(F3/F3_mean)*sqrt(Ts)*randn;
    F4_next = F4 + aF4*(F4_mean - F4)*Ts + sigma_F4*(F4/F4_mean)*sqrt(Ts)*randn;
    
    % Avoid discrete overshoot
    F3_next = max(0, F3_next);
    F4_next = max(0, F4_next);

    d_next = [F3_next;
              F4_next];

end