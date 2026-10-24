function xdot = ModifiedFourTankSystem(t,x,u,p,d)
    % Unpack states, MVs, and parameters
    F = u; % Flow rates in pumps [cm3/s]
    a = p(1:4,1); % Pipe cross sectional areas [cm2]
    gamma = p(9:10,1); % Valve positions [-]
    g = p(11,1); % Acceleration of gravity [cm/s2]
    rho = p(12,1); % Density of water [g/cm3]
    
    % Inflows
    qin = zeros(4,1);
    qin(1,1) = gamma(1)*F(1); % Valve 1 to tank 1 [cm3/s]
    qin(2,1) = gamma(2)*F(2); % Valve 2 to tank 2 [cm3/s]
    qin(3,1) = (1-gamma(2))*F(2); % Valve 2 to tank 3 [cm3/s]
    qin(4,1) = (1-gamma(1))*F(1); % Valve 1 to tank 4 [cm3/s]
    
    % Outflows
    h = FourTankSystemLevels(x,p);
    qout = a.*sqrt(2*g*h); % Outflow from each tank [cm3/s]
    
    % Differential equations, mass balances
    xdot = zeros(4,1);
    xdot(1,1) = rho*(qin(1,1)+qout(3,1)-qout(1,1)); % Tank 1
    xdot(2,1) = rho*(qin(2,1)+qout(4,1)-qout(2,1)); % Tank 2
    xdot(3,1) = rho*(qin(3,1)-qout(3,1)+d(1)); % Tank 3
    xdot(4,1) = rho*(qin(4,1)-qout(4,1)+d(2)); % Tank 4

end