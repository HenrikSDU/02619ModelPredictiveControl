function [x,info] = qpsolver(H,g,l,u,A,bl,bu,xinit)
    % Solve:
    %   min 0.5*x'*H*x + g'*x
    %   subject to l <= x <= u
    %              bl <= A*x <= bu
    %
    % Use [] for absent bounds or constraints.
    % H must be symmetric positive semidefinite.

    n = size(H,1);

    g = g(:);

    % Variable bounds
    if isempty(l)
        l = -inf(n,1);
    else
        l = l(:);
    end

    if isempty(u)
        u = inf(n,1);
    else
        u = u(:);
    end

    % Linear constraints
    if isempty(A)
        A = zeros(0,n);
    end

    m = size(A,1);

    if isempty(bl)
        bl = -inf(m,1);
    else
        bl = bl(:);
    end

    if isempty(bu)
        bu = inf(m,1);
    else
        bu = bu(:);
    end

    % Initial guess
    if nargin < 8 || isempty(xinit)
        xinit = zeros(n,1);
    else
        xinit = xinit(:);
    end

    % Dimension checks
    assert(size(H,2) == n && numel(g) == n, ...
        'H and g have incompatible dimensions.');

    assert(numel(l) == n && numel(u) == n, ...
        'Variable bounds must have n elements.');

    assert(size(A,2) == n && ...
           numel(bl) == m && numel(bu) == m, ...
        'Linear constraints have incompatible dimensions.');

    assert(numel(xinit) == n, ...
        'xinit must have n elements.');

    assert(all(l <= u) && all(bl <= bu), ...
        'Lower bounds must not exceed upper bounds.');

    % Equality constraints: finite identical bounds
    isEquality = isfinite(bl) & isfinite(bu) & (bl == bu);

    Aeq = A(isEquality,:);
    beq = bl(isEquality);

    % Finite upper and lower inequality bounds
    hasUpper = ~isEquality & isfinite(bu);
    hasLower = ~isEquality & isfinite(bl);

    Aineq = [ A(hasUpper,:);
             -A(hasLower,:)];

    bineq = [ bu(hasUpper);
             -bl(hasLower)];

    options = optimoptions('quadprog', 'Algorithm','active-set', 'Display','off');

    [x,fval,exitflag,output,lambda] = quadprog(H,g,Aineq,bineq,Aeq,beq,l,u,xinit,options);

    % Solver information
    info.success = (exitflag > 0);
    info.exitflag = exitflag;
    info.fval = fval;                   % cost
    info.output = output;
    info.lambda = lambda;

end