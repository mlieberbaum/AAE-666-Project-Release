function [rOut, vOut, qOut, wOut, uOut] = dynprop(r, v, q, w, simIn, SC, ctrl)
    
    % Set up initial state vector
    x0 = [r ; v ; q ; w];

    % Set up ODE45 solver
    S = odeset('RelTol', 1.0e-12, 'AbsTol', 1.0e-12);

    % Propagate simulation, dependent on control law
    if strcmp(ctrl.type, 'BWCONT')

        [tOut, xOut] = ode45(@(t,x) bongWieCont(t,x,SC,ctrl), [0 simIn.dt], x0, S);

        % Compute control input at the i+1 step
        % u = w X (Je w) - D*w - K*qe
        w = xOut(end,11:13).';
        qe = xOut(end,7:9).';
        D = ctrl.D;
        K = ctrl.K;
        Je = ctrl.Jest;
        u = cross(w, Je*w) - D*w - K*qe;

    elseif strcmp(ctrl.type, 'NONE')

        [tOut, xOut] = ode45(@(t,x) noCtrl(t,x,SC), [0 simIn.dt], x0, S);
        u = zeros(3,1);

    end

    % Check output time
    if abs(tOut(end) - simIn.dt) > 1.0e-12
        error('Error propagating step');
    end

    % Set output variables
    xOut = xOut(end,:);
    rOut = xOut(1:3).';
    vOut = xOut(4:6).';
    qOut = xOut(7:10).';
    wOut = xOut(11:13).';
    uOut = u;

end
