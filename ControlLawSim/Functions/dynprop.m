function [rOut, vOut, qOut, wOut, uOut, uActOut, ctrl] = dynprop(r, v, q, w, simIn, SC, ctrl, simFrame, uHist)
    
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
        qe = errorQuaternion(ctrl.qc, xOut(end,7:10).');
        D = ctrl.D;
        K = ctrl.K;
        Je = ctrl.Jest;
        uCtl = ctrl.mu * cross(w, Je*w) - D*w - K*qe(1:3);
        uAct = uCtl;

    elseif strcmp(ctrl.type, 'BWDISC') || strcmp(ctrl.type, 'BWDISCFULL')

        if mod(simFrame, ctrl.nZOH) == 0

            % Compute the control input at this time step.  This control input
            % is applied throughout the entire propagation to the next time
            % step, to simulate a digital controller using a zero-order hold
            w = x0(11:13);
            qe = errorQuaternion(ctrl.qc, x0(7:10));
            D = ctrl.D;
            K = ctrl.K;
            Je = ctrl.Jest;
            uCtl = ctrl.mu * cross(w, Je*w) - D*w - K*qe(1:3);

            % Implement time delay in control system
            if isfield(ctrl, 'delay') && ctrl.delay > 0
                if simFrame <= ctrl.delay
                    uAct = [0;0;0];
                else
                    uAct = uHist(simFrame - ctrl.delay, :).';
                end
            else
                uAct = uCtl;
            end

        else

            % Otherwise, use the previous control input
            uCtl = ctrl.uPrev;
            uAct = uCtl;

        end

        if strcmp(ctrl.type, 'BWDISC')
            [tOut, xOut] = ode45(@(t,x) bongWieDisc(t,x,SC,uAct), [0 simIn.dt], x0, S);
        elseif strcmp(ctrl.type, 'BWDISCFULL')
            [tOut, xOut] = ode45(@(t,x) bongWieDiscFullSim(t,x,SC,uAct), [0 simIn.dt], x0, S);
        end


    elseif strcmp(ctrl.type, 'LINEAR')

        % Compute the control input at this time step.  This control input
        % is applied throughout the entire propagation to the next time
        % step, to simulate a digital controller using a zero-order hold.
        % Use the small angle approximation to obtain the angle error.
        qErr = -errorQuaternion(ctrl.qc, x0(7:10));
        thErr = qErr(1:3) * 2;

        uP = ctrl.Kp * thErr;

        if simFrame == 0
            ctrl.thErrInt = thErr * simIn.dt;
            ctrl.thErrPrev = thErr;
            uD = zeros(3,1);
            uI = zeros(3,1);
        else
            uD = ctrl.Kd * (thErr - ctrl.thErrPrev) / simIn.dt;
            ctrl.thErrPrev = thErr;
            uI = ctrl.Ki * ctrl.thErrInt;
            ctrl.thErrInt = ctrl.thErrInt + thErr * simIn.dt;
        end

        uCtl = uP + uD + uI;
        uAct = uCtl;

        % Bong Wie Disc assumes a pre-calculated control input, so it is
        % valid to call for the linear controller simulation
        [tOut, xOut] = ode45(@(t,x) bongWieDisc(t,x,SC,uAct), [0 simIn.dt], x0, S);

    elseif strcmp(ctrl.type, 'NONE')

        [tOut, xOut] = ode45(@(t,x) noCtrl(t,x,SC), [0 simIn.dt], x0, S);
        uCtl = zeros(3,1);
        uAct = uCtl;

    else

        error('Error: Unspecified control algorithm');

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
    uOut = uCtl;
    uActOut = uAct;

end
