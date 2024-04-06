function data = ControlLawSim(inputData)


    %% INPUT DATA STRUCTURES
    simIn = inputData.sim;
    x0 = inputData.x0;
    SC = inputData.spacecraft;
    ctrl = inputData.ctrl;


    %% INITIALIZE OUTPUT DATA ARRAYS
    data.t = (0 : simIn.dt : simIn.tf).';
    npts = numel(data.t);

    data.r = nan(npts, 3);
    data.v = nan(npts, 3);
    data.q = nan(npts, 4);
    data.w = nan(npts, 3);
    data.u = nan(npts, 3);
    data.ua = nan(npts, 3);

    % Set initial conditions to output arrays
    data.r(1,:) = x0.r.';
    data.v(1,:) = x0.v.';
    data.q(1,:) = x0.q.';
    data.w(1,:) = x0.w.';
    data.u(1,:) = [0 0 0];


    %% MAIN SIMULATION LOOP

    simFrame = 0;

    for idx = 1 : npts-1

        % Get state
        r = data.r(idx,:).';
        v = data.v(idx,:).';
        q = data.q(idx,:).';
        w = data.w(idx,:).';

        % Propagate state
        [rOut, vOut, qOut, wOut, uOut, uActOut] = dynprop(r, v, q, w, simIn, SC, ctrl, simFrame, data.u);

        % Set to output array
        data.r(idx+1,:) = rOut.';
        data.v(idx+1,:) = vOut.';
        data.q(idx+1,:) = qOut.';
        data.w(idx+1,:) = wOut.';
        data.u(idx+1,:) = uOut.';
        data.ua(idx+1,:) = uActOut.';

        % Set control input to ctrl structure (used for zero order hold)
        ctrl.uPrev = uOut.';

        % Increment sim frame
        simFrame = simFrame + 1;

    end



end
