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

    % Set initial conditions to output arrays
    data.r(1,:) = x0.r.';
    data.v(1,:) = x0.v.';
    data.q(1,:) = x0.q.';
    data.w(1,:) = x0.w.';
    data.u(1,:) = [0 0 0];


    %% MAIN SIMULATION LOOP
    for idx = 1 : npts-1

        % Get state
        r = data.r(idx,:).';
        v = data.v(idx,:).';
        q = data.q(idx,:).';
        w = data.w(idx,:).';

        % Propagate state
        [rOut, vOut, qOut, wOut, uOut] = dynprop(r, v, q, w, simIn, SC, ctrl);

        % Set to output array
        data.r(idx+1,:) = rOut.';
        data.v(idx+1,:) = vOut.';
        data.q(idx+1,:) = qOut.';
        data.w(idx+1,:) = wOut.';
        data.u(idx+1,:) = uOut.';

    end



end
