function inputData = LinearControlLaw(type)
    
    
    % Simulation parameters
    inputData.sim.tf = 100;
    inputData.sim.dt = 0.01;


    % Initial state
    inputData.x0.r = [6678137.0, 0.0, 0.0].';
    inputData.x0.v = [0.0, 7361.21593216773, 4250].';
    inputData.x0.w = [0.01, 0.01, 0.01].';
    inputData.x0.q = [0.57, 0.57, 0.57, 0.159].';
    inputData.x0.q = inputData.x0.q ./ norm(inputData.x0.q);


    % Controller parameters
    if strcmp(type, 'PD')
        inputData.ctrl.Kp = 30.0;
        inputData.ctrl.Kd = 379.2;
        inputData.ctrl.Ki = 0.0;
    elseif strcmp(type, 'PID')
        inputData.ctrl.Kp = 89.91;
        inputData.ctrl.Kd = 568.8;
        inputData.ctrl.Ki = 4.74;
    else
        error('Invalid control type');
    end
    
    inputData.ctrl.type = 'LINEAR';

    inputData.ctrl.qc = [0 ; 0 ; 0 ; 1];
    inputData.ctrl.qc = inputData.ctrl.qc ./ norm(inputData.ctrl.qc);


    % Spacecraft plant model parameters
    inputData.spacecraft.J = [1200   100  -200
                               100  2200   300
                              -200   300  3100] * 0.9;
    
end