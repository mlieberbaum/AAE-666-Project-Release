function xOut = noCtrl(t,x,SC)
    
    % Earth gravitational constant
    muEarth = 398600.4415 * 1e9;

    % Inertias
    J11 = SC.J(1,1);
    J22 = SC.J(2,2);
    J33 = SC.J(3,3);
    J12 = SC.J(1,2);
    J13 = SC.J(1,3);
    J23 = SC.J(2,3);

    % Set up dynamic equations (uncontrolled dynamics)
    %   x(1)  = r1
    %   x(2)  = r2
    %   x(3)  = r3
    %   x(4)  = v1
    %   x(5)  = v2
    %   x(6)  = v3
    %   x(7)  = q1
    %   x(8)  = q2
    %   x(9)  = q3
    %   x(10) = q4
    %   x(11) = w1
    %   x(12) = w2
    %   x(13) = w3

    xOut = [ x(4)
             x(5)
             x(6)
            -muEarth*x(1) / (x(1)*x(1) + x(2)*x(2) + x(3)*x(3))^(3/2)
            -muEarth*x(2) / (x(1)*x(1) + x(2)*x(2) + x(3)*x(3))^(3/2)
            -muEarth*x(3) / (x(1)*x(1) + x(2)*x(2) + x(3)*x(3))^(3/2)
             0.5 * ( x(8)*x(13) - x(9)*x(12) + x(10)*x(11))
             0.5 * ( x(9)*x(11) - x(7)*x(13) + x(10)*x(12))
             0.5 * ( x(7)*x(12) - x(8)*x(11) + x(10)*x(13))
             0.5 * (-x(7)*x(11) - x(8)*x(12) -  x(9)*x(13))
             ((x(13)*(J12*x(11) + J22*x(12) + J23*x(13)) - x(12)*(J13*x(11) + J23*x(12) + J33*x(13)))*(J23^2 - J22*J33))/(J33*J12^2 - 2*J12*J13*J23 + J22*J13^2 + J11*J23^2 - J11*J22*J33) - ((J12*J23 - J13*J22)*(x(12)*(J11*x(11) + J12*x(12) + J13*x(13)) - x(11)*(J12*x(11) + J22*x(12) + J23*x(13))))/(J33*J12^2 - 2*J12*J13*J23 + J22*J13^2 + J11*J23^2 - J11*J22*J33) + ((J13*J23 - J12*J33)*(x(13)*(J11*x(11) + J12*x(12) + J13*x(13)) - x(11)*(J13*x(11) + J23*x(12) + J33*x(13))))/(J33*J12^2 - 2*J12*J13*J23 + J22*J13^2 + J11*J23^2 - J11*J22*J33)
            -((x(13)*(J11*x(11) + J12*x(12) + J13*x(13)) - x(11)*(J13*x(11) + J23*x(12) + J33*x(13)))*(J13^2 - J11*J33))/(J33*J12^2 - 2*J12*J13*J23 + J22*J13^2 + J11*J23^2 - J11*J22*J33) - ((J12*J13 - J11*J23)*(x(12)*(J11*x(11) + J12*x(12) + J13*x(13)) - x(11)*(J12*x(11) + J22*x(12) + J23*x(13))))/(J33*J12^2 - 2*J12*J13*J23 + J22*J13^2 + J11*J23^2 - J11*J22*J33) - ((J13*J23 - J12*J33)*(x(13)*(J12*x(11) + J22*x(12) + J23*x(13)) - x(12)*(J13*x(11) + J23*x(12) + J33*x(13))))/(J33*J12^2 - 2*J12*J13*J23 + J22*J13^2 + J11*J23^2 - J11*J22*J33)
             ((x(12)*(J11*x(11) + J12*x(12) + J13*x(13)) - x(11)*(J12*x(11) + J22*x(12) + J23*x(13)))*(J12^2 - J11*J22))/(J33*J12^2 - 2*J12*J13*J23 + J22*J13^2 + J11*J23^2 - J11*J22*J33) + ((J12*J13 - J11*J23)*(x(13)*(J11*x(11) + J12*x(12) + J13*x(13)) - x(11)*(J13*x(11) + J23*x(12) + J33*x(13))))/(J33*J12^2 - 2*J12*J13*J23 + J22*J13^2 + J11*J23^2 - J11*J22*J33) - ((J12*J23 - J13*J22)*(x(13)*(J12*x(11) + J22*x(12) + J23*x(13)) - x(12)*(J13*x(11) + J23*x(12) + J33*x(13))))/(J33*J12^2 - 2*J12*J13*J23 + J22*J13^2 + J11*J23^2 - J11*J22*J33)
             ];


end