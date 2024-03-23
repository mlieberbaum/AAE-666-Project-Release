function [rXYZ, vXYZ] = orbel2rv(a, e, i, Omega, omega, th)

    muEarth = 398600.4415;

    thStar = wrapTo2Pi(th - omega);
    
    C = dcmI2R(i, Omega, th);
    
    p = a*(1 - e^2);
    r = p/(1 + e*cos(thStar));
    v = sqrt(muEarth*(2/r - 1/a));
    h = sqrt(muEarth * p);

    gamma = acos(h/(r*v));

    if thStar > pi
        gamma = -gamma;
    end
    
    rVecRth = [r, 0, 0];
    vVecRth = [v*sin(gamma), v*cos(gamma), 0];

    rXYZ = rVecRth * C';
    vXYZ = vVecRth * C';

end
