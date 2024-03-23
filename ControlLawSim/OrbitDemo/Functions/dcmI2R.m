function C = dcmI2R(i, Omega, th)
    
    cO = cos(Omega);
    sO = sin(Omega);
    cth = cos(th);
    sth = sin(th);
    ci = cos(i);
    si = sin(i);
    
    C = [cO*cth - sO*ci*sth      -cO*sth - sO*ci*cth     sO*si
         sO*cth + cO*ci*sth      -sO*sth + cO*ci*cth    -cO*si
         si*sth                   si*cth                 ci   ];

end