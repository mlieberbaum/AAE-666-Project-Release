% Wie, B., Weiss, H., and Arapostathis, A., "Quaternion Feedback Regulator 
% for Spacecraft Eigenaxis Rotations,” Journal of Guidance, Control, and 
% Dynamics, Vol. 12, No. 3, 1989, pp. 375-380.	doi: 10.2514/3.20418


% Note: This uses the transpose of Omega in Eq. 6, because Eq. 6
% specifically is to compute the error quaternion, not the quaternion
% product

function q3 = quaternionProduct(q1, q2)
    
    Q = [ q1(4),  -q1(3),   q1(2),  q1(1)
          q1(3),   q1(4),  -q1(1),  q1(2)
         -q1(2),   q1(1),   q1(4),  q1(3)
         -q1(1),  -q1(2),  -q1(3),  q1(4) ];

    q3 = Q * q2;

end