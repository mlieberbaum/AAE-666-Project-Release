function qe = errorQuaternion(q, qc)
    
    qcinv = qc;
    qcinv(4) = -qcinv(4);

    qe = quaternionProduct(qcinv, q);

end