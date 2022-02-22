function [q,v,a] = TrajetoriaCubica(t, ti, tf, qi, qf)
    dqi = 0;
    dqf = 0;
    
    A = [1 ti ti^2 ti^3;
         1 tf tf^2 tf^3;
         0 1  2*ti 3*ti^2;
         0 1  2*tf 3*tf^2];
     
    B = [qi qf dqi dqf]';
    
    X = A\B;
    
    q = 1*X(1) + X(2)*t + X(3)*t.^2 + X(4)*t.^3;
    v = 0*X(1) + X(2)*1 + 2*X(3)*t  + 3*X(4)*t.^2;   
    a = 0*X(1) + 0*X(2) + 2*X(3)    + 6*X(4)*t;
end