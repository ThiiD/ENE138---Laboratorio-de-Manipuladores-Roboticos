function [q,v,a] = TrajetoriaQuintupla(t, ti, tf, qi, qf)
    dqi = 0;
    dqf = 0;
    ddqi = 0;
    ddqf = 0;
    
    A = [1  ti  ti^2   ti^3     ti^4      ti^5;
         1  tf  tf^2   tf^3     tf^4      tf^5;
         0  1   2*ti   3*ti^2   4*ti^3    5*ti^4;
         0  1   2*tf   3*tf^2   4*tf^3    5*tf^4;
         0  0   2      6*ti     12*ti^2   20*ti^3;
         0  0   2      6*tf     12*tf^2   20*tf^3
         ];
    
    B = [qi qf dqi dqf ddqi ddqf]';
    
    X = A\B;
    
    
    q = X(1) + X(2)*t + X(3)*t.^2 + X(4)*t.^3   + X(5)*t.^4     + X(6)*t.^5;
    v = 0    + X(2)   + 2*X(3)*t  + 3*X(4)*t.^2 + 4*X(5)*t.^3   + 5*X(6)*t.^4;
    a = 0    + 0      + 2*X(3)    + 6*X(4)*t    + 12*X(5)*t.^2  + 20*X(6)*t.^3;
    
end