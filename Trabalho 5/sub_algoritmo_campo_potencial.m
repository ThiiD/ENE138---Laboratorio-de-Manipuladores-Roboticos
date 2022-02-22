function [Ftot] = sub_algoritmo_campo_potencial(Katt, Krep, episilon_0,  Xd, X )

Fatt = Katt .* (Xd' - X);


% -------------- Obstaculos --------------
% O primeiro atinge o objetivo
% O segundo não atinge o objetivo
obst = [0.0684 ; 0.1009 ];  % ATINGE O OBJETIVO
%obst = [0.06727775960005736 0.07 0.08 0.09 0.1 0.11 0.12 0.13 0.14 0.15 0.16 0.17 0.18 0.19 0.2; 0.06727775960005736 0.07 0.08 0.09 0.1 0.11 0.12 0.13 0.14 0.15 0.16 0.17 0.18 0.19 0.2 ]; % NÃO ATINGE O OBJETIVO


Frep = 0;

for i = 1:width(obst)
    eps = sqrt((obst(1:2,i)' - X)*(obst(1:2,i)'-X)');
    if eps > episilon_0
       continue 
    end
    Frep = Frep + (Krep / eps^3 * (1/episilon_0 - 1/eps) * (obst(1:2,i)' - X)); 
end

Ftot = Fatt + Frep;
end