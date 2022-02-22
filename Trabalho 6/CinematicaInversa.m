function [th] = CinematicaInversa(L1, L2, xf, yf)

    cO2 = ((xf^2 + yf^2) - (L1^2 + L2^2))/(2*L1*L2);
    if abs(cO2) > 1
        disp('Ponto inalcançavel')
        return
    end
    sO2 = [-sqrt(1-cO2^2) sqrt(1-cO2^2)];
    k1 = L1 + L2*cO2;
    k2 = L2*sO2;
    r = sqrt(k1.^2 + k2.^2);
    O2 = atan2(sO2, cO2);
    O1 = atan2(yf./r, xf./r) - atan2(k2,k1);
    th = [O1(1),-O2(2)];
end