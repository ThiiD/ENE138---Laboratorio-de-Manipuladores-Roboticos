function [J] = JacobianoJunta1(th)

J =[ -0.1*sin(th(1))   0;
      0.1*cos(th(1))   0];

end
