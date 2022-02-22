function [J] = JacobianoJunta2(th)

J =[ -0.1*sin(th(1)) - 0.1*sin(th(1)+th(2))   -0.1*sin(th(1) + th(2))
      0.1*cos(th(1)) + 0.1*cos(th(1)+th(2))    0.1*cos(th(1) + th(2))];

end

