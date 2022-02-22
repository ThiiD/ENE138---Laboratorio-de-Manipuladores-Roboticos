function [q,v,a] = LSPB(t, ti, tf, tb, V, qi, qf)    
 
      a = V/tb;
      q = ones(1,51)
      v = ones(1,51)
      a1 = ones(1,51)
      
      a = abs((qf-qi)/((tf-tb)*tb))
      V = a*tb
      
      for i = 1:length(t)
         if t(i) <= tb
            q(i) = qi + a/2*t(i)^2*((qf - qi)/abs(qf-qi))
            v(i) = a*t(i)*((qf - qi)/abs(qf-qi))
            a1(i) = a*((qf - qi)/abs(qf-qi))
            
         elseif t(i) > tb && t(i) <= tf-tb
            %q(i) = ((qf + qi + V*tb)/2 + V*t(i))
            %q(i) = ((2*qi+a*tb^2)/2 + V*(t(i) - tb))*((qf - qi)/abs(qf-qi))
            qi2 = qi + a/2*tb^2*((qf - qi)/abs(qf-qi))
            q(i) = qi2 + (V*(t(i) - tb) * (qf - qi2)/abs(qf-qi2))
            v(i) = V *((qf - qi)/abs(qf-qi))
            a1(i) = 0
         else
             qi3 = qi2 + V*(tf-2*tb)*(qf - qi2)/abs(qf-qi2)
             taux = tf - tb
             %q(i) = qf - (a*tf^2)/2 + a*tf*t(i) - (a/2)*t(i)^2
             q(i) = qi3 + (V*(t(i)-taux) - a*(t(i)-taux)^2/2)*(qf - qi3)/abs(qf-qi3)
             v(i) = (a*tf - a*t(i))*(qf - qi3)/abs(qf-qi3)
             a1(i) = -a*(qf - qi3)/abs(qf-qi3)
         end
         
         
      end


    a = a1










end