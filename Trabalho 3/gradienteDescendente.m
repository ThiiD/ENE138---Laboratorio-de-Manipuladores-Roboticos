function [tetha] = gradienteDescendente (G ,th_begin )

%CÁLCULO CINEMÁTICA INVERSA
eta = 0.01;
ksi = 1E-9;

p(1) = 0.1*cos(th_begin(1)) + 0.1*cos(th_begin(1) + th_begin(2));
p(2) = 0.1*sin(th_begin(1)) + 0.1*sin(th_begin(1) + th_begin(2));
 
EQM = 0.5*(G - p')'* (G - p')

inter = 0;
max_inter = 5000;

while (EQM > ksi)
    
    deltaEQM = -(G - p');
    
    %Calcular o jacobiano
    J =[ -0.1*sin(th_begin(1)) - 0.1*sin(th_begin(1)+th_begin(2))   -0.1*sin(th_begin(1) + th_begin(2))
          0.1*cos(th_begin(1)) + 0.1*cos(th_begin(1)+th_begin(2))   0.1*cos(th_begin(1) + th_begin(2))]; 
   
  
    %Gradiente das juntas    
    deltatheta = pinv(J') * deltaEQM;
    th_begin = th_begin - eta* deltatheta;
    
    %Manter angulo entre 0 e 2pi   
    th_begin = mod(th_begin,2*pi);
     
%     %%Manter angulo entre -2pi e 2pi       
%     if(th_begin(1) > 2*pi)
%         th_begin(1) = th_begin(1) - 2*pi;
%     end
%     if(th_begin(2) > 2*pi)
%         th_begin(2) = th_begin(2) - 2*pi;
%     end
   
    %Aplicação da Cinematica     
    p = [ 0.1*cos(th_begin(1)) + 0.1*cos(th_begin(1) + th_begin(2));
          0.1*sin(th_begin(1)) + 0.1*sin(th_begin(1) + th_begin(2));];      
    EQM = 0.5*(G - p')'  * (G - p'); 
    inter = inter + 1;
    
   if ( inter > max_inter)
       break
   end   
end
    tetha = th_begin;
end
