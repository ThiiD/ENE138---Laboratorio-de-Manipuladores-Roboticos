clear; clc; close all
% Variáveis simbólicas D-H:

syms d theta a alpha real
% Matriz de translação em x
Tx = [1 0 0 a
0 1 0 0
0 0 1 0
0 0 0 1];

% Matriz de translação em z
Tz = [1 0 0 0
0 1 0 0
0 0 1 d
0 0 0 1];

% Matriz de rotação em x
Rx = [1 0 0 0
0 cos(alpha) -sin(alpha) 0
0 sin(alpha) cos(alpha) 0
0 0 0 1];

% Matriz de rotação em z
Rz = [cos(theta) -sin(theta) 0 0
sin(theta) cos(theta) 0 0
0 0 1 0
0 0 0 1];

% Matriz D-H:
A = simplify(Tz * Rz * Tx * Rx);
A

%Robô Planar 2DoF:
% Tabela D-H:
% |=====================================|
% | Ai | d | theta | a | alpha |
% |----|-----|---------|------|---------|
% | 1 | 0 | th1* | l1 | 0 |
% | 2 | 0 | th2* | l2 | 0 |
% |=====================================|
% Variáveis simbólicas do robô em questão:
syms th1 th2 l1 l2 real
% Frame {0}:
F0 = eye(4);
% Transformação D-H do frame {0} para o frame {1}:
% Primeira transformação D-H
A1 = subs(A , [d theta a alpha] , [0 th1 l1 0]);
% Frame {1}:
F1 = simplify(F0 * A1); % ou, simplesmente, F1 = A1;
% Transformação D-H do frame {1} para o frame {2}:
% Segunda transformação D-H
A2 = subs(A , [d theta a alpha] , [0 th2 l2 0]);
% Frame {2}:
F2 = simplify(F1 * A2); % ou, F2 = simplify(A1 * A2);

vrep = remApi('remoteApi'); %using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19998,true,true,5000,5); %'127.0.0.1' -> IP local

[a1, j1] = vrep.simxGetObjectHandle(clientID,'Junta_1', vrep.simx_opmode_blocking);
[a2, j2] = vrep.simxGetObjectHandle(clientID,'Junta_2', vrep.simx_opmode_blocking);
[~, caneta] = vrep.simxGetObjectHandle(clientID,'Bic',  vrep.simx_opmode_blocking);

vrep.simxSetJointPosition(clientID,j1,0,vrep.simx_opmode_oneshot);
vrep.simxSetJointPosition(clientID,j2,0,vrep.simx_opmode_oneshot);

[~,position] = vrep.simxGetObjectPosition(clientID,caneta,j1,vrep.simx_opmode_blocking);

position 


% ----
% Plot
% ----
% Comprimento dos links (mude os valores para resultados diferentes)
L1 = 0.1;
L2 = 0.1;
% Ângulos das juntas (mude os valores para resultados diferentes)
%TH1 = deg2rad(30);
%TH2 = deg2rad(30);
for i = 1:90
   TH1 = i*pi/180 ;
   TH2 = i*0.5*pi/180;
    vrep.simxSetJointPosition(clientID,j1,TH1,vrep.simx_opmode_oneshot);
    vrep.simxSetJointPosition(clientID,j2,TH2,vrep.simx_opmode_oneshot);
    [~,position] = vrep.simxGetObjectPosition(clientID,caneta,j1,vrep.simx_opmode_oneshot);
    pause(0.05)
end



% Frame {0} numérico
f0 = F0;
% Frame {1} numérico
f1 = double(subs(F1 , [l1 th1] , [L1 TH1]));
% Frame {2} numérico
f2 = double(subs(F2 , [l1 l2 th1 th2] , [L1 L2 TH1 TH2]));
% Ligação do frame {0} ao frame {1}
plot3([f0(1,4) f1(1,4)] , [f0(2,4) f1(2,4)] , [f0(3,4) f1(3,4)] , 'y' , 'linewidth' , 4)
axis equal; xlabel('x'); ylabel('y'); zlabel('z'); grid on; hold on;
% Ligação do frame {1} ao frame {2}
plot3([f1(1,4) f2(1,4)] , [f1(2,4) f2(2,4)] , [f1(3,4) f2(3,4)] , 'y' , 'linewidth' , 4)
% FRAME {0}
% Origem do frame {0}
plot3(f0(1,4) , f0(2,4) , f0(3,4) , 'ok' , 'linewidth' , 2 , 'markersize' , 12)
% Eixo-x do frame {0}
plot3([f0(1,4) f0(1,4)+f0(1,1)] , [f0(2,4) f0(2,4)+f0(2,1)] , [f0(3,4) f0(3,4)+f0(3,1)] , 'b' , 'linewidth' , 2)
text(f0(1,4)+f0(1,1) , f0(2,4)+f0(2,1) , f0(3,4)+f0(3,1) , 'x_{\{0\}}')
% Eixo-y do frame {0}
plot3([f0(1,4) f0(1,4)+f0(1,2)] , [f0(2,4) f0(2,4)+f0(2,2)] , [f0(3,4) f0(3,4)+f0(3,2)] , 'r' , 'linewidth' , 2)
text(f0(1,4)+f0(1,2) , f0(2,4)+f0(2,2) , f0(3,4)+f0(3,2) , 'y_{\{0\}}')
% Eixo-z do frame {0}
plot3([f0(1,4) f0(1,4)+f0(1,3)] , [f0(2,4) f0(2,4)+f0(2,3)] , [f0(3,4) f0(3,4)+f0(3,3)] , 'g' , 'linewidth' , 2)
text(f0(1,4)+f0(1,3) , f0(2,4)+f0(2,3) , f0(3,4)+f0(3,3) , 'z_{\{0\}}')


% FRAME {1}
% Origem do frame {1}
plot3(f1(1,4) , f1(2,4) , f1(3,4) , 'ok' , 'linewidth' , 2 , 'markersize' , 12)
% Eixo-x do frame {1}
plot3([f1(1,4) f1(1,4)+f1(1,1)] , [f1(2,4) f1(2,4)+f1(2,1)] , [f1(3,4) f1(3,4)+f1(3,1)] , 'b' , 'linewidth' , 2)
text(f1(1,4)+f1(1,1) , f1(2,4)+f1(2,1) , f1(3,4)+f1(3,1) , 'x_{\{1\}}')
% Eixo-y do frame {1}
plot3([f1(1,4) f1(1,4)+f1(1,2)] , [f1(2,4) f1(2,4)+f1(2,2)] , [f1(3,4) f1(3,4)+f1(3,2)] , 'r' , 'linewidth' , 2)
text(f1(1,4)+f1(1,2) , f1(2,4)+f1(2,2) , f1(3,4)+f1(3,2) , 'y_{\{1\}}')
% Eixo-z do frame {1}
plot3([f1(1,4) f1(1,4)+f1(1,3)] , [f1(2,4) f1(2,4)+f1(2,3)] , [f1(3,4) f1(3,4)+f1(3,3)] , 'g' , 'linewidth' , 2)
text(f1(1,4)+f1(1,3) , f1(2,4)+f1(2,3) , f1(3,4)+f1(3,3) , 'z_{\{1\}}')

% FRAME {2}
% Origem do frame {2}
plot3(f2(1,4) , f2(2,4) , f2(3,4) , 'ok' , 'linewidth' , 2 , 'markersize' , 12)
% Eixo-x do frame {2}
plot3([f2(1,4) f2(1,4)+f2(1,1)] , [f2(2,4) f2(2,4)+f2(2,1)] , [f2(3,4) f2(3,4)+f2(3,1)] , 'b' , 'linewidth' , 2)
text(f2(1,4)+f2(1,1) , f2(2,4)+f2(2,1) , f2(3,4)+f2(3,1) , 'x_{\{2\}}')
% Eixo-y do frame {2}
plot3([f2(1,4) f2(1,4)+f2(1,2)] , [f2(2,4) f2(2,4)+f2(2,2)] , [f2(3,4) f2(3,4)+f2(3,2)] , 'r' , 'linewidth' , 2)
text(f2(1,4)+f2(1,2) , f2(2,4)+f2(2,2) , f2(3,4)+f2(3,2) , 'y_{\{2\}}')
% Eixo-z do frame {2}
plot3([f2(1,4) f2(1,4)+f2(1,3)] , [f2(2,4) f2(2,4)+f2(2,3)] , [f2(3,4) f2(3,4)+f2(3,3)] , 'g' , 'linewidth' , 2)
text(f2(1,4)+f2(1,3) , f2(2,4)+f2(2,3) , f2(3,4)+f2(3,3) , 'z_{\{2\}}')
hold off;
title(sprintf('End-effector: x = %.2f, y = %.2f, z = %.2f' , f2(1,4) , f2(2,4) , f2(3,4)));
drawnow;
