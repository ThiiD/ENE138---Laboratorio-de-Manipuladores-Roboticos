clc, clear all;

% -------------- Definições do CoppeliaSim --------------
vrep = remApi('remoteApi'); %using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19998,true,true,5000,5); %'127.0.0.1' -> IP local

[a1, j1] = vrep.simxGetObjectHandle(clientID,'Junta_1', vrep.simx_opmode_blocking);
[a2, j2] = vrep.simxGetObjectHandle(clientID,'Junta_2', vrep.simx_opmode_blocking);
[~, caneta] = vrep.simxGetObjectHandle(clientID,'Bic',  vrep.simx_opmode_blocking);

vrep.simxSetJointPosition(clientID,j1,0,vrep.simx_opmode_oneshot);
vrep.simxSetJointPosition(clientID,j2,0,vrep.simx_opmode_oneshot);

% -------------- Definições de Posições --------------
x = [0.1 0.2 0.0  0.1  0.0];
y = [0.1 0.0 0.2 -0.1 -0.2];

G = [x ; y];
% % -------------- Definições do Projeto --------------
K = [1 0; 0 1];
iter = 0;               % numero de iterações
max_iter = 10000;        % Maximo de iterações
qsi = 1e-3;             % limite de erro
dt = 5e-2;              % Variação de Tempo
q = [0 0];
[~,Xe] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot_wait); 
Xe = Xe(1:2);
der_Xd = [0 0]';



% -------------- Algoritmo do Jacobiano Pseudo-Inverso --------------
disp('Algoritmo do Jacobiano Pseudo-Inverso')


for i = 1:length(G)
    Xd = G(1:2,i);
    erro = Xd - Xe';
    while (((abs(erro(1)) > qsi) || (abs(erro(2)) > qsi)) && (iter < max_iter))
        [~,Xe] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot_wait); 
        Xe = Xe(1:2);
        erro = G(1:2,i) - Xe';
        J_inv = JacobianoInverso(q);
        
        var_q = J_inv*(der_Xd + K*erro);
        q = q + var_q'*dt;
        vrep.simxSetJointPosition(clientID,j1,q(1),vrep.simx_opmode_oneshot);
        vrep.simxSetJointPosition(clientID,j2,q(2),vrep.simx_opmode_oneshot);
        
        
        iter = iter + 1;
    end
    disp('--------------------------------------')
    disp('Posição esperada: ')
    disp(['       X = ', num2str(Xd(1)),'       Y = ', num2str(Xd(2))])
    disp('Posição obtida:')
    [~,position] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot_wait); 
    disp(['       X = ', num2str(position(1)),'       Y = ', num2str(position(2))])
    pause()
    iter = 0;
end


% -------------- Algoritmo do Jacobiano Transposto --------------
disp('--------------------------------------------------')
disp('--------------------- // -------------------------')
disp('--------------------------------------------------')
disp('Algoritmo do Jacobiano Transposto')


[~,Xe] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot_wait); 
Xe = Xe(1:2);
dt = 5;
for i = 1:length(G)
    Xd = G(1:2,i);
    erro = Xd - Xe';
    while (((abs(erro(1)) > qsi || abs(erro(2)) > qsi)) && (iter < max_iter))
        [~,Xe] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot_wait); 
        Xe = Xe(1:2);
        erro = G(1:2,i) - Xe';
        J_trans = JacobianoTransposto(q);
        
        var_q = J_trans*K*erro;
        q = q + var_q'*dt;
        vrep.simxSetJointPosition(clientID,j1,q(1),vrep.simx_opmode_oneshot);
        vrep.simxSetJointPosition(clientID,j2,q(2),vrep.simx_opmode_oneshot);
        
        
        iter = iter + 1;
%         disp(num2str(iter));
    end
    disp('--------------------------------------')
    disp('Posição esperada: ')
    disp(['       X = ', num2str(Xd(1)),'       Y = ', num2str(Xd(2))])
    disp('Posição obtida:')
    [~,position] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot_wait); 
    disp(['       X = ', num2str(position(1)),'       Y = ', num2str(position(2))])
    pause()
    iter = 0;
end

