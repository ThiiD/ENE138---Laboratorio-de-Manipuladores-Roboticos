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
x = [0.0];
y = [0.2];
Xd = [x ; y];

% % -------------- Definições do Projeto --------------
Katt = 3;
Krep = .00005;
episilon_0 = 0.05;
eta = 0.8;
qsi = 0.003;
iter = 0;
iter_max = 10000;
th = [0 0];

[~, X] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot_wait);
X = X(1:2);
d = sqrt((Xd' - X)*(Xd'-X)');

 while (abs(d) > qsi && iter<iter_max)
    Ftot = sub_algoritmo_campo_potencial(Katt, Krep, episilon_0, Xd, X);
    J2 = JacobianoJunta2(th);
    J1 = JacobianoJunta1(th);
    Ftot = J1*Ftot' + J2*Ftot';
    th = (th' + eta*Ftot )';

    vrep.simxSetJointPosition(clientID,j1,th(1),vrep.simx_opmode_oneshot);
    vrep.simxSetJointPosition(clientID,j2,th(2),vrep.simx_opmode_oneshot);

    [~, X] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot_wait);
    X = X(1:2);
    d = sqrt((Xd' - X)*(Xd'-X)');
    iter = iter + 1;
end

disp('--------------------------------------')
disp('Posição esperada: ')
disp(['       X = ', num2str(Xd(1)),'       Y = ', num2str(Xd(2))])
disp('Posição obtida:')
[~,position] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot_wait); 
disp(['       X = ', num2str(position(1)),'       Y = ', num2str(position(2))])
pause()