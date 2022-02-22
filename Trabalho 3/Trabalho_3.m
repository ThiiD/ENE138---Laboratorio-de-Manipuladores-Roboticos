clc, clear all;

vrep = remApi('remoteApi'); %using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19998,true,true,5000,5); %'127.0.0.1' -> IP local

[a1, j1] = vrep.simxGetObjectHandle(clientID,'Junta_1', vrep.simx_opmode_blocking);
[a2, j2] = vrep.simxGetObjectHandle(clientID,'Junta_2', vrep.simx_opmode_blocking);
[~, caneta] = vrep.simxGetObjectHandle(clientID,'Bic',  vrep.simx_opmode_blocking);

vrep.simxSetJointPosition(clientID,j1,0,vrep.simx_opmode_oneshot);
vrep.simxSetJointPosition(clientID,j2,0,vrep.simx_opmode_oneshot);

x = [0.1 0.2 0.0  0.1  0.0];
y = [0.1 0.0 0.2 -0.1 -0.2];

G = [x ; y];

it = 0;

th_begin = [0 0];


eta = 0.01;
qsi = 1E-16;

p(1) = 0.1*cos(th_begin(1)) + 0.1*cos(th_begin(1) + th_begin(2));
p(2) = 0.1*sin(th_begin(1)) + 0.1*sin(th_begin(1) + th_begin(2));


EQM = 0.5 * ( G(1:2,1) - p')' * ( G(1:2,1) - p');

for i = 1:length(G)
    while (EQM > qsi)

        gradienteEQM = -(G(1:2,i) - p');
        
        J =[ -0.1*sin(th_begin(1)) - 0.1*sin(th_begin(1)+th_begin(2))   -0.1*sin(th_begin(1) + th_begin(2))
              0.1*cos(th_begin(1)) + 0.1*cos(th_begin(1)+th_begin(2))   0.1*cos(th_begin(1) + th_begin(2))];
          
        gradienteTheta = pinv(J) * gradienteEQM;
        
        th_begin = (th_begin' - eta * gradienteTheta)';
        th_begin = mod(th_begin, 2*pi);
        p(1) = 0.1*cos(th_begin(1)) + 0.1*cos(th_begin(1) + th_begin(2));
        p(2) = 0.1*sin(th_begin(1)) + 0.1*sin(th_begin(1) + th_begin(2));
        
        EQM = 0.5 * ( G(1:2,i) - p')' * ( G(1:2,i) - p');
    end
    theta = th_begin;
    EQM = 10;
    vrep.simxSetJointPosition(clientID,j1,theta(1),vrep.simx_opmode_oneshot);
    vrep.simxSetJointPosition(clientID,j2,theta(2),vrep.simx_opmode_oneshot);
    disp('--------------------------------------')
    disp('Posição esperada: ')
    disp(['       X = ', num2str(G(1,i)),'       Y = ', num2str(G(2,i))])
    disp('Posição obtida:')
    for k = 1:2000
        [~,position] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot); 
    end
    disp(['       X = ', num2str(position(1)),'       Y = ', num2str(position(2))])
    pause()
end
