clc, clear all;

L1 = 0.1;
L2 = 0.1;

x = [0.1 0.10 0.02];
y = [0.1 0.04 0.12];

vrep = remApi('remoteApi'); %using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19998,true,true,5000,5); %'127.0.0.1' -> IP local

[a1, j1] = vrep.simxGetObjectHandle(clientID,'Junta_1', vrep.simx_opmode_blocking);
[a2, j2] = vrep.simxGetObjectHandle(clientID,'Junta_2', vrep.simx_opmode_blocking);
[~, caneta] = vrep.simxGetObjectHandle(clientID,'Bic',  vrep.simx_opmode_blocking);

vrep.simxSetJointPosition(clientID,j1,0,vrep.simx_opmode_oneshot);
vrep.simxSetJointPosition(clientID,j2,0,vrep.simx_opmode_oneshot);

[~,position] = vrep.simxGetObjectPosition(clientID,caneta,j1,vrep.simx_opmode_blocking);

for i = 1:length(x)
    cO2 = ((x(i)^2 + y(i)^2) - (L1^2 + L2^2))/(2*L1*L2);
    if abs(cO2) > 1
        disp('Ponto inalcançavel')
        continue
    end
    sO2 = [-sqrt(1-cO2^2) sqrt(1-cO2^2)];
    k1 = L1 + L2*cO2;
    k2 = L2.*sO2;
    r = sqrt(k1^2 + k2.^2);
    O2 = atan2(sO2, cO2);
    O1 = atan2(y(i)./r, x(i)./r) - atan2(k2,k1);
    for l = 1:2
        vrep.simxSetJointPosition(clientID,j1,O1(l),vrep.simx_opmode_oneshot);
        vrep.simxSetJointPosition(clientID,j2,O2(l),vrep.simx_opmode_oneshot);
        disp('--------------------------------------')
        if l ==1
            disp('elbow up')
        else
            disp('elbow down')
        end
        disp('Posição esperada: ')
        disp(['       X = ', num2str(x(i)),'       Y = ', num2str(y(i))])
        disp('Posição obtida:')
        for k = 1:2000
            [~,position] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot); 
        end
        disp(['       X = ', num2str(position(1)),'       Y = ', num2str(position(2))])
        pause()
        
    end
end
