clc; clear all
theta1 = deg2rad([30 45 60 90 -90]);
theta2 = deg2rad([30 45 60 90 -90]);
alpha1 = 0.1;
alpha2 = 0.1;

 
disp('--------------------------------------------------------')

for i = 1:length(theta1)
    A1 = [cos(theta1(i))   -sin(theta1(i))   0     alpha1*cos(theta1(i));
         sin(theta1(i))   cos(theta1(i))     0     alpha1*sin(theta1(i));
                0                0           1              0;
                0                0           0              1          ];
      
    A2 = [cos(theta2(i))   -sin(theta2(i))   0     alpha2*cos(theta2(i));
         sin(theta2(i))   cos(theta2(i))     0     alpha2*sin(theta2(i));
                0                0           1              0;
                0                0           0              1          ] ;
        
    TH = A1*A2;
    
    disp(['Posição esperada para theta1 = ',num2str(theta1(i)),'  e   theta 2 = ', num2str(theta2(i)), '  :'])
    disp(['       X0 = ', num2str(TH(1,4)), '      Y0 = ', num2str(TH(2,4))])
    
    
    vrep.simxSetJointPosition(clientID,j1,theta1(i),vrep.simx_opmode_oneshot);
    vrep.simxSetJointPosition(clientID,j2,theta2(i),vrep.simx_opmode_oneshot);
    pause(.5)
         
    for k = 1:2000
        [~,position] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot); 
    end

    disp(['Posição do robo:'])
    disp(['       X0 = ',num2str(position(1)),'       Y0 = ', num2str(position(2))])
    pause()
    disp('--------------------------------------------------------')
end

