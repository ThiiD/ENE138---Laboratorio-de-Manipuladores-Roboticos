clc, close all; clear all;
% -------------------------------------------------
% ----------- Definições do CoppeliaSim -----------
% -------------------------------------------------

vrep = remApi('remoteApi'); %using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19998,true,true,5000,5); %'127.0.0.1' -> IP local

[a1, j1] = vrep.simxGetObjectHandle(clientID,'Junta_1', vrep.simx_opmode_blocking);
[a2, j2] = vrep.simxGetObjectHandle(clientID,'Junta_2', vrep.simx_opmode_blocking);
[~, caneta] = vrep.simxGetObjectHandle(clientID,'Bic',  vrep.simx_opmode_blocking);

vrep.simxSetJointPosition(clientID,j1,0,vrep.simx_opmode_oneshot);
vrep.simxSetJointPosition(clientID,j2,0,vrep.simx_opmode_oneshot);

% -------------------------------------------------
% -------------- Definições Iniciais --------------
% -------------------------------------------------
L1 = 0.1;
L2 = 0.1;
% ----------------- Ponto Inicial -----------------
xi = 0.2;
yi = 0.0;
% ------------------ Ponto Final ------------------
xf = 0.0;
yf = 0.2;

% -------------- Parametros Iniciais --------------
dt = 0.1;
t = 0:dt:5;
ti = t(1);
tf = t(end);

x = linspace(xi, xf, length(t));
y = linspace(yi, yf, length(t));


% -------------------------------------------------
% ----------------- Estrategia 1 ------------------
% -------------- Trajetoria Cubica ----------------
% -------------------------------------------------
[Pxc, Vxc, Axc] = TrajetoriaCubica(t, ti, tf, xi, xf);
[Pyc, Vyc, Ayc] = TrajetoriaCubica(t, ti, tf, yi, yf);

tic;
i=1;
figure(3);
while(i<=length(t))
   if(toc >= dt) % se passou um tempo >= dt
       plot(Pxc(i), Pyc(i), 'ro');
       grid on;
       xlabel('x');
       ylabel('y');
       title('Movimentação do robô - Trajetoria Cubica');
       xlim([0 0.2]);
       ylim([0 0.2]);
       th = CinematicaInversa(L1, L2, Pxc(i), Pyc(i));
       vrep.simxSetJointPosition(clientID,j1,th(1),vrep.simx_opmode_oneshot);
       vrep.simxSetJointPosition(clientID,j2,th(2),vrep.simx_opmode_oneshot);
       hold on;
       drawnow;
       i = i+1;
       tic;
   end
end

% -------- Resultados - Trajetoria Cubica ---------
figure(4);
subplot(3,1,1);
suptitle('Resultados - Trajetoria Cubica');
plot(t,Pxc, 'linewidth', 2);
hold on;
plot(t,Pyc, 'linewidth', 2);
grid on;
legend('X', 'Y');
xlabel('Tempo [s]');
ylabel('Posição [mm]');

subplot(3,1,2);
plot(t, Vxc, 'linewidth', 2);
hold on;
plot(t, Vyc, 'linewidth', 2);
grid on;
legend('X', 'Y')
xlabel('Tempo [s]');
ylabel('Velocidade [mm/s]');

subplot(3,1,3)
plot(t, Axc, 'linewidth', 2);
hold on;
plot(t, Ayc, 'linewidth', 2);
grid on;
legend('X', 'Y');
xlabel('Tempo [s]');
ylabel('Aceleração [mm/s^2]');


[~, X] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot_wait);
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%% Trajetoria Cubica %%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('Posição esperada: ')
disp(['       X = ', num2str(Pxc(end)),'       Y = ', num2str(Pyc(end))])
disp('Posição obtida:')
disp(['       X = ', num2str(X(1)),'       Y = ', num2str(X(2))])

% -------------------------------------------------
% ----------------- Estrategia 2 ------------------
% ------------- Trajetoria Quintupla --------------
% -------------------------------------------------
xi = X(1);
yi = X(2);
xf = 0.1;
yf = 0.1;
[Pxq, Vxq, Axq] = TrajetoriaQuintupla(t, ti, tf, xi, xf);
[Pyq, Vyq, Ayq] = TrajetoriaQuintupla(t, ti, tf, yi, yf);

tic;
i=1;
figure(5)
while(i<=length(t))
   if(toc >= dt) % se passou um tempo >= dt
       plot(Pxq(i), Pyq(i), 'ro');
       grid on;
       xlabel('x');
       ylabel('y');
       title('Movimentação do robô - Trajetoria Quintupla');
       xlim([0 0.2]);
       ylim([0 0.2]);
       th = CinematicaInversa(L1, L2, Pxq(i), Pyq(i));
       vrep.simxSetJointPosition(clientID,j1,th(1),vrep.simx_opmode_oneshot);
       vrep.simxSetJointPosition(clientID,j2,th(2),vrep.simx_opmode_oneshot);
       hold on;
       drawnow;
       i = i+1;
       tic;
   end
end

figure(6);
subplot(3,1,1);
suptitle('Resultados - Trajetoria Quintupla');
plot(t,Pxq, 'linewidth', 2);
hold on;
plot(t,Pyq, 'linewidth', 2);
grid on;
legend('X', 'Y');
xlabel('Tempo [s]');
ylabel('Posição [mm]');

subplot(3,1,2);
plot(t, Vxq, 'linewidth', 2);
hold on;
plot(t, Vyq, 'linewidth', 2);
grid on;
legend('X', 'Y');
xlabel('Tempo [s]');
ylabel('Velocidade [mm/s]');

subplot(3,1,3);
plot(t, Axq, 'linewidth', 2);
hold on;
plot(t, Ayq, 'linewidth', 2);
grid on;
legend('X', 'Y');
xlabel('Tempo [s]');
ylabel('Aceleração [mm/s^2]');


[~, X] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot_wait);
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%%%%%%%%%%%%%%%%%%%%%%% Trajetoria Quintupla %%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('Posição esperada: ')
disp(['       X = ', num2str(xf),'       Y = ', num2str(yf)])
disp('Posição obtida:')
disp(['       X = ', num2str(X(1)),'       Y = ', num2str(X(2))])

% -------------------------------------------------
% ----------------- Estrategia 3 ------------------
% ------------- Trajetoria por LSPB ---------------
% -------------------------------------------------
tb = 1.5;
Vmax = 0.028;

xi = X(1);
yi = X(2);
xf = 0.05;
yf = -0.15;

[Px3, Vx3, Ax3] = LSPB(t, ti, tf, tb, Vmax, xi, xf);
[Py3, Vy3, Ay3] = LSPB(t, ti, tf, tb, Vmax, yi, yf);

tic;
i=1;
figure(7)
while(i<=length(t))
   if(toc >= dt) % se passou um tempo >= dt
       plot(Px3(i), Py3(i), 'ro');
       grid on;
       xlabel('x');
       ylabel('y');
       title('Movimentação do robô - Trajetoria LSPB');
       xlim([0 0.2]);
       ylim([-0.2 0.2]);
       th = CinematicaInversa(L1, L2, Px3(i), Py3(i));
       vrep.simxSetJointPosition(clientID,j1,th(1),vrep.simx_opmode_oneshot);
       vrep.simxSetJointPosition(clientID,j2,th(2),vrep.simx_opmode_oneshot);
       hold on;
       drawnow;
       i = i+1;
       tic;
   end
end

figure(8);
subplot(3,1,1);
suptitle('Resultados - Trajetoria LSBP');
plot(t,Px3, 'linewidth', 2);
hold on;
plot(t,Py3, 'linewidth', 2);
grid on;
legend('X', 'Y');
xlabel('Tempo [s]');
ylabel('Posição [mm]');

subplot(3,1,2);
plot(t, Vx3, 'linewidth', 2);
hold on;
plot(t, Vy3, 'linewidth', 2);
grid on;
legend('X', 'Y');
xlabel('Tempo [s]');
ylabel('Velocidade [mm/s]');

subplot(3,1,3);
plot(t, Ax3, 'linewidth', 2);
hold on;
plot(t, Ay3, 'linewidth', 2);
grid on;
legend('X', 'Y');
xlabel('Tempo [s]');
ylabel('Aceleração [mm/s^2]');


[~, X] = vrep.simxGetObjectPosition(clientID,caneta,-1,vrep.simx_opmode_oneshot_wait);
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%%%%%%%%%%%%%%%%%%%%%%% Trajetoria Quintupla %%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('Posição esperada: ')
disp(['       X = ', num2str(xf),'       Y = ', num2str(yf)])
disp('Posição obtida:')
disp(['       X = ', num2str(X(1)),'       Y = ', num2str(X(2))])