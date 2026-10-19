%% Cálculo de Ganancias del Observador (Método place + Bessel)
clear; clc;

%% 1. Definición de la secuencia de polos
ts = 0.25;   % tiempo de establecimiento (3 a 5 veces más rápido que el regulador)
% Polos de Bessel de 2do orden escalados por ts
p_bessel = [-4.053+2.34j, -4.053-2.34j] / ts;

fprintf('Diseñando observador para Ts = %.3f s\n', ts);
fprintf('Polos deseados: %.3f + %.3fj  y  %.3f - %.3fj\n\n', ...
    real(p_bessel(1)), imag(p_bessel(1)), real(p_bessel(2)), imag(p_bessel(2)));

%% 2. Canal de la Barra (Posición y Velocidad)
% El modelo de un integrador doble (posición y velocidad) es:
% A = [0 1; 0 0], C = [1 0]
A_barra = [0 1; 0 0];
C_barra = [1 0];

% Cálculo de la ganancia L usando place (transponiendo A y C)
L_barra = place(A_barra', C_barra', p_bessel)';

lb1 = L_barra(1); % Ganancia de corrección de posición
lb2 = L_barra(2); % Ganancia de corrección de velocidad

%% 3. Canal del Disco (Posición y Velocidad)
% Para el disco, la dinámica base es similar (integrador doble).
% Nota: El código C incluye la fricción (rho*q2) en la predicción, 
% pero para el cálculo de la ganancia de corrección usamos el mismo 
% integrador doble porque queremos que el error converja con los polos de Bessel.
A_disco = [0 1; 0 0];
C_disco = [1 0];

% Usamos los mismos polos para el disco (podes hacerlo más lento si el ruido molesta)
L_disco = place(A_disco', C_disco', p_bessel)';

ld1 = L_disco(1);
ld2 = L_disco(2);

%% 4. Mostrar Resultados
fprintf('--- Ganancias Calculadas ---\n');
fprintf('Barra: lb1 = %.4f, lb2 = %.4f\n', lb1, lb2);
fprintf('Disco: ld1 = %.4f, ld2 = %.4f\n\n', ld1, ld2);

fprintf('Copiá y pegá esta línea en la GUI o en el código C:\n');
fprintf('OBS:%.4f,%.4f,%.4f,%.4f\n', lb1, lb2, ld1, ld2);

% Verificación opcional: Polos en lazo cerrado del observador
eig_obs_barra = eig(A_barra - L_barra*C_barra);
fprintf('\nVerificación de polos (deben coincidir con p_bessel):\n');
disp(eig_obs_barra);