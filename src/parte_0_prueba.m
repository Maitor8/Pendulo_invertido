% =========================================================================
% Preprocesamiento e Identificación de parámetros del péndulo con volante
% =========================================================================
clear; close all;

% -------- 1. Carga de datos y preprocesamiento --------
archivo = 'registro_laboratorio4.0.csv';
data = readtable(archivo);

tiempo = data.t_s;                
tita1  = data.theta_rad;          
omega1 = data.omega_rad_s/2;        
vel2   = data.v_disco_rad_s/2;      
u      = data.u_ctrl;                

% Aceleraciones por diferencias centradas
acel1 = gradient(omega1, tiempo);
acel2 = gradient(vel2, tiempo);

% Filtrado con media móvil
vel1  = movmean(omega1, 10);
vel2  = movmean(vel2, 10);
acel1 = movmean(acel1, 15);
acel2 = movmean(acel2, 15);

% Asegurar que todos los vectores sean columnas
tita1 = tita1(:); vel2 = vel2(:); 
acel1 = acel1(:); acel2 = acel2(:); 
u = u(:); tiempo = tiempo(:);

% -------- 2. Identificación del vector de parámetros q = [q1; q2; q3] --------
Y_q = acel1;
Phi_q = [-sin(tita1), vel2, -u];

% Cálculo de la pseudoinversa usando SVD
[U_q, S_q, V_q] = svd(Phi_q, 'econ');
S_q_inv = diag(1 ./ diag(S_q));
q_est = V_q * S_q_inv * U_q' * Y_q;

q1 = q_est(1);
q2 = q_est(2);
q3 = q_est(3);

% -------- 3. Identificación del parámetro rho --------
Y_rho = acel1 + acel2;
Phi_rho = (q3 * u) - (q2 * vel2);

% Cálculo de la pseudoinversa usando SVD para rho
[U_rho, S_rho, V_rho] = svd(Phi_rho, 'econ');
S_rho_inv = diag(1 ./ diag(S_rho));
rho = V_rho * S_rho_inv * U_rho' * Y_rho;

% -------- 4. Validación y cálculo de aceleraciones del modelo --------
alfa1_modelo = -q1*sin(tita1) + q2*vel2 - q3*u;
alfa2_modelo = rho*(q3*u - q2*vel2) - alfa1_modelo;


error_q = Y_q - Phi_q * q_est;
MSE_q = mean(error_q.^2);

error_rho = Y_rho - Phi_rho * rho;
MSE_rho = mean(error_rho.^2);
fprintf('Parámetros estimados:\n');
fprintf('q1 = %.4f\n', q1);
fprintf('q2 = %.4f\n', q2);
fprintf('q3 = %.4f\n\n', q3);
fprintf('rho = %.4f\n', rho);
fprintf('\nError Cuadrático Medio (MSE):\n');
fprintf('MSE Ecuación q: %.4e\n', MSE_q);
fprintf('MSE Ecuación rho: %.4e\n', MSE_rho);

% -------- 5. Gráficos --------
figure('Name', 'Validación alfa1', 'Position', [100 100 900 400]);
plot(tiempo, acel1, 'b-', 'LineWidth', 1.2); hold on;
plot(tiempo, alfa1_modelo, 'r--', 'LineWidth', 1.2);
xlabel('Tiempo [s]'); ylabel('\alpha_1 [rad/s^2]');
title('Validación del modelo - \alpha_1');
legend('Estimada', 'Modelo identificado');
grid on;

figure('Name', 'Validación alfa2', 'Position', [100 100 900 400]);
plot(tiempo, acel2, 'b-', 'LineWidth', 1.2); hold on;
plot(tiempo, alfa2_modelo, 'r--', 'LineWidth', 1.2);
xlabel('Tiempo [s]'); ylabel('\alpha_2 [rad/s^2]');
title('Comparación aceleración del volante - \alpha_2');
legend('Estimada', 'Modelo identificado (usando \rho)');
grid on;