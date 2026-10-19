%% ========================================================================
%  Discretización de sistema 3x3 con polos de Bessel y cálculo de K
%  usando PLACE
%  ========================================================================
clc; close all;

%% 1) Definir el sistema continuo 3x3 (modifícalo según tu caso)
%    Ejemplo: sistema en forma canónica controlable
A = [0  1  0;
     q1  0  q2;
    -q1 0 -(rho+1)*q2];
B = [0; -q3; (rho+1)*q3];
C = eye(3);
D = zeros(3,1);

sys_c = ss(A,B,C,D);

%% 3) Escalar los polos según el tiempo de establecimiento ts deseado
ts = 1.0;   % [s]  tiempo de establecimiento (criterio 2%)

p_bessel = [-3.9668-3.7844j,-3.9668+3.7844j, -5.00947]/ts;

fprintf('Factor de escala (wn equivalente): %.4f rad/s\n', factor);
fprintf('Polos deseados en continuo (plano s):\n');
disp(p_bessel);

%% 4) Discretizar el sistema con ZOH
Ts   = 0.005;                                % [s] tiempo de muestreo
sys_d = c2d(sys_c, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(sys_d);

fprintf('Matriz Ad (discreta):\n'); disp(Ad);
fprintf('Matriz Bd (discreta):\n'); disp(Bd);

%% 5) Mapear los polos deseados al plano Z:  z = exp(s*Ts)
p_des_z = exp(p_bessel * Ts);

fprintf('Polos deseados en discreto (plano z):\n');
disp(p_des_z);
fprintf('Magnitud de los polos (deben ser < 1):\n');
disp(abs(p_des_z));

%% 6) Verificar controlabilidad (requisito de PLACE)
Co = ctrb(Ad, Bd);
if rank(Co) < size(Ad,1)
    error('Sistema NO controlable: no se puede aplicar PLACE.');
end
fprintf('Rango de la matriz de controlabilidad: %d (OK)\n', rank(Co));

%% 7) Calcular la ganancia K con PLACE
K = place(Ad, Bd, p_des_z);

fprintf('\nGanancia K obtenida con PLACE:\n');
disp(K);

%% 8) Verificación: polos en lazo cerrado
Acl     = Ad - Bd*K;
polos_cl = eig(Acl);

fprintf('Polos lazo cerrado obtenidos:\n');  disp(sort(polos_cl));
fprintf('Polos deseados (referencia):\n');    disp(sort(p_des_z));

%% 9) Respuesta al escalón del sistema en lazo cerrado
sys_cl = ss(Acl, Bd, Cd, Dd, Ts);
figure('Name','Lazo cerrado');
step(sys_cl);
title(sprintf('Lazo cerrado — t_s ≈ %.2f s, T_s = %.3f s', ts, Ts));
grid on;