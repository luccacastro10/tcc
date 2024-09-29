function T = dh_matrix(q1, d, alpha, a)
    % Calcula a matriz de transformação de Denavit-Hartenberg (DH) para um valor dado de q1.
    % Parâmetros:
    %   q1    - Ângulo da junta (em radianos)
    %   d     - Deslocamento ao longo do eixo z
    %   a     - Comprimento do elo ao longo do eixo x
    %   alpha - Ângulo de torção (entre os eixos z de dois frames sucessivos)
    
    % Cálculo da matriz de transformação homogênea usando os parâmetros DH
    T = [cos(q1), -sin(q1)*cos(alpha),  sin(q1)*sin(alpha), a*cos(q1);
         sin(q1),  cos(q1)*cos(alpha), -cos(q1)*sin(alpha), a*sin(q1);
         0,       sin(alpha),          cos(alpha),         d;
         0,       0,                   0,                  1];
end
