function [S14, S24, S34, J] = literal_kinematics(leg, q)
%LITERAL_KINEMATICS Calcula posição e Jacobiano completo do efetuador da perna
%
%   [S14, S24, S34, J] = literal_kinematics(leg, q)
%
%   Calcula a posição do efetuador da perna especificada pelo objeto SerialLink
%   e vetor de juntas q, usando as expressões literais da cinemática direta.
%   Também calcula a Jacobiano completo (linear e angular) do efetuador.
%
%   Entradas:
%       leg - objeto SerialLink representando a perna do robô
%       q   - vetor coluna (3x1) com os ângulos articulares atuais da perna
%
%   Saídas:
%       S14, S24, S34 - coordenadas x, y, z da posição do efetuador no frame base
%       J             - matriz Jacobiana 6x3, onde as 3 primeiras linhas são a
%                       parte linear e as 3 últimas linhas a parte angular
%
%   Observações:
%       - A função utiliza cálculo simbólico para derivar o Jacobiano.
%       - A matriz base do robô é considerada na transformação final.
%       - O Jacobiano angular é calculada a partir dos eixos de rotação das juntas,
%         considerando a rotação da base.
%       - Método foi feito a partir do chatbot literal_kinematics.md. Em
%       caso de dúvidas, consultar esse documento.
%
%   Exemplo de uso:
%       [x, y, z, J] = literal_kinematics(leg1, q_leg1);
%
%   Autor: Lucca
%   Data: 2025-04-21
  
  DH = get_DH_from_SerialLink(leg, q);
  
  % Extrai parâmetros DH
  theta = DH(:,1);
  d = DH(:,2);
  a = DH(:,3);
  alpha = DH(:,4);
  base = leg.base.T;
  
  % Cossenos e senos para cada parâmetro
  c1 = cos(theta(1)); s1 = sin(theta(1));
  c2 = cos(theta(2)); s2 = sin(theta(2));
  c3 = cos(theta(3)); s3 = sin(theta(3));
  
  ca1 = cos(alpha(1)); sa1 = sin(alpha(1));
  ca2 = cos(alpha(2)); sa2 = sin(alpha(2));
  ca3 = cos(alpha(3)); sa3 = sin(alpha(3));
  
  % Elementos da matriz base
  r11 = base(1,1); r12 = base(1,2); r13 = base(1,3); px = base(1,4);
  r21 = base(2,1); r22 = base(2,2); r23 = base(2,3); py = base(2,4);
  r31 = base(3,1); r32 = base(3,2); r33 = base(3,3); pz = base(3,4);
  
  % Cálculo dos termos intermediários M_ij da multiplicação A1*A2
  M11 = c1*c2 - s1*ca1*s2;
  M12 = -c1*s2*ca2 - s1*ca1*c2*ca2 + s1*sa1*sa2;
  M13 = c1*s2*sa2 + s1*ca1*c2*sa2 + s1*sa1*ca2;
  M14 = a(1)*c1 + a(2)*M11 + d(2)*s1*sa1;
  
  M21 = s1*c2 + c1*ca1*s2;
  M22 = -s1*s2*ca2 + c1*ca1*c2*ca2 - c1*sa1*sa2;
  M23 = s1*s2*sa2 - c1*ca1*c2*sa2 - c1*sa1*ca2;
  M24 = a(1)*s1 + a(2)*M21 - d(2)*c1*sa1;
  
  M31 = sa1*s2;
  M32 = sa1*c2*ca2 + ca1*sa2;
  M33 = -sa1*c2*sa2 + ca1*ca2;
  M34 = d(1) + a(2)*M31 + d(2)*ca1;
  
  % Cálculo da posição do efetuador no frame 0 DH
  T14 = M11*a(3)*c3 + M12*a(3)*s3 + M14;
  T24 = M21*a(3)*c3 + M22*a(3)*s3 + M24;
  T34 = M31*a(3)*c3 + M32*a(3)*s3 + M34;
  
  % Posição final no frame da base do robô
  S14 = r11*T14 + r12*T24 + r13*T34 + px;
  S24 = r21*T14 + r22*T24 + r23*T34 + py;
  S34 = r31*T14 + r32*T24 + r33*T34 + pz;
  
  fprintf('Posição literal do efetuador:\n\n %.4f\n %.4f\n %.4f\n', S14, S24, S34);
  
  % --- Cálculo do Jacobiano linear via cálculo simbólico ---
  syms th1 th2 th3 real
  
  c1s = cos(th1); s1s = sin(th1);
  c2s = cos(th2); s2s = sin(th2);
  c3s = cos(th3); s3s = sin(th3);
  
  ca1s = cos(alpha(1)); sa1s = sin(alpha(1));
  ca2s = cos(alpha(2)); sa2s = sin(alpha(2));
  
  M11s = c1s*c2s - s1s*ca1s*s2s;
  M12s = -c1s*s2s*ca2s - s1s*ca1s*c2s*ca2s + s1s*sa1s*sa2s;
  M14s = a(1)*c1s + a(2)*M11s + d(2)*s1s*sa1s;
  
  M21s = s1s*c2s + c1s*ca1s*s2s;
  M22s = -s1s*s2s*ca2s + c1s*ca1s*c2s*ca2s - c1s*sa1s*sa2s;
  M24s = a(1)*s1s + a(2)*M21s - d(2)*c1s*sa1s;
  
  M31s = sa1s*s2s;
  M32s = sa1s*c2s*ca2s + ca1s*sa2s;
  M34s = d(1) + a(2)*M31s + d(2)*ca1s;
  
  T14s = M11s*a(3)*c3s + M12s*a(3)*s3s + M14s;
  T24s = M21s*a(3)*c3s + M22s*a(3)*s3s + M24s;
  T34s = M31s*a(3)*c3s + M32s*a(3)*s3s + M34s;
  
  S14s = r11*T14s + r12*T24s + r13*T34s + px;
  S24s = r21*T14s + r22*T24s + r23*T34s + py;
  S34s = r31*T14s + r32*T24s + r33*T34s + pz;
  
  pos_sym = [S14s; S24s; S34s];
  vars = [th1; th2; th3];
  
  Jv_sym = jacobian(pos_sym, vars);
  Jv = double(subs(Jv_sym, vars, theta));
  
  % --- Cálculo do Jacobiano angular ---
  % Função para matriz DH simbólica
  DH_matrix = @(th, d_, a_, al) [ ...
    cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a_*cos(th); ...
    sin(th),  cos(th)*cos(al), -cos(th)*sin(al), a_*sin(th); ...
    0,        sin(al),          cos(al),         d_; ...
    0,        0,                0,               1];
  
  A1 = DH_matrix(th1, d(1), a(1), alpha(1));
  A2 = DH_matrix(th2, d(2), a(2), alpha(2));
  A3 = DH_matrix(th3, d(3), a(3), alpha(3));
  
  R1 = A1(1:3,1:3);
  A12 = A1 * A2;
  R2 = A12(1:3,1:3);
  
  R_base = base(1:3,1:3);

  z0 = R_base * [0;0;1];
  z1 = R_base * R1(:,3);
  z2 = R_base * R2(:,3);
  
  Jw_sym = [z0, z1, z2];
  Jw = double(subs(Jw_sym, vars, theta));
  
  fprintf('\nJacobiano literal do efetuador:\n');

  J = [Jv ; Jw]
end

function DH = get_DH_from_SerialLink(leg, q)
  n = length(leg.links);
  DH = zeros(n,4);
  for i = 1:n
    theta_real = q(i) + leg.links(i).offset;
    d = leg.links(i).d;
    a = leg.links(i).a;
    alpha = leg.links(i).alpha;
    DH(i,:) = [theta_real, d, a, alpha];
  end
end
