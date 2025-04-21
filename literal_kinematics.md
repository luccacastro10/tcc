# Matriz de Denavit-Hartenberg explanation

Created: 4/21/2025, 9:10:42 AM
Last Updated: 4/21/2025, 9:23:05 AM
---

### User

Me explique o que é a matriz de Denavit-Hartenberg


---

### Assistant (gpt-4.1-mini)

Claro! A **matriz de Denavit-Hartenberg (DH)** é uma ferramenta matemática muito usada em robótica para descrever a posição e orientação relativa entre os elos (ou segmentos) de um manipulador robótico.

### Contexto
Um manipulador robótico é composto por vários elos conectados por juntas. Para controlar o robô, precisamos saber a posição e orientação de cada elo em relação a um sistema de referência fixo (geralmente a base do robô). A matriz DH facilita essa tarefa ao padronizar a forma de representar essas transformações.

### O que é a matriz DH?

A matriz DH é uma matriz de transformação homogênea \(4 \times 4\) que representa a transformação de um sistema de coordenadas ligado a um elo \(i-1\) para o sistema de coordenadas do elo \(i\). Ela é construída a partir de quatro parâmetros chamados **parâmetros DH**:

- \( \theta_i \): ângulo de rotação em torno do eixo \(z_{i-1}\) (ângulo da junta)
- \( d_i \): deslocamento ao longo do eixo \(z_{i-1}\)
- \( a_i \): comprimento do elo, ou deslocamento ao longo do eixo \(x_i\)
- \( \alpha_i \): ângulo entre os eixos \(z_{i-1}\) e \(z_i\), medido em torno do eixo \(x_i\)

### Matriz de transformação DH

A matriz DH é dada por:

\[
A_i =
\begin{bmatrix}
\cos \theta_i & -\sin \theta_i \cos \alpha_i & \sin \theta_i \sin \alpha_i & a_i \cos \theta_i \\
\sin \theta_i & \cos \theta_i \cos \alpha_i & -\cos \theta_i \sin \alpha_i & a_i \sin \theta_i \\
0 & \sin \alpha_i & \cos \alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

### Para que serve?

- **Cinemática direta:** Multiplicando as matrizes DH de todos os elos, você obtém a posição e orientação da ferramenta final (end-effector) em relação à base do robô.
- **Padronização:** Facilita a modelagem de robôs complexos, pois todos os elos são descritos de forma consistente.
- **Simplicidade:** Reduz o problema de descrever transformações espaciais complexas a uma sequência de transformações simples.

### Resumo

A matriz de Denavit-Hartenberg é uma forma padronizada de representar a posição e orientação relativa entre elos de um robô, usando quatro parâmetros geométricos, e é fundamental para a análise e controle de manipuladores robóticos.

Se quiser, posso ajudar a montar uma matriz DH para um exemplo específico! Quer?


---

### User

Tenho um manipulador robôtico de 3 juntas, de tal forma que minha tabela de Denavit-Hartenberg possui 3 linhas, indicando uma multiplicação em cascata de 3 matrizes de Denavit-Hartenberg referente a cada linha. Preciso que voce me retorne a matriz resultante em função dos elementos da tabela montada


---

### Assistant (gpt-4.1-mini)

Claro! Para um manipulador robótico com 3 juntas, você terá 3 matrizes de Denavit-Hartenberg \( A_1, A_2, A_3 \), cada uma construída a partir dos parâmetros DH da respectiva junta.

### Passo 1: Parâmetros DH

Suponha que sua tabela DH seja assim (cada linha corresponde a uma junta \(i\)):

| \(i\) | \(\theta_i\) | \(d_i\) | \(a_i\) | \(\alpha_i\) |
|-------|--------------|---------|---------|--------------|
| 1     | \(\theta_1\) | \(d_1\) | \(a_1\) | \(\alpha_1\) |
| 2     | \(\theta_2\) | \(d_2\) | \(a_2\) | \(\alpha_2\) |
| 3     | \(\theta_3\) | \(d_3\) | \(a_3\) | \(\alpha_3\) |

### Passo 2: Matriz DH para cada junta

Cada matriz \(A_i\) é dada por:

\[
A_i =
\begin{bmatrix}
\cos \theta_i & -\sin \theta_i \cos \alpha_i & \sin \theta_i \sin \alpha_i & a_i \cos \theta_i \\
\sin \theta_i & \cos \theta_i \cos \alpha_i & -\cos \theta_i \sin \alpha_i & a_i \sin \theta_i \\
0 & \sin \alpha_i & \cos \alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

### Passo 3: Matriz resultante

A matriz total \(T\) que transforma do sistema base até o sistema final (end-effector) é a multiplicação em cascata:

\[
T = A_1 \cdot A_2 \cdot A_3
\]

---

### Expressão geral da matriz resultante

Como a multiplicação de matrizes é longa para escrever explicitamente, vou te mostrar a forma simbólica para que você possa implementar ou calcular em software simbólico (Matlab, Python, etc).

Se quiser, posso ajudar a expandir a multiplicação para um caso específico, mas em geral:

\[
T = \left(
\begin{bmatrix}
R_1 & p_1 \\
0 & 1
\end{bmatrix}
\right)
\cdot
\left(
\begin{bmatrix}
R_2 & p_2 \\
0 & 1
\end{bmatrix}
\right)
\cdot
\left(
\begin{bmatrix}
R_3 & p_3 \\
0 & 1
\end{bmatrix}
\right)
\]

onde cada \(R_i\) é a submatriz \(3 \times 3\) de rotação e \(p_i\) é o vetor \(3 \times 1\) de translação da matriz \(A_i\).

---

### Se quiser, posso montar o produto completo para você, só me confirme se deseja a multiplicação simbólica completa!


---

### User

quero a multiplicação completa


---

### Assistant (gpt-4.1-mini)

Perfeito! Vou montar a multiplicação completa das três matrizes de Denavit-Hartenberg \(A_1\), \(A_2\) e \(A_3\), em função dos parâmetros \(\theta_i, d_i, a_i, \alpha_i\).

---

### Matriz DH genérica para junta \(i\):

\[
A_i =
\begin{bmatrix}
c_i & -s_i c_{\alpha_i} & s_i s_{\alpha_i} & a_i c_i \\
s_i & c_i c_{\alpha_i} & -c_i s_{\alpha_i} & a_i s_i \\
0 & s_{\alpha_i} & c_{\alpha_i} & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

onde \(c_i = \cos \theta_i\), \(s_i = \sin \theta_i\), \(c_{\alpha_i} = \cos \alpha_i\), \(s_{\alpha_i} = \sin \alpha_i\).

---

### Passo 1: Multiplicar \(A_1 \cdot A_2 = M\)

Definindo:

\[
A_1 =
\begin{bmatrix}
c_1 & -s_1 c_{\alpha_1} & s_1 s_{\alpha_1} & a_1 c_1 \\
s_1 & c_1 c_{\alpha_1} & -c_1 s_{\alpha_1} & a_1 s_1 \\
0 & s_{\alpha_1} & c_{\alpha_1} & d_1 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

\[
A_2 =
\begin{bmatrix}
c_2 & -s_2 c_{\alpha_2} & s_2 s_{\alpha_2} & a_2 c_2 \\
s_2 & c_2 c_{\alpha_2} & -c_2 s_{\alpha_2} & a_2 s_2 \\
0 & s_{\alpha_2} & c_{\alpha_2} & d_2 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

Multiplicando \(M = A_1 A_2\):

\[
M =
\begin{bmatrix}
M_{11} & M_{12} & M_{13} & M_{14} \\
M_{21} & M_{22} & M_{23} & M_{24} \\
M_{31} & M_{32} & M_{33} & M_{34} \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

onde:

\[
\begin{aligned}
M_{11} &= c_1 c_2 - s_1 c_{\alpha_1} s_2 \\
M_{12} &= -c_1 s_2 c_{\alpha_2} - s_1 c_{\alpha_1} c_2 c_{\alpha_2} + s_1 s_{\alpha_1} s_{\alpha_2} \\
M_{13} &= c_1 s_2 s_{\alpha_2} + s_1 c_{\alpha_1} c_2 s_{\alpha_2} + s_1 s_{\alpha_1} c_{\alpha_2} \\
M_{14} &= a_1 c_1 + a_2 (c_1 c_2 - s_1 c_{\alpha_1} s_2) + d_2 s_1 s_{\alpha_1} \\
\\
M_{21} &= s_1 c_2 + c_1 c_{\alpha_1} s_2 \\
M_{22} &= -s_1 s_2 c_{\alpha_2} + c_1 c_{\alpha_1} c_2 c_{\alpha_2} - c_1 s_{\alpha_1} s_{\alpha_2} \\
M_{23} &= s_1 s_2 s_{\alpha_2} - c_1 c_{\alpha_1} c_2 s_{\alpha_2} - c_1 s_{\alpha_1} c_{\alpha_2} \\
M_{24} &= a_1 s_1 + a_2 (s_1 c_2 + c_1 c_{\alpha_1} s_2) - d_2 c_1 s_{\alpha_1} \\
\\
M_{31} &= s_{\alpha_1} s_2 \\
M_{32} &= s_{\alpha_1} c_2 c_{\alpha_2} + c_{\alpha_1} s_{\alpha_2} \\
M_{33} &= -s_{\alpha_1} c_2 s_{\alpha_2} + c_{\alpha_1} c_{\alpha_2} \\
M_{34} &= d_1 + a_2 s_{\alpha_1} s_2 + d_2 c_{\alpha_1} \\
\end{aligned}
\]

---

### Passo 2: Multiplicar \(T = M \cdot A_3\)

Agora, \(A_3\) é:

\[
A_3 =
\begin{bmatrix}
c_3 & -s_3 c_{\alpha_3} & s_3 s_{\alpha_3} & a_3 c_3 \\
s_3 & c_3 c_{\alpha_3} & -c_3 s_{\alpha_3} & a_3 s_3 \\
0 & s_{\alpha_3} & c_{\alpha_3} & d_3 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

Multiplicando \(T = M A_3\):

\[
T =
\begin{bmatrix}
T_{11} & T_{12} & T_{13} & T_{14} \\
T_{21} & T_{22} & T_{23} & T_{24} \\
T_{31} & T_{32} & T_{33} & T_{34} \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

onde:

\[
\begin{aligned}
T_{11} &= M_{11} c_3 + M_{12} s_3 \\
T_{12} &= -M_{11} s_3 c_{\alpha_3} + M_{12} c_3 c_{\alpha_3} + M_{13} s_{\alpha_3} \\
T_{13} &= M_{11} s_3 s_{\alpha_3} - M_{12} c_3 s_{\alpha_3} + M_{13} c_{\alpha_3} \\
T_{14} &= M_{11} a_3 c_3 + M_{12} a_3 s_3 + M_{14} \\
\\
T_{21} &= M_{21} c_3 + M_{22} s_3 \\
T_{22} &= -M_{21} s_3 c_{\alpha_3} + M_{22} c_3 c_{\alpha_3} + M_{23} s_{\alpha_3} \\
T_{23} &= M_{21} s_3 s_{\alpha_3} - M_{22} c_3 s_{\alpha_3} + M_{23} c_{\alpha_3} \\
T_{24} &= M_{21} a_3 c_3 + M_{22} a_3 s_3 + M_{24} \\
\\
T_{31} &= M_{31} c_3 + M_{32} s_3 \\
T_{32} &= -M_{31} s_3 c_{\alpha_3} + M_{32} c_3 c_{\alpha_3} + M_{33} s_{\alpha_3} \\
T_{33} &= M_{31} s_3 s_{\alpha_3} - M_{32} c_3 s_{\alpha_3} + M_{33} c_{\alpha_3} \\
T_{34} &= M_{31} a_3 c_3 + M_{32} a_3 s_3 + M_{34} \\
\end{aligned}
\]

---

### Resumo final

A matriz resultante \(T = A_1 A_2 A_3\) é:

\[
\boxed{
T =
\begin{bmatrix}
T_{11} & T_{12} & T_{13} & T_{14} \\
T_{21} & T_{22} & T_{23} & T_{24} \\
T_{31} & T_{32} & T_{33} & T_{34} \\
0 & 0 & 0 & 1
\end{bmatrix}
}
\]

com os elementos \(T_{ij}\) definidos acima em função dos parâmetros DH.

---

Se quiser, posso ajudar a simplificar para casos específicos (por exemplo, \(\alpha_i = 0\)) ou gerar código para calcular essa matriz automaticamente. Quer?


---

### User

Perfeito, porém agora preciso te passar uma informação extra. A base do meu manipulador robótico não coincide com o frame 0 da matriz de Denavit-Hartenberg, de forma que é preciso fazer uma pré multiplicação referente a essa transformação homogênea da base do robô para o frame 0 da tabela de Denavit-Hartenberg. Assim, preciso, na verdade, da formulação matemática da multiplicação de 4 matrizes em cascata. Me retorne o equacionamento completo


---

### Assistant (gpt-4.1-mini)

Entendi! Você tem uma transformação homogênea da base do robô para o frame 0 da tabela DH, digamos \(B\), e quer a transformação total do sistema base do robô até o end-effector, que será:

\[
T_{\text{total}} = B \cdot A_1 \cdot A_2 \cdot A_3
\]

---

### Passo 1: Definir a matriz da base \(B\)

Seja a matriz homogênea da base:

\[
B =
\begin{bmatrix}
r_{11} & r_{12} & r_{13} & p_x \\
r_{21} & r_{22} & r_{23} & p_y \\
r_{31} & r_{32} & r_{33} & p_z \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

onde \(r_{ij}\) formam a matriz de rotação da base e \((p_x, p_y, p_z)\) é o vetor de translação da base.

---

### Passo 2: Matriz total

A matriz total é:

\[
T_{\text{total}} = B \cdot A_1 \cdot A_2 \cdot A_3
\]

---

### Passo 3: Multiplicação em cascata

Já temos \(M = A_1 A_2\) e \(T = M A_3\) do passo anterior. Agora, vamos multiplicar \(B \cdot T\):

\[
T_{\text{total}} = B \cdot T =
\begin{bmatrix}
r_{11} & r_{12} & r_{13} & p_x \\
r_{21} & r_{22} & r_{23} & p_y \\
r_{31} & r_{32} & r_{33} & p_z \\
0 & 0 & 0 & 1
\end{bmatrix}
\cdot
\begin{bmatrix}
T_{11} & T_{12} & T_{13} & T_{14} \\
T_{21} & T_{22} & T_{23} & T_{24} \\
T_{31} & T_{32} & T_{33} & T_{34} \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

---

### Passo 4: Resultado da multiplicação \(T_{\text{total}} = B \cdot T\)

\[
T_{\text{total}} =
\begin{bmatrix}
S_{11} & S_{12} & S_{13} & S_{14} \\
S_{21} & S_{22} & S_{23} & S_{24} \\
S_{31} & S_{32} & S_{33} & S_{34} \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

onde:

\[
\begin{aligned}
S_{11} &= r_{11} T_{11} + r_{12} T_{21} + r_{13} T_{31} \\
S_{12} &= r_{11} T_{12} + r_{12} T_{22} + r_{13} T_{32} \\
S_{13} &= r_{11} T_{13} + r_{12} T_{23} + r_{13} T_{33} \\
S_{14} &= r_{11} T_{14} + r_{12} T_{24} + r_{13} T_{34} + p_x \\
\\
S_{21} &= r_{21} T_{11} + r_{22} T_{21} + r_{23} T_{31} \\
S_{22} &= r_{21} T_{12} + r_{22} T_{22} + r_{23} T_{32} \\
S_{23} &= r_{21} T_{13} + r_{22} T_{23} + r_{23} T_{33} \\
S_{24} &= r_{21} T_{14} + r_{22} T_{24} + r_{23} T_{34} + p_y \\
\\
S_{31} &= r_{31} T_{11} + r_{32} T_{21} + r_{33} T_{31} \\
S_{32} &= r_{31} T_{12} + r_{32} T_{22} + r_{33} T_{32} \\
S_{33} &= r_{31} T_{13} + r_{32} T_{23} + r_{33} T_{33} \\
S_{34} &= r_{31} T_{14} + r_{32} T_{24} + r_{33} T_{34} + p_z \\
\end{aligned}
\]

---

### **Resumo final:**

\[
\boxed{
\begin{aligned}
T_{\text{total}} &= B \cdot A_1 \cdot A_2 \cdot A_3 = \\
&=
\begin{bmatrix}
S_{11} & S_{12} & S_{13} & S_{14} \\
S_{21} & S_{22} & S_{23} & S_{24} \\
S_{31} & S_{32} & S_{33} & S_{34} \\
0 & 0 & 0 & 1
\end{bmatrix}
\end{aligned}
}
\]

com \(S_{ij}\) definidos acima em função da matriz base \(B\) e da matriz \(T = A_1 A_2 A_3\).

---

Se quiser, posso ajudar a montar um código para calcular essa matriz total automaticamente, ou ajudar a interpretar os resultados. Quer?


---

### User

ótimo, agora preciso de um código matlab que calcule os termos da ultima coluna da matriz resultante referentes às posições px, py, pz da base do manipulador até o efetuador, ou seja, os elementos S_{14}, S_{24}, S_{34}.


---

### Assistant (gpt-4.1-mini)

Claro! Vou te fornecer um código MATLAB que calcula os elementos \(S_{14}\), \(S_{24}\) e \(S_{34}\) da última coluna da matriz total \(T_{\text{total}} = B \cdot A_1 \cdot A_2 \cdot A_3\), ou seja, a posição do efetuador no sistema da base do robô.

---

### Código MATLAB

```matlab
function [S14, S24, S34] = posicao_efetuador(B, DH)
  % B: matriz 4x4 da base do robô
  % DH: matriz 3x4 com os parâmetros DH [theta, d, a, alpha] para as 3 juntas
  % Retorna S14, S24, S34: posição do efetuador no frame da base
  
  % Extrai parâmetros DH
  theta = DH(:,1);
  d = DH(:,2);
  a = DH(:,3);
  alpha = DH(:,4);
  
  % Função auxiliar para matriz DH de uma junta
  A = @(th, di, ai, al) [ ...
    cos(th), -sin(th)*cos(al),  sin(th)*sin(al), ai*cos(th); ...
    sin(th),  cos(th)*cos(al), -cos(th)*sin(al), ai*sin(th); ...
    0,        sin(al),          cos(al),         di; ...
    0,        0,                0,               1];
  
  % Calcula as matrizes A1, A2, A3
  A1 = A(theta(1), d(1), a(1), alpha(1));
  A2 = A(theta(2), d(2), a(2), alpha(2));
  A3 = A(theta(3), d(3), a(3), alpha(3));
  
  % Multiplica as matrizes DH
  T = A1 * A2 * A3;
  
  % Multiplica pela matriz da base
  T_total = B * T;
  
  % Extrai os elementos da última coluna (posição)
  S14 = T_total(1,4);
  S24 = T_total(2,4);
  S34 = T_total(3,4);
end
```

---

### Como usar

```matlab
% Exemplo de matriz base B (identidade, sem deslocamento)
B = eye(4);

% Exemplo de tabela DH: [theta, d, a, alpha] para 3 juntas
DH = [pi/4, 0.5, 1.0, pi/2;
      pi/3, 0,   1.0, 0;
      pi/6, 0,   0.5, 0];

[S14, S24, S34] = posicao_efetuador(B, DH);

fprintf('Posição do efetuador:\n x = %.4f\n y = %.4f\n z = %.4f\n', S14, S24, S34);
```

---

Se quiser, posso ajudar a adaptar para casos com variáveis simbólicas ou gerar o código para outras linguagens. Quer?


---

### User

Não quero que o código faça esse multiplicação literal. O que quero, na verdade, é validar o equacionamento que você formulou acima, de tal forma que eu consiga formular exatamente a equação que descreve os valores de S_{14}, S_{24} e S_{34}. Dessa forma, a função deve implementar essas equações literais, para que eu possa posteriormente validar com meu modelo (deixa que a validação eu faço, foque apenas no cálculo literal desses componentes). Assim, a função que voce vai fazer desse retornar os valores de 3 equações, compostas por senos, cossenos, etc


---

### Assistant (gpt-4.1-mini)

Entendi! Você quer uma função MATLAB que calcule **diretamente** os valores de \(S_{14}\), \(S_{24}\) e \(S_{34}\) usando as expressões literais que definimos, sem fazer multiplicação matricial, para validar o equacionamento.

Vou implementar as fórmulas explícitas para \(S_{14}\), \(S_{24}\), \(S_{34}\) conforme:

\[
\begin{aligned}
S_{14} &= r_{11} T_{14} + r_{12} T_{24} + r_{13} T_{34} + p_x \\
S_{24} &= r_{21} T_{14} + r_{22} T_{24} + r_{23} T_{34} + p_y \\
S_{34} &= r_{31} T_{14} + r_{32} T_{24} + r_{33} T_{34} + p_z
\end{aligned}
\]

onde \(T_{14}, T_{24}, T_{34}\) são funções explícitas dos parâmetros DH e trigonometria, conforme o produto \(A_1 A_2 A_3\) que detalhamos.

---

### Código MATLAB com cálculo literal

```matlab
function [S14, S24, S34] = posicao_literal(B, DH)
  % B: matriz 4x4 da base do robô
  % DH: matriz 3x4 com os parâmetros DH [theta, d, a, alpha] para as 3 juntas
  % Retorna S14, S24, S34 calculados pelas expressões literais
  
  % Extrai parâmetros DH
  theta = DH(:,1);
  d = DH(:,2);
  a = DH(:,3);
  alpha = DH(:,4);
  
  % Cossenos e senos para cada parâmetro
  c1 = cos(theta(1)); s1 = sin(theta(1));
  c2 = cos(theta(2)); s2 = sin(theta(2));
  c3 = cos(theta(3)); s3 = sin(theta(3));
  
  ca1 = cos(alpha(1)); sa1 = sin(alpha(1));
  ca2 = cos(alpha(2)); sa2 = sin(alpha(2));
  ca3 = cos(alpha(3)); sa3 = sin(alpha(3));
  
  % Elementos da matriz base B
  r11 = B(1,1); r12 = B(1,2); r13 = B(1,3); px = B(1,4);
  r21 = B(2,1); r22 = B(2,2); r23 = B(2,3); py = B(2,4);
  r31 = B(3,1); r32 = B(3,2); r33 = B(3,3); pz = B(3,4);
  
  % Primeiro, calcular T14, T24, T34 (posição do efetuador no frame 0 DH)
  
  % Para facilitar, calculamos os termos intermediários M_ij da multiplicação A1*A2
  % conforme definido anteriormente:
  
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
  
  % Agora, calcular T14, T24, T34 = M * A3 (posição do efetuador no frame 0 DH)
  
  T14 = M11*a(3)*c3 + M12*a(3)*s3 + M14;
  T24 = M21*a(3)*c3 + M22*a(3)*s3 + M24;
  T34 = M31*a(3)*c3 + M32*a(3)*s3 + M34;
  
  % Finalmente, calcular S14, S24, S34 = B * [T14; T24; T34; 1]
  
  S14 = r11*T14 + r12*T24 + r13*T34 + px;
  S24 = r21*T14 + r22*T24 + r23*T34 + py;
  S34 = r31*T14 + r32*T24 + r33*T34 + pz;
end
```

---

### Como usar

```matlab
% Exemplo matriz base B (pode ser identidade ou qualquer transformação)
B = eye(4);

% Exemplo tabela DH: [theta, d, a, alpha]
DH = [pi/4, 0.5, 1.0, pi/2;
      pi/3, 0,   1.0, 0;
      pi/6, 0,   0.5, 0];

[S14, S24, S34] = posicao_literal(B, DH);

fprintf('Posição literal do efetuador:\n x = %.4f\n y = %.4f\n z = %.4f\n', S14, S24, S34);
```

---

Se quiser, posso ajudar a gerar a versão simbólica para facilitar a validação analítica. Quer?


---

### User

Você conseguiria descrever a equação de S14, S24 e S34 na forma completa, sem fazer uso dos termos intermediários? Quero incluir as equações no meu TCC


---

### Assistant (gpt-4.1-mini)

Claro, Lucca! Vou escrever as expressões completas para \( S_{14} \), \( S_{24} \) e \( S_{34} \) sem usar os termos intermediários \( M_{ij} \), diretamente em função dos parâmetros DH e da matriz base \( B \).

---

### Dados

- Parâmetros DH para as 3 juntas: \(\theta_i, d_i, a_i, \alpha_i\), com \(i=1,2,3\).
- Definimos:
  \[
  c_i = \cos \theta_i, \quad s_i = \sin \theta_i, \quad
  c_{\alpha_i} = \cos \alpha_i, \quad s_{\alpha_i} = \sin \alpha_i
  \]
- Matriz base \( B \):
  \[
  B =
  \begin{bmatrix}
  r_{11} & r_{12} & r_{13} & p_x \\
  r_{21} & r_{22} & r_{23} & p_y \\
  r_{31} & r_{32} & r_{33} & p_z \\
  0 & 0 & 0 & 1
  \end{bmatrix}
  \]

---

### Expressões completas para \( S_{14}, S_{24}, S_{34} \)

\[
\boxed{
\begin{aligned}
S_{14} &= r_{11} \Big( a_1 c_1 + a_2 (c_1 c_2 - s_1 c_{\alpha_1} s_2) + d_2 s_1 s_{\alpha_1} + a_3 \big[ (c_1 c_2 - s_1 c_{\alpha_1} s_2) c_3 + (-c_1 s_2 c_{\alpha_2} - s_1 c_{\alpha_1} c_2 c_{\alpha_2} + s_1 s_{\alpha_1} s_{\alpha_2}) s_3 \big] \Big) \\
&\quad + r_{12} \Big( a_2 (-c_1 s_2 c_{\alpha_2} - s_1 c_{\alpha_1} c_2 c_{\alpha_2} + s_1 s_{\alpha_1} s_{\alpha_2}) + a_3 \big[ (-c_1 s_2 c_{\alpha_2} - s_1 c_{\alpha_1} c_2 c_{\alpha_2} + s_1 s_{\alpha_1} s_{\alpha_2}) c_3 + (c_1 s_2 s_{\alpha_2} + s_1 c_{\alpha_1} c_2 s_{\alpha_2} + s_1 s_{\alpha_1} c_{\alpha_2}) s_3 \big] \\
&\quad + a_1 c_1 \Big) \\
&\quad + r_{13} \Big( a_2 (c_1 s_2 s_{\alpha_2} + s_1 c_{\alpha_1} c_2 s_{\alpha_2} + s_1 s_{\alpha_1} c_{\alpha_2}) + a_3 \big[ (c_1 s_2 s_{\alpha_2} + s_1 c_{\alpha_1} c_2 s_{\alpha_2} + s_1 s_{\alpha_1} c_{\alpha_2}) c_3 + (-c_1 s_2 c_{\alpha_2} - s_1 c_{\alpha_1} c_2 c_{\alpha_2} + s_1 s_{\alpha_1} s_{\alpha_2}) s_3 \big] \\
&\quad + d_2 s_1 s_{\alpha_1} \Big) + p_x \\
\\
S_{24} &= r_{21} \Big( a_1 s_1 + a_2 (s_1 c_2 + c_1 c_{\alpha_1} s_2) - d_2 c_1 s_{\alpha_1} + a_3 \big[ (s_1 c_2 + c_1 c_{\alpha_1} s_2) c_3 + (-s_1 s_2 c_{\alpha_2} + c_1 c_{\alpha_1} c_2 c_{\alpha_2} - c_1 s_{\alpha_1} s_{\alpha_2}) s_3 \big] \Big) \\
&\quad + r_{22} \Big( a_2 (-s_1 s_2 c_{\alpha_2} + c_1 c_{\alpha_1} c_2 c_{\alpha_2} - c_1 s_{\alpha_1} s_{\alpha_2}) + a_3 \big[ (-s_1 s_2 c_{\alpha_2} + c_1 c_{\alpha_1} c_2 c_{\alpha_2} - c_1 s_{\alpha_1} s_{\alpha_2}) c_3 + (s_1 s_2 s_{\alpha_2} - c_1 c_{\alpha_1} c_2 s_{\alpha_2} - c_1 s_{\alpha_1} c_{\alpha_2}) s_3 \big] \\
&\quad + a_1 s_1 \Big) \\
&\quad + r_{23} \Big( a_2 (s_1 s_2 s_{\alpha_2} - c_1 c_{\alpha_1} c_2 s_{\alpha_2} - c_1 s_{\alpha_1} c_{\alpha_2}) + a_3 \big[ (s_1 s_2 s_{\alpha_2} - c_1 c_{\alpha_1} c_2 s_{\alpha_2} - c_1 s_{\alpha_1} c_{\alpha_2}) c_3 + (-s_1 s_2 c_{\alpha_2} + c_1 c_{\alpha_1} c_2 c_{\alpha_2} - c_1 s_{\alpha_1} s_{\alpha_2}) s_3 \big] \\
&\quad - d_2 c_1 s_{\alpha_1} \Big) + p_y \\
\\
S_{34} &= r_{31} \Big( d_1 + a_2 s_{\alpha_1} s_2 + d_2 c_{\alpha_1} + a_3 \big[ s_{\alpha_1} s_2 c_3 + (s_{\alpha_1} c_2 c_{\alpha_2} + c_{\alpha_1} s_{\alpha_2}) s_3 \big] \Big) \\
&\quad + r_{32} \Big( a_2 (s_{\alpha_1} c_2 c_{\alpha_2} + c_{\alpha_1} s_{\alpha_2}) + a_3 \big[ (s_{\alpha_1} c_2 c_{\alpha_2} + c_{\alpha_1} s_{\alpha_2}) c_3 + (-s_{\alpha_1} c_2 s_{\alpha_2} + c_{\alpha_1} c_{\alpha_2}) s_3 \big] + d_1 + d_2 c_{\alpha_1} \Big) \\
&\quad + r_{33} \Big( a_2 (-s_{\alpha_1} c_2 s_{\alpha_2} + c_{\alpha_1} c_{\alpha_2}) + a_3 \big[ (-s_{\alpha_1} c_2 s_{\alpha_2} + c_{\alpha_1} c_{\alpha_2}) c_3 + (s_{\alpha_1} c_2 c_{\alpha_2} + c_{\alpha_1} s_{\alpha_2}) s_3 \big] + d_1 + d_2 c_{\alpha_1} \Big) + p_z
\end{aligned}
}
\]

---

### Observações

- Cada termo dentro dos parênteses corresponde à posição do efetuador no frame 0 da DH.
- Os termos \(r_{ij}\) e \(p_x, p_y, p_z\) aplicam a transformação da base do robô para o frame 0 DH.
- Essa formulação é a multiplicação completa das 4 matrizes homogêneas em cascata, expressa explicitamente.

---

Se quiser, posso ajudar a formatar isso em LaTeX para seu TCC, ou gerar uma versão simbólica para facilitar a apresentação. Quer?


---
