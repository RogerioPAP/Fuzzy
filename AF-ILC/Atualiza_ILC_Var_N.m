%==== ILC   INICIO da Inicialiação de Parâmetros ==========================
%Inicialização dos parâmetros do ILC
ilc_kp = 0.3; % ILC Ganho proporcional
ilc_kd = 0.3; % ILC Ganho Derivativo
ilc_qw = 0.03; % Q-filter passa baixo frequência de corte
ilc_qn = 3; % Ordem do Filtro

% Now prepare the input signals (r,v,e,w)
stde = 1e-4; % valor máximo do ruído branco do erro
stdv = 1e-1; % valor máximo do ruído branco da perturbação


% input is [r,v,e,w]
% output is [d,u,y]
% Creating the outputs store
Wj = []; % All ILC correction signal
Yj = []; % All magnetic field output
Ej = []; % All errors55

% Creating a zero phase lowpass Q-filter
[qa,qb,qc,qd] = butter(ilc_qn,ilc_qw);
Qq = ss(qa,qb,qc,qd,-1);
h = impulse(Qq,N); h = h(1:N);

H = toeplitz(h,[h(1);zeros(N-1,1)]);
Q = (H*H'+ H'*H)/2;

% Criando a matriz de aprendizado L
U = [[zeros(N-1,1),eye(N-1)];[0,zeros(1,N-1)]]; % Creating a shifted identity matrix
L = ((ilc_kp + ilc_kd) * U) - (ilc_kd * eye(N)); % Ceating learning algorithom matrix

%==== ILC   FIM da Inicialiação de Parâmetros =============================