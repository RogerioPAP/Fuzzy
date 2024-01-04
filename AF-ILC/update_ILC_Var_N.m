%==== ILC  START of Parameter Initialization ==========================
%Initializing ILC Parameters
ilc_kp = 0.3; % ILC -Proportional gain
ilc_kd = 0.3; % ILC - Derivative Gain
ilc_qw = 0.03; % Q-filter pass down -
ilc_qn = 3; % Filter Order

% Now prepare the input signals (r,v,e,w)
stde = 1e-4; % maximum error white noise value
stdv = 1e-1; % maximum white noise value of the disturbance


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

% Creating the learning matrix L
U = [[zeros(N-1,1),eye(N-1)];[0,zeros(1,N-1)]]; % Creating a shifted identity matrix
L = ((ilc_kp + ilc_kd) * U) - (ilc_kd * eye(N)); % Ceating learning algorithom matrix

%==== ILC   END of Parameter Initialization =============================