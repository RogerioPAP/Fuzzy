

Ni=200; N = Ni; % Número de amostras para f=0.05==>N=200

warning('off');


HabBug = 1;
w = zeros(N,1); % Column for correction signal
Atualiza_ILC_Var_N;


%%%%%%%%%%%%%%%com as 9 regrs
  FuzzyAFILC = readfis('FuzzyRogerilc4');
    %%%%%%%%%%%%%%%%%%%%%

    
global uu vl bulg % variável globais que são usadas no modelo
%uu saída do controlador
%vl velocidade de lingotamento
%bulg amplitude do bulg
vl=10;
tfinal = 2000;
dT = 0.1;   % tempo de amostragem
uu=0;       %valor inicial de uu
f=0.05;


% disp(' ');
% set(handles.text2, 'String', 'Iniciando processamento com simulação. Favor aguardar...')
disp(['Iniciando processamento com simulação ', num2str(TipoSimu), ' com intervalo ', num2str(Tipo_intervalo)]);
disp('Favor aguardar...');

switch Tipo_intervalo
    case 1

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%teste de 200 para 190 (cenário1)
        if(N==200) f=0.05; f1=0.0526; vl1=10; vl2=10.52; end; %
        %%%%%%%%%%%%%%%%%%%%%%%%%teste para de 200 para 196 

    case 2
 %%%%%%%%%%%%%%%%%%%%%%%% teste 200 para 185  (cenário 2)
        if(N==200) f=0.05; f1=0.054; vl1=10; vl2=10.80; end; 

   case 3 
 %%%%%%%%%%%%%%%%%%%%%%%% de 200 para para 177
        if(N==200) f=0.05; f1=0.056; vl1=10; vl2=11.20; end; 
 
   end



%vl=200*FreqBug;
%periodo do bulging é o inverso Tb = 1/f para f=0.05  Tb=20seg==> 200 amostras por periodo
%N=200, f=0.05 e Vl=10;    N=167, f=0.06, Vl=0.06*200=12;
%criação numa variável local do Bulging
%funcionou para N=100; f=0.05  Vl=20 mais ou menos para N=200 f=0.05 e
%Vl=10
for i=1:1:(tfinal/(2*dT) + 1)
    Blg(i,1)=9*sin(f*(i-1)*dT*(2*pi));
    Blg(i,2)=1*sin(2*f*(i-1)*dT*(2*pi));
    Blg(i,3)=0.5*sin(3*f*(i-1)*dT*(2*pi));
    Blg(i,4)=0.2*sin(4*f*(i-1)*dT*(2*pi));
end
for i=(tfinal/(2*dT) + 2):1:(tfinal/dT + 1)
    Blg(i,1)=9*sin(f1*(i-(tfinal/(2*dT) + 2))*dT*(2*pi));
    Blg(i,2)=1*sin(2*f1*(i-(tfinal/(2*dT) + 2))*dT*(2*pi));
    Blg(i,3)=0.5*sin(3*f1*(i-(tfinal/(2*dT) + 2))*dT*(2*pi));
    Blg(i,4)=0.2*sin(4*f1*(i-(tfinal/(2*dT) + 2))*dT*(2*pi));
end 
Xs = [35 50 50];       % ponto de operaçao
%Xs =[ posicao da válvula     nível do molde      medição do sensor]

% Sintonia do Controlador PID
  kp = 0.3;
  ki = 0.01;
  kd = 0.5;
  k_geral= 0; %amostra geral
  k = 1; %amostra 
  k_fft = 0; %contador de amostras para o fft
  j_fft = 1; %contador de iteração
  ts = 0;  %variável de tempo usada para o loop
  int_erro = 0; % Inicializando a integal do erro
  erro = [];
  %Prâmetros do AFILC
  IAEj = 0;  IAE = []; AIAE(1:2)=0; Ae(1:2)=0;Nj(1:2)=0;
  IAE_Total = 0;
  Qz(1:2) = 0; Qcont = 0; Qz_temp=0;
  j_ilc = 0; TendN = 0; TendN_Ant =0; 
  Q_IAE = 0;  A_IAE =0; N_ant=0; Tend_IAE = 0; Tend_IAE_ant=0;
  delta=0; delta_ant=0; T_TendIAE = 1;
  T_TendN = 4; %não pode ser menor do que 2 dá pau
  
  IAE_Total=0;
  IAE_SegParte=0;
  
  while (ts < tfinal)
      %atualização do vl 
      if(ts==0)           vl = vl1;           end
      if(ts==900)         vl = vl2;           end
      
      SP = 100;   
       k_geral= k_geral+ 1;

       ts = k_geral*dT;
   
%===========================  PID   ==============================
        erro(k_geral) = SP - Xs(3);
        %Cálculo da Integral do erro
        int_erro = int_erro + erro(k_geral)*dT;

        %Cálculo da derivada do erro
        if (k_geral>1)
            d_erro = (erro(k_geral) - erro(k_geral-1))/dT;
        else
            d_erro = 0;
        end
        %Saída do Controlador PID
        uu = kp * erro(k_geral) + ki * int_erro + kd * d_erro; 
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%para cálculo do IAE parte2
if((k_geral==10005))
            Q_IAE = 0;
            up_rc(k_geral-N:k_geral)=0;
            IAE_SegParte=0;
        end
            IAE_SegParte = IAE_SegParte + abs(erro(k_geral));
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        IAE_Total = IAE_Total + abs(erro(k_geral));
        switch TipoSimu 
            case 1   %So PID sem bug
               wk=0;          HabFuzzy=0;        HabBug = 0;
            case 2   %Pid com bug
               wk=0;          HabFuzzy=0;        HabBug = 1;
            case 3   %PID e ILC
               wk = w(k);     HabFuzzy=0;        HabBug = 1;
            case 4   %PID e ILC com Fuzzy
               wk = w(k);     HabFuzzy=1;        HabBug = 1;
            otherwise
               wk=0;
        end
            
        uu = uu + wk;
        
        bulg=HabBug*(Blg(k_geral,1)+Blg(k_geral,2)+Blg(k_geral,3)+Blg(k_geral,4));
      % bulg=Blg(k_geral,1);
        if uu > 35    uu = 35;    end
        if uu < -35   uu = -35;   end


        %Execução do modelo
        [T,X] = ode45('moldebulg',[0 dT], Xs);
            %T = Tempo de execução do ode45
            %X =[ posicao da válvula     nível do molde      medição do sensor]
            % 'moldebulg': modelo a ser chamado
            % [0 dT] periodo do calculo que seria sempre o dt (periodo de amostrag
        n = length(T);   
        ys(k_geral,1) = X(n,3);
        Xs=[X(n,1) X(n,2) X(n,3)];  
        ej(k) = SP - Xs(3);
       
        if(ts>100)% aguarda acabar o estado transitoria
         if (k<(Ni-40))% o IAE será calculado para uma faixa fixa sem importar a varia;'ao de N
            IAEj = IAEj + abs(ej(k));
         end

         if (k==N)  %===========
             k=0;
             j_ilc = j_ilc + 1;
             %========== Cálculo das entradas do Fuzzy ===============
             IAE(j_ilc) = IAEj;
             IAEj = 0;           
             
             if j_ilc > 1   % Os dois primeiros foram inicializados como zero
                 AIAE(j_ilc) = IAE(j_ilc)-IAE(j_ilc-1);
                 A_IAE = AIAE(j_ilc);
                 Q_IAE = Q_IAE + sign(A_IAE);
                 Tend_IAE_ant = Tend_IAE;
                 Tend_IAE = A_IAE/5;
                 if (Tend_IAE>1)        Tend_IAE=1;          end
                 if (Tend_IAE<-1)       Tend_IAE=-1;         end
                 
                                
                 Nj(j_ilc)=N;
                 TendN_Ant = TendN;
                 if j_ilc > T_TendN

                     
                     x_TendN = vet_tempo((k_geral-T_TendN*N):k_geral-1);
                     y_TendN = vet_N((k_geral-T_TendN*N):k_geral-1);
                     p_TendN = polyfit(x_TendN,y_TendN,1);
                     TendN = p_TendN(1)*100;
                     if (TendN>1)        TendN=1;          end
                     if (TendN<-1)       TendN=-1;         end
                     
                 end
                               
                 if((Q_IAE<0))%||(delta-delta_ant~=0))
                     Q_IAE=0;
                 end
                 Ae(j_ilc) = erro(k_geral) - erro(k_geral-1);

                   SaidaFuzzy = evalfis([IAE(j_ilc) TendN Q_IAE Tend_IAE],FuzzyAFILC);
%                  
                 
                 delta = round(SaidaFuzzy)*HabFuzzy;
                 
                 
                 N = N+delta; % Número de amostras para f=0.05==>N=200                   
            
                 Atualiza_ILC_Var_N;%L e Q atualizado
                 if(N > length(w))
                     w = [ w; (w(1)+   (w(length(w)))/2)*ones(N-length(w),1)];
                     % ej = [ ej (ej(1)+(ej(length(ej)))/2)*ones(1, N-length(ej))];
                     ej = [ ej zeros(1, N-length(ej))];
                 else
                     w = w(1:N);
                     ej = ej(1:N);
                 end
             end
             
         w = Q*(w+L*ej'); % The updated correction signal
         end  %=========== Fim do If K == N
         %Roda a cada amostragem após transitório
         k = k + 1;    
      end
        
    
         % Armazenamento de variaveis
         if j_ilc>1         
             vet_IAE(k_geral) = IAE(j_ilc); 
             vet_AIAE(k_geral) =  A_IAE;
         else
             vet_IAE(k_geral) = 0;
             vet_AIAE(k_geral) =  0;
         end
         
             
         vet_u_ILC(k_geral) = wk;             
         vet_N(k_geral) = N;
         vet_TendN(k_geral) = TendN;
         vet_QIAE(k_geral) = Q_IAE;
         vet_u(k_geral, :) = uu; 
         vet_nivel(k_geral,:) = X(n,3); % armazena o valor do nível
         vet_tempo(k_geral) = ts;  %armazena o tempo
         vet_curso_valvula(k_geral) = X(n,1);  % armazena posição da válvula
         vet_k(k_geral, :) = k; 
         vet_Tend_IAE(k_geral) = Tend_IAE;
  end

set(handles.text2,'Visible','off');
  
  subplot(2,1,1,'Visible','on');
subplot(2,1,2,'Visible','on');


%   set(handles.axes1)
%%%%%%%%%%%%%%%%%%%%%%%%%%% com legenda
subplot(2,1,1,'Parent',handles.uipanel4); plot(vet_tempo, vet_nivel,'k', vet_tempo, vet_u,'b'); 
ylabel('Nivel(mm)'); xlabel('Tempo(s)'); grid on; xlim([116 2000]); ylim([82 118]);



%%%%%%%%%%%%%%%%%%retirando iae, pedido zeleandro com lengenda
%title('Nível do Molde');
subplot(2,1,2,'Parent',handles.uipanel4); plot(vet_tempo, vet_N, 'r'); 
ylabel('N(amostras)'); xlabel('Tempo(s)'); grid on; xlim([116 2000]); ylim([150 250]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
drawnow;
  