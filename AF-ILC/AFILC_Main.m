%
% SRcdFuzzy: A Software for simulating adaptive regulatory controllers of
% cyclical disturbances with frequency variations estimated from fuzzy
% logic 
%
% Developed by Rogério P. Pereira, Eduardo J. F. Andrade, José L. F. 
% Salles, Carlos T. Valadão, Ravena S. Monteiro, Gustavo Maia de Almeida, 
% Marco A. S. L. Cuadros, and Teodiano F. Bastos-Filho
%
% Version Feb 2024
% 
% This code manages all the algorithms needed to run the AF-ILC 
% simulation. It calls the scripts: "moldebulg.m", which has the 
% model of the mold plant with bulging disturbance; "update_ILC_Var_N.m", 
% which updates the ILC matrix. Also "AFILC_main.m" updates the graphical 
% user interface. Additionally, in the main code, the file 
% "Fuzzy_AFR-GPC.fis" containing the Fuzzy Rules is read, and 
% furthermore, the routine that creates the cyclical disturbance is 
% executed.
%
% MIT License

Ni=200; N = Ni; % Number of samples for f=0.05==>N=200
warning('off');


HabBug = 1;
w = zeros(N,1); % Column for correction signal
%Update_ILC_Var_N;
update_ILC_Var_N;


FuzzyAFILC = readfis('Fuzzy_ILC');
    
global uu vl bulg % global variables that are used in the model
%uu controller output
%vl casting speed
%bulg,amplitude of bulguing 
vl=10;
tfinal = 2000;
dT = 0.1;   % sampling time
uu=0;       %initial value of uu
f=0.05;


% disp(' ');
% set(handles.text2, 'String', 'Iniciando processamento com simulação. Favor aguardar...')
disp(['Starting processing with simulation ', num2str(TipoSimu), ' with interval ', num2str(Tipo_intervalo)]);
disp('Please wait...');

switch Tipo_intervalo
    case 1

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%test from 200 to 190 (scenario1)
        if(N==200) f=0.05; f1=0.0526; vl1=10; vl2=10.52; end; %
        %%%%%%%%%%%%%%%%%%%%%%%%%teste para de 200 para 196 

    case 2
 %%%%%%%%%%%%%%%%%%%%%%%%test 200 to 185 (scenario 2)
        if(N==200) f=0.05; f1=0.054; vl1=10; vl2=10.80; end; 

   case 3 
 %%%%%%%%%%%%%%%%%%%%%%%% from 200 to 177
        if(N==200) f=0.05; f1=0.056; vl1=10; vl2=11.20; end; 
 
   end



%vl=200*FreqBug;
%bulging period is the inverse Tb = 1/f for f=0.05 Tb=20sec==> 200 samples per period
%N=200, f=0.05 e Vl=10;    N=167, f=0.06, Vl=0.06*200=12;
%Bulging local variable creation

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
Xs = [35 50 50];       % operating point
%Xs =[ valve position     mold level      level sensor]

% PID Controller Tuning

  kp = 0.3;
  ki = 0.01;
  kd = 0.5;
  k_geral= 0; %%general sample
  k = 1; %sample
  
  ts = 0;   %time variable used for the loop
  int_erro = 0; % Initializing the error integral
  erro = [];
  % %AFILC Parameters
  IAEj = 0;  IAE = []; AIAE(1:2)=0; Ae(1:2)=0;Nj(1:2)=0;
  IAE_Total = 0;
  Qz(1:2) = 0; Qcont = 0; Qz_temp=0;
  j_ilc = 0; TendN = 0; TendN_Ant =0; 
  Q_IAE = 0;  A_IAE =0; N_ant=0; Tend_IAE = 0; Tend_IAE_ant=0;
  delta=0; delta_ant=0; T_TendIAE = 1;
  T_TendN = 4; %cannot be less than 2
  
  IAE_Total=0;
  IAE_SegParte=0;
  x_old = 0;
  
set(handles.text2,'Visible','on');


  while (ts < tfinal)
      
      x_len = ts/tfinal*100;
      set(handles.text2,'String',['Processing... (', num2str(round(x_len)), '%)']);
      
      if (ts == tfinal)
          x_len = 100;
          progressBar = patch(handles.progressBarAxes, 'XData', [0 x_len x_len 0], 'YData', [0 0 1 1], 'FaceColor', 'b');
          xlim(handles.progressBarAxes, [0 100]);
          set(handles.progressBarAxes, 'XTick', [], 'YTick', []);    
          drawnow;
          x_old = x_len;
          pause(1);
      else if (x_len - x_old > 4)
              progressBar = patch(handles.progressBarAxes, 'XData', [0 x_len x_len 0], 'YData', [0 0 1 1], 'FaceColor', 'b');
              xlim(handles.progressBarAxes, [0 100]);
              set(handles.progressBarAxes, 'XTick', [], 'YTick', []);    
              drawnow;
              x_old = x_len;
          end
      end
      

      %vl update
      if(ts==0)           vl = vl1;           end
      if(ts==900)         vl = vl2;           end
      
      SP = 100;   
       k_geral= k_geral+ 1;

       ts = k_geral*dT;
   
%===========================  PID   ==============================
        erro(k_geral) = SP - Xs(3);
        %Calculation of the integral of the error
        int_erro = int_erro + erro(k_geral)*dT;

         %Calculation of the derivative of the error
        if (k_geral>1)
            d_erro = (erro(k_geral) - erro(k_geral-1))/dT;
        else
            d_erro = 0;
        end
        %PID Controller Output
        uu = kp * erro(k_geral) + ki * int_erro + kd * d_erro; 
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%for calculating the IAE part 2
if((k_geral==10005))
            Q_IAE = 0;
            up_rc(k_geral-N:k_geral)=0;
            IAE_SegParte=0;
        end
            IAE_SegParte = IAE_SegParte + abs(erro(k_geral));
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        IAE_Total = IAE_Total + abs(erro(k_geral));
        switch TipoSimu 
            case 1   % PID without buging
               wk=0;          HabFuzzy=0;        HabBug = 0;
            case 2    %PID With buging
               wk=0;          HabFuzzy=0;        HabBug = 1;
            case 3    %PID and ILC
               wk = w(k);     HabFuzzy=0;        HabBug = 1;
            case 4   %PID and ILC with Fuzzy
               wk = w(k);     HabFuzzy=1;        HabBug = 1;
            otherwise
               wk=0;
        end
            
        uu = uu + wk;
        
        bulg=HabBug*(Blg(k_geral,1)+Blg(k_geral,2)+Blg(k_geral,3)+Blg(k_geral,4));
      % bulg=Blg(k_geral,1);
        if uu > 35    uu = 35;    end
        if uu < -35   uu = -35;   end


         %Model execution
        [T,X] = ode45('moldebulg',[0 dT], Xs);
            %T = Execution time of ode45
            %X =[ valve position     mold level      measured on the senso]
            
            % 'moldebulg':model to be called
            % [0 dT] calculation period, dt (sampling period)
            
        n = length(T);   
        ys(k_geral,1) = X(n,3);
        Xs=[X(n,1) X(n,2) X(n,3)];  
        ej(k) = SP - Xs(3);
       
        if(ts>100)% waits for the transitional state to end
         if (k<(Ni-40))% the IAE will be calculated for a fixed range without regard to the fact that N varies
            IAEj = IAEj + abs(ej(k));
         end

         if (k==N)  %===========
             k=0;
             j_ilc = j_ilc + 1;
             %========== Calculation of Fuzzy inputs ===============
             IAE(j_ilc) = IAEj;
             IAEj = 0;           
             
             if j_ilc > 1   % % The first two were initialized to zero
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
                 
                 
                 N = N+delta; %Number of samples for f=0.05==>N=200               
            
                 update_ILC_Var_N;%update of L and Q 
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
         end  %===========  end do If K == N
        %Runs every sample after transient
         k = k + 1;    
      end
        
    
          % Variable storage
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
         vet_nivel(k_geral,:) = X(n,3); % stores the level value
         vet_tempo(k_geral) = ts; %stores time
         vet_curso_valvula(k_geral) = X(n,1);  %stores valve position
         vet_k(k_geral, :) = k; 
         vet_Tend_IAE(k_geral) = Tend_IAE;
  end

set(handles.text2,'Visible','off');




subplot(2,1,1,'Visible','on');
subplot(2,1,2,'Visible','on');

x_len = 0;
progressBar = patch(handles.progressBarAxes, 'XData', [0 x_len x_len 0], 'YData', [0 0 0 0], 'FaceColor', 'b');
xlim(handles.progressBarAxes, [0 100]);
set(handles.progressBarAxes,'XTick', [], 'YTick', []);
set(handles.progressBarAxes,'Visible','Off');


%%%%%%%%%%%%%%%%%%%%%%%%%%% with subtitles 
subplot(2,1,1,'Parent',handles.uipanel4); plot(vet_tempo, vet_nivel,'k', vet_tempo, vet_u,'b'); 
ylabel('Level(mm)'); xlabel('Time(s)'); grid on; xlim([116 2000]); ylim([82 118]);



%%%%%%%%%%%%%%%%%%
%title('Mold Level');
subplot(2,1,2,'Parent',handles.uipanel4); plot(vet_tempo, vet_N, 'r'); 
ylabel('N(samples)'); xlabel('Time(s)'); grid on; xlim([116 2000]); ylim([150 250]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
drawnow;


