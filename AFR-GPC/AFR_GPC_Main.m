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
% This code manages all the algorithms to run the AFR-GPC simulation. It 
% calls the scripts: "diophantine.m", responsible for solving the 
% Diophantine Equation of GPC; "moldebulg.m", which has the model of the 
% mold plant with bulging disturbance. Also "AFR_GPC_main.m" updates the 
% graphical user interface. Additionally, this code calls the file 
% "Fuzzy_AFR-GPC.fis" containing the Fuzzy Rules, and furthermore, the 
% routine that creates the cyclical disturbance is executed.
%
% MIT License



% clc; clear all; close all;

%N = 200; % Number of samples for f=0.05==>N=200
%N=167;
%Mudar Ni Ni=200 e Ni=167
Ni=200; N = Ni; % Number of samples for f=0.05==>N=200

%%%%

warning('off');



%%%%%%%%%%%%%%%%%%%%
FuzzyAFILC = readfis('Fuzzy_AFR-GPC');
%%%%%%%%%%%%%%%%%%%%%%%


% IAE (0 1000) tendn (-15  15)  Ae(-1000  1000)  Aez(-5  5)   AIAE(-1000
% 1000)

global uu vl bulg % variável globais que são usadas no modelo
%uu controller output
%vl casting speed
%bulg bulguing amplitude

tfinal = 2000;
dT = 0.1;   % sampling time
uu=0;       %initial value of uu
II=round(tfinal/dT);
r = 100*ones(II+100,1);

%ve_li = 20*ones(1,II);%2,2m/min
%ve_li(9000:18000) = 22.2; %1,2m/mim
%ve_li(18000:27000) = 25;%0.05hz, 
%ve_li(27000:end) = 28.57;%1.6m/min

f=0.05;

%%%%%%%%%%%%%%%%%%%%%%%%%%%

% if(N==200) f=0.05; f1=0.0526; vl1=10; vl2=10.52; end; %f=0.05 N=200 vl=10 --- f=0.0526   N=190  vl=10.52
% 
% %if(N==200) f=0.05; f1=0.06; vl1=10; vl2=12; end;
% 
% if(N==167) f=0.06; f1=0.0637; vl1=12; vl2=12.74; end; %f=0.06 N=167 vl=10 --- f=0.0637   N=157  vl=12.74

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555



disp(' ');
disp(['Starting processing with simulation ', num2str(TipoSimu), ' with interval ', num2str(Tipo_intervalo)]);
disp('Please wait..');
switch Tipo_intervalo
    case 1        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%test from 200 to 190 - scenario 1  %%%%%%%%%%%%%%%%%%%%
if(N==200) f=0.05; f1=0.0526; vl1=10; vl2=10.52; end; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 2
%%%%%%%%%%%%%%%%%%%%%%%%RGPC-200 to 185-scenario 2 %%%%%%%%%%%%%%%%%%%%%%%%2%%%%%%%%%%%%
if(N==200) f=0.05; f1=0.054; vl1=10; vl2=10.80; end; 
%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
 case 3
%%%%%%%%%%%%%%%%%%%%%%%% RGPC-200 to 177- scenario 3
if(N==200) f=0.05; f1=0.0565; vl1=10; vl2=11.20; end; 
%%%%%%%%%%%%%%%%%%%%%%%%%% 
case 4
  %%%%%%%%%%%%%%%%%%%%%%%% RGPC-200 to 175

if(N==100) f=0.1; f1=0.11; vl1=20; vl2=22; end;
end

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


% figure
% plot(Blg)
dist=Blg; 
%vl = 20;  %initial value of casting speed
Xs = [35 100 100];       % operating point
%Xs =[ valve position     mold level      Level sensor]
 

 % PID Controller Tuning
  %kp = 0.3;
  %ki = 0.01;
  %kd = 0.5;
  
  %%%%%Filter tuning%%%%%%%%%%%
  hc = 8; 
  hp = 9; 
  lambda_n = 297.3468;
  alfa = 0.9777;
  delta_n = 712.6945;
  delta_r = delta_n;
  lambda_r = 62.0452;
  
  G = zeros(hp,hc); %initializing the matrix G
  y=zeros(II+hp,1); %initializing the plant output with predictive
  y_n=zeros(II+hp,1);
  y_r=zeros(II+hp,1);
  Du=zeros(II+hp,1); %initializes the control differential vector
  Du_n=zeros(II+hp,1);
  u=zeros(II+hp,1);%initializes the system control vector
  u_n=zeros(II+hp,1);
  u_r=zeros(II+hp,1);
  DNu=zeros(II+hp,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%Polynomials A and B
  d = 0;
  A = [1 -1.822 0.822];
  B = [0.01924];
  
  %Second Order Filter

  q0 = 0.85;
  q1 = (1-q0)/2;
  Hfiltro = [q1 q0 q1];
  
  
  k_geral= N+5; %general sample
  k = 1; %sample
  
  ts = 0;  %time variable used for the loop
  int_erro = 0; % Initializing the error integral
  erro = [];
  %Parameters
  IAEj = 0;  IAE = []; AIAE(1:2)=0; Ae(1:2)=0;
  IAE_Total = 0;
  Qz(1:2) = 0; Qcont = 0; Qz_temp=0;
  j_ilc = 0; TendN = 0; Q_IAE = 0;  A_IAE =0; N_ant=0; Tend_IAE = 0;
  delta=0;
  T_TendN = 4; %cannot be less than 2 
  
  IAE_Total=0;
  IAE_SegParte=0;
  
  x_old = 0;
  
  set(handles.text2,'Visible','On');
  
  while (ts < tfinal)
      
      x_len = ts/tfinal*100;
      set(handles.text2,'String',['Processing... (', num2str(round(x_len)), '%)']);
      
      if (ts == tfinal)
          x_len = 100;
          progressBar = patch(handles.axes5, 'XData', [0 x_len x_len 0], 'YData', [0 0 1 1], 'FaceColor', 'b');
          xlim(handles.axes5, [0 100]);
          set(handles.axes5, 'XTick', [], 'YTick', []);    
          drawnow;
          x_old = x_len;
          pause(1);
      else if (x_len - x_old > 3)
              progressBar = patch(handles.axes5, 'XData', [0 x_len x_len 0], 'YData', [0 0 1 1], 'FaceColor', 'b');
              xlim(handles.axes5, [0 100]);
              set(handles.axes5, 'XTick', [], 'YTick', []);    
              drawnow;
              x_old = x_len;
          end
      end
      
      if(ts==0)           vl = vl1;           end
      if(ts==900)         vl = vl2;           end
      
      SP = 100;   
       k_geral= k_geral+ 1;

       ts = k_geral*dT;
    %       if ts < tfinal/2    uu = 35;     else        uu = 0;    end
      %%%%%%%%%%%%%%%%%%%%Preditivo%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    DN = ones(1,N+2);
    DN(2:N-1) = 0;
    DN(N:end) = -Hfiltro;
    
    
    %Calculating A times Deltaa
    Atil_r = conv(A,DN);

    Atil_n = conv(A,[1 -1]);

    %ii=0;

    %Carrying out the Diophantine Equation
    [E_r,F_r] = diophantina(Atil_r,hp);

    [E_n,F_n] = diophantina(Atil_n,hp);

    F_r(:,end)=[];

    F_n(:,end)=[];
    jjj=0;
    %Carrying out the Diophantine Equation
    H_r=[];
    for jjj = d+1:hp
        H_r(jjj,:) = conv((E_r(jjj,:)),B);
        H_n(jjj,:) = conv((E_n(jjj,:)),B);
    end
    %Taking the parameters of Matrix H
    gy_r = H_r(hp,:)';
    gy_n = H_n(hp,:)';

    iii = 0;
    jjj = 0;
    %Assembling the G Matrix
    for iii = 1:hp
        for jjj = 1:hp
            if (jjj+iii-1)<= hp  
                G1_r(jjj+iii-1,iii) = gy_r(jjj);
                G1_n(jjj+iii-1,iii) = gy_n(jjj);
            end
        end
    end
    iii=0;
    jjj=0;
    for iii = 1:hc
        for jjj = 1:hp
            if (jjj+iii-1)<= hp  
                G_r(jjj+iii-1,iii) = gy_r(jjj);
                G_n(jjj+iii-1,iii) = gy_n(jjj);
            end
        end
    end
    % Calculation of G_linha
    cci = 0;
    jj = 0;
    kk = 0;
    iii=0;
    %G_linha = zeros(hp,hp);
    for iii = 1:hp %Number of lines equal to the prediction horizon
        G_aux2_r = H_r(d+iii,:);
        G_aux2_n = H_n(d+iii,:);
        G2_r = G1_r(iii,:);
        G2_n = G1_n(iii,:);
    
        in_r = max(find(G_aux2_r));
        in_n = max(find(G_aux2_n));
        in2_r = max(find(G2_r));
        in2_n = max(find(G2_n));
        if isempty(in2_r)==1
            in2_r = 1;
        end
        if isempty(in2_n)==1
            in2_n = 1;
        end
     
        SS_r = (G_aux2_r(in2_r+1:in_r));
        SS_n = (G_aux2_n(in2_n+1:in_n));
        if length(B) > 1
            if isempty(SS_r)==1
                SS_r = 10000;
            end
            if isempty(SS_n)==1
                SS_n = 10000;
            end
        if isnan(SS_r)==1
            SS_r = 10000;
        end
        if isnan(SS_n)==1
            SS_n = 10000;
        end
            G_linha_r(iii,:) = SS_r;
            G_linha_n(iii,:) = SS_n;
        else
            G_linha_r(iii,:) = 0;
            G_linha_n(iii,:) = 0;
        end
    end
    iii = 0;
    jj = 0;

    P = eye(hc);

    K_n = delta_n*inv(delta_n*G_n'*G_n + lambda_n*P)*G_n';
    K_n = K_n(1,:);

 
    BB = G_r';
    GG = 2.*(delta_r*BB*G_r + lambda_r*eye(hc));


    
    y_n(k_geral,1)=1.822*y_n(k_geral-1,1) - 0.822*y_n(k_geral-2,1) + 0.01924*u_n(k_geral-1,1);
    
    % Calculate Du_n control action
    % Calculating the Future Reference
    %i=0;
    W_n(1,1) = alfa*y_n(k_geral,1) + (1-alfa)*r(k_geral+1,1);
    for i = 2:hp
        W_n(i,1) = alfa*W_n(i-1,1) + (1-alfa)*r(k_geral+i,1);
    end
    %Free Response Calculation
    i = k_geral;

    [lin_n,col_n]=size(G_linha_n);
    kk=0;

    for i = 4:4+(col_n-1)
        kk=kk+1;
        Du1_n(kk) = Du_n(k_geral-kk);
    end

    [lin2_n,col2_n]=size(F_n);
    kk=0;
    ww = 1;
    for i = 4:4+(col2_n-1)
        y1_n(ww) = y_n(k_geral-kk);
        kk=kk+1;
        ww=ww+1;
    end

    i=0;

    if isempty(G_linha_n) == 0
        yf1_n = G_linha_n*Du1_n';
    else
        yf1_n = 0;
    end

    yf2_n = F_n*y1_n';

    yf_n = yf1_n + yf2_n;
    
    Du_n(k_geral,1) = K_n*(W_n-yf_n);
    u_n(k_geral,1) = Du_n(k_geral,1) + u_n(k_geral-1,1);
    a = u_n(k_geral,1);
    % Get yr using the measured value of the plant
    
       
   bulg = Blg(k_geral,1)+Blg(k_geral,2)+Blg(k_geral,3)+Blg(k_geral,4);
   uu = u(k_geral-1,1);  
%===========================  PID   ==============================
        erro(k_geral) = SP - Xs(3);
        
 IAE_Total = IAE_Total + abs(erro(k_geral));
        switch TipoSimu 
            case 1   %RGPC
               HabFuzzy=0;   
            otherwise   %RGPC with Fuzzy
               HabFuzzy=1;
         end  
        
       
       
     
       
        if uu > 70    uu = 70;    end
        if uu < 0   uu = 0;   end

        %Model execution
        [T,X] = ode45('moldebulg',[0 dT], Xs);
            %T = ode45 runtime
            %X =[ valve position     mold level      Level sensor]
            % 'moldebulg': model to be called
            % [0 dT] calculation period, dt (sampling period)
            % sampling
        n = length(T);   
        y(k_geral,1) = X(n,3);
        Xs=[X(n,1) X(n,2) X(n,3)];  
        ej(k) = SP - Xs(3);
       
         y_r(k_geral,1) = y(k_geral,1) - y_n(k_geral,1);
        
           %Get u_r
    
    
    
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555
    W_r(1,1) = y_r(k_geral,1);
    for i = 2:hp
        W_r(i,1) = (y_r(k_geral,1));
    end
    %W = W(d+1:hp);
    W_r;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Free Response Calculation
    i = k_geral;
    [lin_r,col_r]=size(G_linha_r);
    kk=0;
    for i = 4:4+(col_r-1)
        kk=kk+1;
        Du1_r(kk) = DNu(k_geral-kk);
    end

    [lin2_r,col2_r]=size(F_r);

    kk=0;
    ww = 1;
    y1_r = [];
    for i = 4:4+(col2_r-1)
        y1_r(ww) = y_r(k_geral-kk);
        kk=kk+1;
        ww=ww+1;
    end
    i=0;

    if isempty(G_linha_r) == 0
        yf1_r = G_linha_r*Du1_r';
    else
        yf1_r = 0;
    end
    yf2_r = [];
    yf2_r = F_r*y1_r';
    yf_r = yf1_r + yf2_r;
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    %Restriction on Control Action u
 umax = 70;
 %umax = 30;
 umin = 0;
 I = eye(hc,hc);
 II = [I;-I];
 gama1 = ones(hc,1)*(umax-u(k_geral-N,1));
 gama2 = ones(hc,1)*(u(k_geral-N,1)-umin);
 gama = [gama1;gama2];

   
%Restriction on Control Signal Variation
 Dumax = 80;
 Dumin = -80;
 S0 = eye(hc,hc);
 [l,c] = size(S0);
 jj=0;
 for i = 2:l
     for jj = 1:c
         if i == jj
            S0(i,jj-1) = -1;
         end
     end
 end

 SS = [S0;-S0];
% 
 %CC1 = I*Dumax;
 %CC2 = I*Dumin;
 
 CC1 = ones(hc,1)*Dumax;
 CC2 = -ones(hc,1)*Dumin;
 
 CC = [CC1;CC2];
% 
 c0 = zeros(hc,1);
 c0(1) = 1;
% 
 D01 = c0*DNu(k_geral-1);
 D02 = -c0*DNu(k_geral-1);
 DD = [D01;D02];
% 
 EE1 = ones(hc,1)*Du(k_geral-N);
 EE2 = ones(hc,1)*Du(k_geral-N);
% 
 RR1 = [CC1+D01-EE1];
 RR2 = [CC2+D02+EE2];
% 
 RR = [RR1;RR2];
 
 
%Restriction on the N-periodic variation of the control action
  DNumax = 80;
  DNumin = -80;
  I = eye(hc,hc);
  II = [I;-I];
  BB1 = ones(hc,1)*DNumax;
  BB2 = -ones(hc,1)*DNumin;
  BB = [BB1; BB2];
% % %

AAA = [II;II;SS];
GAMA = [gama;BB;RR];

%%%%%%Control Signal Calculation%%%%%%%%%%%%%

FF = 2.*delta_r*(yf_r)'*G_r;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%to calculate the IAE part 2
if((k_geral==10005))
            Q_IAE = 0;
            up_rc(k_geral-N:k_geral)=0;
            IAE_SegParte=0;
        end
            IAE_SegParte = IAE_SegParte + abs(erro(k_geral));
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%DNu calculation
try
     DD = quadprog(GG,FF,AAA,GAMA);
     DNu(k_geral,1) = DD(1);
     
     u_r(k_geral,1) = u_r(k_geral-N,1) + DNu(k_geral,1);
     ab = u_r(k_geral,1);
     
     
catch
    ise = 1e18 
    aaa = true
    break
end
 
    
   
     u(k_geral,1) = u_n(k_geral,1) + u_r(k_geral,1);
  
     Du(k_geral,1) = u(k_geral,1) - u(k_geral-1,1);
    
   
        if(ts>100)% waits for the transitional state to end
         if (k<150)% the IAE will only be calculated on the first 150 samples
            IAEj = IAEj + abs(ej(k));
         end
         if ((abs(erro(k_geral))<0.2 && sign(erro(k_geral)) ~= sign(erro(k_geral-1))) && Qcont>20) % deteção de zero
             Qz_temp = Qcont;  
             Qcont=0;
         else
               Qcont=Qcont+1;
         end

         if (k==N)  %===========
             k=0;
             j_ilc = j_ilc + 1;
             %========== Calculation of Fuzzy inputs ===============
             IAE(j_ilc) = IAEj;
             IAEj = 0;           
             
             if j_ilc > 1   % The first two were initialized to zero
                 AIAE(j_ilc) = IAE(j_ilc)-IAE(j_ilc-1);
                 A_IAE = AIAE(j_ilc);
                 Q_IAE = Q_IAE + sign(A_IAE);
                 %Tend_IAE = sign(A_IAE);
                 Tend_IAE_ant = Tend_IAE;
                 Tend_IAE = A_IAE/5;
                 if (Tend_IAE>1)        Tend_IAE=1;        end
                 if (Tend_IAE<-1)       Tend_IAE=-1;       end
                 

                 if j_ilc > T_TendN

                     x_TendN = vet_tempo((k_geral-T_TendN*N):k_geral-1);
                     y_TendN = vet_N((k_geral-T_TendN*N):k_geral-1);
                     p_TendN = polyfit(x_TendN,y_TendN,1);
                     TendN = p_TendN(1)*100;
                     if (TendN>1)        TendN=1;          end
                     if (TendN<-1)       TendN=-1;         end
                     
                 end

                 if(Q_IAE<0)
                     Q_IAE=0;
                 end
                 Ae(j_ilc) = erro(k_geral) - erro(k_geral-1);
                 Aez(j_ilc) = erro(k_geral) - erro(k_geral-N);
                 Qz(j_ilc) = Qz_temp;
                 
                 SaidaFuzzy = evalfis([IAE(j_ilc) TendN Q_IAE Tend_IAE],FuzzyAFILC);
                 delta = round(SaidaFuzzy);
                 N_ant = N;
                 
                 %%%%%%%%%%%%%%%%%%
                 N = N+HabFuzzy*delta; % Number of samples for f=0.05==>N=200
     
                 %Update_ILC_Var_N;%L and Q updated
                 k=1;
  
              end

         end  %=========== End of If K == N
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
         
         if j_ilc > 2
             vet_Qz(k_geral) = Qz(j_ilc);
         else
             vet_Qz(k_geral) = 0;
         end
             
             
         vet_N(k_geral) = N;
         vet_TendN(k_geral) = TendN;
         vet_QIAE(k_geral) = Q_IAE;
         vet_u(k_geral, :) = uu; 
         vet_nivel(k_geral,:) = X(n,3); % stores the level value
         vet_tempo(k_geral) = ts;  %stores time
         vet_curso_valvula(k_geral) = X(n,1);  % stores valve position
         vet_k(k_geral, :) = k; 
         vet_Tend_IAE(k_geral) = Tend_IAE;

  end

set(handles.text2,'Visible','off');
    
      %     %%%%%%%%%%%%%%%%%%%%%%%%%
  subplot(2,1,1,'Parent',handles.uipanel4); plot(vet_tempo, vet_nivel,'k', vet_tempo, vet_u,'b'); 
  ylabel('Level(mm)'); xlabel('Time(s)'); grid on; xlim([116 2000]); ylim([95 105]);
   
 
  subplot(2,1,2,'Parent',handles.uipanel4); plot(vet_tempo, vet_N, 'R'); 
  
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%sem N(samples)
  ylabel('N(samples)'); xlabel('Time(s)'); grid on; xlim([116 2000]); ylim([170 210]);
 % legend('N'); xlabel('Time(s)'); grid on; xlim([116 2000]); ylim([195 205]);
 disp('Processing Complete. See the images in the interface.');
 