function xdot = moldebulg_ILC(t,x);
  global vl  uu bulg
 
  %constantes para o calculo do fluxo no tanque 2
  Am=250000;
  h=1200;
  %Cálculo da área da válvula gaveta
  
d= 70; % Diametro do Furo da valvula Gaveta (mm)
r= d/2;
at = ((75)^2)*3.1415926;     % area total do furo da Valvula gaveta (mm^2)
xs = x(1)+40; % xs = deslocamento linear da valvula gaveta, a partir do ponto de intersecçao dos furos - o vetor eh "desloc"
%xs = xs*1000;

if x(1) >80 %limite do curso da válvula
    x(1)=80;
end

%xt = 0; % xt = deslocamento total da valvula gaveta, considerando "dead band"
%As= 0; % area efetiva de escoamento de aço 
%cont = 1; % contador para formaçao de vetores

%for cont = 1:2400 % deslocamento valvula gaveta de 0 a 120mm - curso POSCO: 40 a 120 (80mm)
 %if ((xs > 40) & (xs <=115))    % considerando banda morta da valvula = 40mm
    As = 2*[r^2*acos((r-((xs-40)/2))/r) - ((r-((xs-40)/2))*sqrt((r*(xs-40))-((xs-40)/2)^2))]; % Formula Fabio com banda morta
 
    %As = As/(10^6);
    
    %elseif cont == 1 
 %   as(cont) = 0;
 %else
 %   as(cont) = as(cont -1);
 %end;
 %   as2(cont) = 1600*pi + 3200*asin((xs-120)/80) + ((xs-120)/2)*sqrt(6400-(xs-120)^2); % Formula S.R.Yoo e Y.S.Kureon - Usina de POSCO 
 %   desloc(cont) = xs;
%    as2(cont) = -2*[(xs/2)*sqrt(r^2-xs^2) + (r^2/2)*asin(xs/r)] + at/2 % calculo metodo MLC02
 %   cont = cont + 1; 
 %   xs = xs + 0.05;
%end;
  
  %Cálculo da saída do posicionador
  
  xdot(1)=-2.5*x(1) + 2.5*uu;
  
  %Cálculo do nível do molde
  
  xdot(2)=(1/Am)*(As*sqrt(2*9810*h)) - vl;
  
  %Sensor de nível
  
  xdot(3) = -4*x(3) +4*(x(2) + bulg);
  
  xdot = xdot';
  