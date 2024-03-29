function xdot = moldebulg_ILC(t,x);
%
% SRcdFuzzy: A Software for simulating adaptive regulatory controllers of
% cyclical disturbances with frequency variations estimated from fuzzy
% logic 
%
% Developed by Rog�rio P. Pereira, Eduardo J. F. Andrade, Jos� L. F. 
% Salles, Carlos T. Valad�o, Ravena S. Monteiro, Gustavo Maia de Almeida, 
% Marco A. S. L. Cuadros, and Teodiano F. Bastos-Filho
%
% Version Feb 2024
% 
% This code has the model of the mold plant with bulging disturbance. 
%
% MIT License

global vl  uu bulg
 
  %constants for calculating the flow in the tank 
  Am=250000;
  h=1200;
  %Calculation of the gate valve area
  
d= 70; % Gate valve hole diameter (mm)
r= d/2;
at = ((75)^2)*3.1415926;     % total area of the gate valve hole (mm^2)
xs = x(1)+40; % xs = linear displacement of the gate valve, from the point of intersection of the holes 
%xs = xs*1000;

if x(1) >80 %valve travel limit
    x(1)=80;
end

%xt = 0; % xt = total displacement of the gate valve, considering "dead band"
%As= 0; % effective steel flow area
%cont = 1; % counter for vector formation

%for cont = 1:2400 % gate valve displacement from 0 to 120mm - POSITION stroke: 40 to 120 (80mm)
 %if ((xs > 40) & (xs <=115))    % considering valve deadband = 40mm
    As = 2*[r^2*acos((r-((xs-40)/2))/r) - ((r-((xs-40)/2))*sqrt((r*(xs-40))-((xs-40)/2)^2))]; % 
 
  
  
  %Calculation of the positioned output
  
  xdot(1)=-2.5*x(1) + 2.5*uu;
  
  %Mold level calculation
  
  xdot(2)=(1/Am)*(As*sqrt(2*9810*h)) - vl;
  
  %Level sensor
  
  xdot(3) = -4*x(3) +4*(x(2) + bulg);
  
  xdot = xdot';
  