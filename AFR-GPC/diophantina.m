function [E,F]=diofantina(Atil,n)
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
% This code is responsible for solving the Diophantine Equation of GPC
%
% MIT License

%Function for solving the Diophantine equation
%1=Ej(z^-1)*Ae(z^-1)+(z^-j)*Fj(z^-1)
%obtaining the polynomials Ej and F

m=size(Atil);d=(m(1,2)-1);              %size of A(z^-1) extended
E=zeros(n,d+1);                         %coefficient matrix E(z^-1)
F=zeros(n,d+1);F(1,:)=[1,zeros(1,d)];   %coefficient matrix F(z^-1)
for i=1:n
warning off;
[Ea,Fa]=deconv(F(i,:),Atil);             %division of F(z^-1) by A(z^-1) ext.
    if Fa(1,1)==0
        F(i,1:d)=Fa(1,2:d+1);k=n+1-i;k=k(1,1);
        E(i:n,i)=Ea*ones((n+1-i),1);
    end
    if i<n
    F(i+1,1:d)=Fa(1,2:d+1);
    end
end
