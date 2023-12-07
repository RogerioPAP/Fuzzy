 function [E,F]=diofantina(Atil,n)
%Funçao para a soluçao da equaçao diofantina
%1=Ej(z^-1)*Ae(z^-1)+(z^-j)*Fj(z^-1)
%obtendo os polinomios Ej e Fj

m=size(Atil);d=(m(1,2)-1);              %tamanho de A(z^-1) extendida
E=zeros(n,d+1);                         %matriz de coeficientes E(z^-1)
F=zeros(n,d+1);F(1,:)=[1,zeros(1,d)];   %matriz de coeficientes F(z^-1)
for i=1:n
warning off;
[Ea,Fa]=deconv(F(i,:),Atil);             %divisao de F(z^-1) por A(z^-1) ext.
    if Fa(1,1)==0
        F(i,1:d)=Fa(1,2:d+1);k=n+1-i;k=k(1,1);
        E(i:n,i)=Ea*ones((n+1-i),1);
    end
    if i<n
    F(i+1,1:d)=Fa(1,2:d+1);
    end
end
