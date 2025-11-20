function Phases_BS=array_response_phases_BS(pA,pU,freq)

% Number of Antenna elements
N=size(pA,2);
% setting x_u,y_u and z_u
x_U=pU(1);y_U=pU(2);z_U=pU(3);

% Wavelength
lambda=3*10^8/freq;
%Initializarion of the Array response of the BS
%Phases=zeros(N,1);
Phases_BS=[];

% Wave-vector S calculation
%S=zeros(1,3);
S=[];
S=[S, 2*(pi/lambda)*x_U/(sqrt(x_U^2+y_U^2+z_U^2))];
S=[S, 2*(pi/lambda)*y_U/(sqrt(x_U^2+y_U^2+z_U^2))];
S=[S, 2*(pi/lambda)*z_U/(sqrt(x_U^2+y_U^2+z_U^2))];

for n=1:N
    Phases_BS=[Phases_BS; (S*pA(:,n))];
end



end