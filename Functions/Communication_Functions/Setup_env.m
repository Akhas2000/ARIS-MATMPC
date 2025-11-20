function [pA,pR,pK]=Setup_env(N,K,M_size,freq)




%N : Number of antenna elements
%M=[M_H,M_V]: Number of RIS elements
%K : Number of Users
%pA: Antenna elements position pA(:,n)
%pR: RIS elements position pR(:,m)
%pK: Users position pK(:,k)

%Area 



x_min=140;
x_max=150;
y_min=-10;
y_max=10;

%Base station Height
h_BS=68;

% getting horizontal and vertical number of elements
M_H=M_size(1);
M_V=M_size(2);
M=M_H*M_V;

lambda=3*10^8/freq;
%Horizontal and vertical distances of antenna elements and RIS
d_HA=lambda/2;
d_VA=lambda/2;

d_HR=lambda/2;
d_VR=lambda/2;

%Initialization of Antenna, RIS and Users positions
pA=zeros(3,N);
pR=zeros(3,M);
pK=zeros(3,K);

%pA filling
for n=1:N
    pA(1,n)=d_HA;
    pA(3,n)=(n-1)*d_VA;
end

%pR filling
for m=1:M_V*M_H
    %Vertical and Horizontal indexes
    m_V=floor((m-1)/M_V);
    m_H=rem((m-1),M_H);
    %position filling
    pR(1,m)=d_VR*m_V;
    pR(2,m)=m_H*d_HR;
end

%pR filling
%for m=1:M_V*M_H
    %Vertical and Horizontal indexes
    %m_V=floor(m/M_H)+1;
    %m_H=rem(m,M_H)*M_H;
    %I_MV_2 = (m_V >= 0) & (m_V <= M_V/2);
    %I_MH_2 = (m_H >= 0) & (m_H <= M_H/2);
    %position filling
    %pR(1,m)=(floor(M_V/2)-m_V+I_MV_2)*d_VR/2;
    %pR(2,m)=(m_H-floor(M_H/2)+I_MH_2)*d_HR/2;
%end

%pK filling
for k=1:K
    
    pK(1,k)= x_min + (x_max - x_min) * rand;
    pK(2,k)= y_min + (y_max - y_min) * rand;
    pK(3,k)=-h_BS;
    
end


end