function Rates=Rates_No_Complex_phase(pA,pR,pU,pK,R,theta,f_phases,Power,Bandwidth,freq,beta_B,beta_R)

M=size(pR,2);% Number of RIS elements
N=size(pA,2);% Number of Antenna elements
K=size(pK,2);% Number of users

%Gamma Initialization
Rates=[];
G_antenna=10^((8) / 10);
sigma_2= 10^(( - 100-30) / 10);

Rot_Mat_XYZ = R;
for k=1:K
    Mod_gk=(sqrt(beta_R)/norm(pU'-pK(:,k)));
    Mod_H=(sqrt(beta_B)/norm(pU));
    Mod_f=(1/sqrt(N));
    Ck=Power(k)*G_antenna*Mod_f/sigma_2;
    Cos_comp=0;
    Sin_comp=0;

    for j=1:N
        for i=1:M
            % Wavelength
            lambda=3*10^8/freq;
            
            %S_H_AoD
            x_U=pU(1);y_U=pU(2);z_U=pU(3);
            SH_D=[];SH_D=[SH_D, 2*(pi/lambda)*x_U/(sqrt(x_U^2+y_U^2+z_U^2))];
            SH_D=[SH_D, 2*(pi/lambda)*y_U/(sqrt(x_U^2+y_U^2+z_U^2))];SH_D=[SH_D, 2*(pi/lambda)*z_U/(sqrt(x_U^2+y_U^2+z_U^2))];
            %S_H_AoA
            SH_A=-SH_D;
            %arg_H_ij
            arg_H_ij=-2*pi*norm(pU)/lambda+SH_A*Rot_Mat_XYZ*(pR(:,i)-pR(:,1))-SH_D*pA(:,j);
            
            % S_G_AoD
            x_RK=pK(1,k)-x_U;y_RK=pK(2,k)-y_U;z_RK=pK(3,k)-z_U;
            SG_D=[]; SG_D=[SG_D, 2*(pi/lambda)*x_RK/(sqrt(x_RK^2+y_RK^2+z_RK^2))];
            SG_D=[SG_D, 2*(pi/lambda)*y_RK/(sqrt(x_RK^2+y_RK^2+z_RK^2))];SG_D=[SG_D, 2*(pi/lambda)*z_RK/(sqrt(x_RK^2+y_RK^2+z_RK^2))];
            %arg_gk_i
            arg_gk_i=-2*pi*norm(pK(:,k)-pU')/lambda-SG_D*Rot_Mat_XYZ*(pR(:,i)-pR(:,1));
            
            %%
            ang_f_j=f_phases(j);

            %Phi_ij_k
            Phi_ij_k=arg_H_ij+arg_gk_i+ang_f_j;
            

            Cos_comp= Cos_comp+Mod_gk*Mod_H*cos(theta(i)+Phi_ij_k);
            Sin_comp= Sin_comp+Mod_gk*Mod_H*sin(theta(i)+Phi_ij_k);
        
        end
    end

    Gamma_k=Ck*(Cos_comp^2+Sin_comp^2);
    F = compute_aperture_gain_from_rotation(pU, pK(:,k), Rot_Mat_XYZ);
  
    Gamma_k=F*Gamma_k;

    Rates=[Rates, Bandwidth(k)*log(1+Gamma_k)/log(2)];
    
end

end

