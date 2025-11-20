function P_A_RIS=phase_array_response_RIS(pR,pU,pk,R_rot,freq)

if isempty(pk)==1

   % Number of RIS elements
    M=size(pR,2);

    % setting x_u,y_u and z_u
    x_U=pU(1);y_U=pU(2);z_U=pU(3);

    % Wavelength
    lambda=3*10^8/freq;
    
    %Euler rotation matrix XYZ
    Rot_Mat_XYZ = R_rot;

    % Wave-vector S calculation
    S = [...
    -2*(pi/lambda)*x_U/sqrt(x_U^2 + y_U^2 + z_U^2), ...
    -2*(pi/lambda)*y_U/sqrt(x_U^2 + y_U^2 + z_U^2), ...
    -2*(pi/lambda)*z_U/sqrt(x_U^2 + y_U^2 + z_U^2)];

    %Initializarion of the Array response of the RIS
    P_A_RIS = []; % Initialize as empty
    
    for m = 1:M
        temp = (S * (Rot_Mat_XYZ * (pR(:,m) - pR(:,1))));
        P_A_RIS = [P_A_RIS; temp]; % Append the new value as a new row
    end

else% if pk is non-empty
    
    % Number of RIS elements
    M=size(pR,2);

    % setting x_u,y_u and z_u
    x_U=pU(1);y_U=pU(2);z_U=pU(3);

    %coordonates of RK=-OR+OK
    x_RK=pk(1)-x_U;y_RK=pk(2)-y_U;z_RK=pk(3)-z_U;
    
    % Wavelength
    lambda=3*10^8/freq;

    %Initializarion of the Array response of the RIS


    % Wave-vector S calculation
    
    S = [...
    2*(pi/lambda)*x_RK/sqrt(x_RK^2 + y_RK^2 + z_RK^2), ...
    2*(pi/lambda)*y_RK/sqrt(x_RK^2 + y_RK^2 + z_RK^2), ...
    2*(pi/lambda)*z_RK/sqrt(x_RK^2 + y_RK^2 + z_RK^2)];
    
    
    %Euler rotation matrix XYZ
    Rot_Mat_XYZ = R_rot;


    P_A_RIS = []; % Initialize as empty
    
    for m = 1:M
        temp = (S*(Rot_Mat_XYZ*(pR(:,m)-pR(:,1))));
        P_A_RIS = [P_A_RIS; temp]; % Append the new value as a new row
    end

end



end