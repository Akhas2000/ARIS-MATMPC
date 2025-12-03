
function [input, data]  = InitData_Main(settings,Ns,q_sv,q_p,q_h,q_v,q_rot,q_omega,h_UAV,pathXY)
    %% Pull dims from settings
    nx       = settings.nx;
    nu       = settings.nu;
    nz       = settings.nz;
    ny       = settings.ny;
    nyN      = settings.nyN;
    np       = settings.np;
    nc       = settings.nc;
    ncN      = settings.ncN;
    N        = settings.N;
    nbx      = settings.nbx;
    nbu      = settings.nbu;
    nbx_idx  = settings.nbx_idx;
    nbu_idx  = settings.nbu_idx;

    Ts_st=settings.Ts_st ;

    switch settings.model
        %% Tracking with soft communication constraint
        case 'GTMR_4_com_soft'
            %% Initial conditions
            v_max=17;v_min=0;data.v_min=v_min;data.v_max=v_max;
            
            Omega_min=16^2;Omega_max=100^2;data.Omega_min=Omega_min;data.Omega_max=Omega_max;%Hz^2

            K_User=settings.K_User;
            input.x0 = [ pathXY(1,1);pathXY(1,2);h_UAV;   ...  % p start
                         1;0;0;0;   ...  % q start
                         0;0;0;   ...  % v start
                         0;0;0; ...    % ω start
                         sqrt(1e+8)*ones(K_User,1) ... % sv start
                         
                         ];      
            input.u0 = [Omega_min*ones(4,1);ones(K_User,1)];      % Omega1,...,Omega4, sv_dot
            input.z0 = zeros(nz,1);
            para0   = zeros(np,1);


            %% Weights

            
            

            %q_0=[q_p;q_p;q_p; q_v;q_v;q_v;q_sv*ones(K_User,1);q_omega;q_omega;q_omega;q_rot;q_rot;q_rot;q_rot];
            q_0=[q_p;q_p;q_h; q_v;q_v;q_v;q_sv*ones(K_User,1);q_omega;q_omega;q_omega;q_rot];
            
            Q  = repmat(q_0,1,N);
            QN =    q_0;   
            data.q_0=q_0;
            %% Bounds on states (none) and controls Ω_i ≥ 0
            lb_x = [-400;-400;-200;zeros(K_User,1)];
            ub_x = [400;400;200;inf*ones(K_User,1)];
            lb_u = [Omega_min*ones(4,1)];
            ub_u = [Omega_max*ones(4,1)];
            lb_g = [zeros(K_User,1);v_min];
            ub_g = [inf*ones(K_User,1);v_max];
            lb_gN= lb_g;
            ub_gN= ub_g;




    end

    %% General path constraint bounds
    input.lb = repmat(lb_g,N,1);
    input.ub = repmat(ub_g,N,1);
    input.lb = [input.lb; lb_gN];
    input.ub = [input.ub; ub_gN];

    %% Control bounds
    lbu = -inf(nu,1);
    ubu =  inf(nu,1);
    for i = 1:nbu
        lbu(nbu_idx(i)) = lb_u(i);
        ubu(nbu_idx(i)) = ub_u(i);
    end
    input.lbu = repmat(lbu,1,N);
    input.ubu = repmat(ubu,1,N);

    %% State bounds
    lbx = -inf(nbx,1);
    ubx =  inf(nbx,1);
    for i = 1:nbx
        lbx(i) = lb_x(i);
        ubx(i) = ub_x(i);
    end
    input.lbx = repmat(lbx,1,N);
    input.ubx = repmat(ubx,1,N);

    %% Shooting arrays
    input.x  = repmat(input.x0,1,N+1);
    input.u  = repmat(input.u0,1,N);
    input.z  = repmat(input.z0,1,N);
    input.od = repmat(para0,1,N+1);

    %% Weights
    input.W  = Q;
    input.WN = QN;

    %% Multipliers
    input.lambda = zeros(nx,N+1);
    input.mu     = zeros(N*nc+ncN,1);
    input.mu_u   = zeros(N*nu,1);
    input.mu_x   = zeros(N*nbx,1);



    

    %% Reference matrix

    switch settings.model


        case 'GTMR_4_com_soft'

            data.REF        = zeros(Ns, ny);
            data.REF(:,1:2) = pathXY;%p_ref(1:2)
            data.REF(:,3) = h_UAV*ones(Ns, 1);%p_ref(3)
            data.REF(:,4:6) = zeros(Ns, 3);%v_ref
            data.REF(:,7:6+K_User) = zeros(Ns, K_User);%sv_ref
            data.REF(:,6+K_User+1:6+K_User+3) = zeros(Ns, 3);%omega_ref(1:2)
            % q_identity = [1, 0, 0, 0];     
            % data.REF(:,6+K_User+3+1:end) = repmat(q_identity, Ns, 1);   % q_ref:Ns x 4 matrix
            geosedic_ref = 0;     
            data.REF(:,6+K_User+3+1) = repmat(geosedic_ref, Ns, 1);   % eta_ref:Ns x 3 matrix
            

     

           

                                                                                 
    end
end