% File: model_UAV_Cross.m
%------------------------------------------%
% UAV in "×" configuration for MatMPC
% States: [p; q; v; ω] ∈ ℝ^13, controls: rotor speeds Ω_i ∈ ℝ⁴
% Objective: track desired position, quaternion and velocity via refs

% Add the path to the functions
addpath(genpath('Functions'));

%------------------------------------------%
%% Communication variables
K_User = 5;
N_A = 3; % Antenna elements
M_size = [4,4];


M_ele = M_size(1) * M_size(2); % RIS elements

freq = 2 * 10^9; % GHz
beta_dB = -30;
beta_B = 10^(beta_dB / 10);
beta_R = 10^(beta_dB / 10);

R_min=2*10^6;


%% Dimensions
nx   = 13+K_User;   % p(3), q(4), v(3), ω(3),sv1,...,svKUser
nu   = 4+K_User;    % thrust and torques Omega1,...,Omega4,sv1_dot,...,svKUser_dot
nz   = 0;
ny   = 6+K_User+3+1;    % outputs: p(3), v(3),omega(3), sv1,...,svKUser, omega(3), geosedic_dist(1)
nyN  = 6+K_User+3+1;    % terminal outputs: p(3), v(3), sv1,...,svKUser, omega(3), geosedic_dist(1)
np   = 0;    % no online parameters
nc   = K_User+1; % K QoS contraints+1 velocity constraints
ncN  = K_User+1; % K QoS contraints+1 velocity constraints
nbu  = 4;
nbu_idx = 1:4;
nbx=3+K_User;
nbx_idx = [1:3,14:13+K_User];



%% create variables
addpath('/home/abdoul/Desktop/Matlab_Sims/ARIS_Traj_Opt/ARIS_Main/Casadi');
import casadi.*


states   = SX.sym('states',nx,1);
controls = SX.sym('controls',nu,1);
alg      = SX.sym('alg',nz,1);
params   = SX.sym('paras',np,1);    % parameters
% no params
refs     = SX.sym('refs',ny,1);
refN     = SX.sym('refsN',nyN,1);
Q        = SX.sym('Q',ny,1);
QN       = SX.sym('QN',nyN,1);
aux      = SX.sym('aux',ny,1);
auxN     = SX.sym('auxN',nyN,1);


%% UAV constants
m   = 1.042;
g   = 9.81;
d   = 0.23;
I   = diag([0.015,0.015,0.070]);

bf = 5.95*1e-4;     % [N/Hz^2]moment constant
bm = 1* 1e-5;        % [m]thrust constant
Ez  = [0;0;1];


%% split states and control
p     = states(1:3);% position
q   = states(4:7);% quaternion
v     = states(8:10);% velocity
omega = states(11:13);% angular velocity

sv=states(14:13+K_User);% slack variable for Quality of service softening
sv_dot=controls(5:4+K_User);% derivative of slack variable for Quality of service softening


Omega  = controls(1:4); % Speed of rotors


% Define matrix B
B = [ bf,        bf,         bf,         bf;
     -bf*d/sqrt(2), -bf*d/sqrt(2),  bf*d/sqrt(2),  bf*d/sqrt(2);
     -bf*d/sqrt(2),  bf*d/sqrt(2),  bf*d/sqrt(2), -bf*d/sqrt(2);
     -bm,     bm,   -bm,    bm ];

% Define squared motor speeds [Omega1^2; Omega2^2; Omega3^2; Omega4^2]
%Omega_squared = Omega.^2;  
Omega_squared = Omega;  


% Compute [u_T; T] = B * Omega_squared
u_G = B * Omega_squared;

ut=u_G(1);
tau=u_G(2:4);

%% rotation 
% Quaternion components

q=q/(norm(q)+1e-13);
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

% Explicit rotation matrix R from body frame to world frame
R = [ 1-2*(q2^2+q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
      2*(q1*q2 + q0*q3), 1-2*(q1^2+q3^2),     2*(q2*q3 - q0*q1);
      2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1),   1-2*(q1^2+q2^2) ];

wx = omega(1);
wy = omega(2);
wz = omega(3);



%% dynamics
p_dot     = v;
% Explicit quaternion multiplication dq = 0.5*q ∘ [0; ω]
q_dot = 0.5 * [
    -q1*wx - q2*wy - q3*wz;
     q0*wx + q2*wz - q3*wy;
     q0*wy - q1*wz + q3*wx;
     q0*wz + q1*wy - q2*wx
];
v_dot     = -g*Ez + (ut/m)*R*Ez;
omega_dot = I \ (-cross(omega,I*omega) + tau);

x_dot = [p_dot; q_dot; v_dot; omega_dot; sv_dot];

%% implicit form
xdot = SX.sym('xdot',nx,1);
impl_f = xdot - x_dot;


z_fun = [];


%% Communication part

% pU: position of tha UAV
pU = [states(1), states(2), states(3)]; % UAV position


% Set positions
[pA, pR, pK] = Setup_env(N_A, K_User, M_size, freq);
%load('pK.mat');

% Initialize theta
theta = [];

% Power and Bandwidth
Power = 0.2*ones(1, K_User);
Bandwidth = 10^6 * ones(1, K_User);

% Beamforming vector
f_phases = array_response_phases_BS(pA,pU,freq);


%Phase_shift optimization
p_bar_User=(1/K_User)*(sum(pK,2))';

P_A_rx_RIS=phase_array_response_RIS(pR,pU,[],R,freq);
P_A_tx_RIS=phase_array_response_RIS(pR,pU,p_bar_User,R,freq);

for m_idx=1:M_ele

    theta=[theta; P_A_tx_RIS(m_idx)-P_A_rx_RIS(m_idx)];

end

%Rates
Rates=Rates_No_Complex_phase(pA,pR,pU,pK,R,theta,f_phases,Power,Bandwidth,freq,beta_B,beta_R);

%R_Global=sum(Rates);

%% outputs & costs


q_horizontal=[1;0;0;0];
geosedic_distance = quatGeodesicDistance(q, q_horizontal);

h  = [p; v; sv; omega; geosedic_distance];
hN = h;

Q_mat=diag(Q);
QN_mat=diag(QN);


% Outer objectives. 
obji = 0.5 * ( (h - refs)' * diag(Q) * (h - refs)); % The objective function

% Objective function at the terminal stage
objN = 0.5 * ( (hN - refN)' * diag(QN) * (hN - refN));

obji_GGN = 0.5 * ( (aux - refs)' * (aux - refs) );
objN_GGN = 0.5 * ( (auxN - refN)' * (auxN - refN) );

% obji      = 0.5*(h(1:6+K_User+3)-refs(1:6+K_User+3))'  * Q_mat(1:6+K_User+3,1:6+K_User+3)  * (h(1:6+K_User+3)-refs(1:6+K_User+3)) + Q(end) *quaternion_geodesic_distance(q, refs(6+K_User+3+1:end));
% objN      = 0.5*(hN(1:6+K_User+3)-refN(1:6+K_User+3))' * QN_mat(1:6+K_User+3,1:6+K_User+3) * (hN(1:6+K_User+3)-refN(1:6+K_User+3)) + QN(end) *quaternion_geodesic_distance(q, refN(6+K_User+3+1:end));
% 
% 
% obji_GGN  =0.5* (aux(1:6+K_User+3)  - refs(1:6+K_User+3))'  * (aux(1:6+K_User+3)  - refs(1:6+K_User+3))  + Q(end)*quaternion_geodesic_distance(q, refs(6+K_User+3+1:end)); 
% objN_GGN  = 0.5 * (auxN(1:6+K_User+3) - refN(1:6+K_User+3))'  * (auxN(1:6+K_User+3) - refN(1:6+K_User+3)) +  QN(end)*quaternion_geodesic_distance(q, refN(6+K_User+3+1:end));
% 


%% Constraints : soft communication constraints


norm_v = sqrt((v(1)+1e-9)^2 + (v(2)+1e-9)^2 + (v(3)+1e-9)^2);


Rates = Rates(:);
sv    = sv(:);

general_con   = [Rates + sv.^2 - R_min*ones(K_User,1);
                 norm_v];

general_con_N = general_con;

% general_con   = [Rates'+sv.^2 -R_min*ones(K_User,1), norm_v];
% general_con_N = [Rates'+sv.^2 -R_min*ones(K_User,1), norm_v ];




%% discretization
Ts_st = 0.01;


%% Functions 

function d = quatGeodesicDistance(q1, q2)
% Geodesic distance between two UNIT quaternions q1 and q2
% q = [w x y z]' (scalar-first convention)
% Assumes q1 and q2 are already normalized

    % Conjugate of q2
    q2_conj = [q2(1); -q2(2:4)];

    % Quaternion multiplication: q = q1 * q2*
    q = quatMultiply(q1, q2_conj);

    % Numerical clamp for safety
    w = min(max(q(1), -1), 1);

    % Geodesic distance = rotation angle between them
    angle = acos(w);
    d = 2 * angle;
end

%-------------------------------------
% Quaternion multiplication
%-------------------------------------
function q = quatMultiply(q1, q2)
    w1 = q1(1); v1 = q1(2:4);
    w2 = q2(1); v2 = q2(2:4);

    w = w1*w2 - dot(v1, v2);
    v = w1*v2 + w2*v1 + cross(v1, v2);

    q = [w; v];
end
