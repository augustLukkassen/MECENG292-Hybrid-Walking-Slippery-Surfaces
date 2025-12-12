%% Init
addpath('./gen')
nx = 10 ; nu = 2 ; nF = 2; na = 4 ; nb = 4 ; 

horizon = 0.9 ; 
dt = 0.05 ; 
% Ensure N is an integer grid count (avoid floating rounding issues)
N = round(horizon / dt) ; 

x0 = [0 ; 0.9239; 2.2253; 3.0107; 0.5236; 0.8653; 0.3584; -1.0957; -2.3078; 2.0323] ;

% Build a 10xN state initial guess (repeat the 10x1 state across time)
z0_x = repmat(x0, 1, N) ; 
% Seed controls with small nonzero torques to give cost a gradient
z0_u = 0.05 * ones(nu, N) ; 
z0_F = zeros(nF, N) ; 

z0_alpha = zeros(na, 1);
z0_beta = zeros(nb, 1);

% Decision Vector 
z0 = [z0_x(:) ;
      z0_u(:) ; 
      %z0_F(:) ;
      z0_alpha ; 
      z0_beta ] ;

% Bounds 
LB = -Inf(size(z0));
UB =  Inf(size(z0));

% Init Parameters
params.N1 = round(0.3*N) ; 
params.N2 = round(0.7*N) ; 
params.mu = 0.8 ; 
R = eye(5) ; 
R(3:4,3:4) = [0 1; 1 0] ;
params.relabel = R ;
% Soften output dynamics to ease feasibility search
params.eps = 0.02; 

%% Optimization
options = optimset('display','iter','MaxFunEvals',12000,'MaxIter',2000,'diffmaxchange',1.1*1e-5, ...
    'diffminchange',1e-5);

[z_star, J_star] = fmincon(@(z) objective(z, nx, nu, N), ...
                           z0, [], [], [], [], LB, UB, ...
                           @(z) constraints(z,N,nx,nu,nF,na,nb,dt,params), ...
                           options); 

% Extract Control Parameters
idx_X_end     = nx * N;
idx_U_end     = idx_X_end  + nu * N;
idx_alpha_end = idx_U_end  + na;
idx_beta_end  = idx_alpha_end + nb;

params.alpha = z_star(idx_U_end + 1 : idx_alpha_end);
params.beta  = z_star(idx_alpha_end + 1 : idx_beta_end);

%% Simulation & Animation 
[t, x] = simulate_gait(z_star, params) ;

% animateThreeLink(t,x)

%% Plot 
