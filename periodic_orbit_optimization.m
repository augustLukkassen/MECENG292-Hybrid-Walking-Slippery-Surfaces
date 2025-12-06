%% Init
nx = 11 ; nu = 2 ; na = 4 ; nb = 4 ; 

horizon = 10 ; 
dt = 0.01 ; 
N = horizon / dt ; 

x0 = [0 ; 0.9239; 2.2253; 3.0107; 0.5236; 0.8653; 0.3584; -1.0957; -2.3078; 2.0323] ;
alpha_0 = [pi/6; 0; 0; 0] ; 
beta_0 = zeros(4,1) ; 
z0 = [x0; alpha_0; beta_0] ; 

x_T = zeros(1, N*nx) ; 
u_T = zeros(1, N*nu) ; 
z0_T = [x_T;
      u_T;
      n_a_b] ; 

z0_T(1, 1:N*(nx+nu+na+nb)) = z0 ; 


%% Optimization

options = optimset('display','iter','MaxFunEvals',20000,'MaxIter',20000,'diffmaxchange',1.1*1e-5, ...
    'diffminchange',1e-5);

[z_star, J_star] = fmincon(@(z) objective(z,N,nx,nu,n_a_b), ...
                           z0, [], [], [], [], LB, UB, ...
                           @(z) constraints(z,N,nx,nu,n_a_b,params), ...
                           options); 

%% Simulation & Animation 
[t, x] = simulate_three_link_walker(z_star) ;

animateThreeLink(t,x)

%% Plot 

