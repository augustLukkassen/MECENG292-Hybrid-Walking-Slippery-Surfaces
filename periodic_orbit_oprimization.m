%% Optimization init/seed 
x0 = [0 ; 0.9239; 2.2253; 3.0107; 0.5236; 0.8653; 0.3584; -1.0957; -2.3078; 2.0323] ;

alpha_0 = [pi/6; 0; 0; 0] ; 
beta_0 = zeros(4,1) ; 
z0 = [x0; alpha_0; beta_0] ;  

params = struct();
params.mu = 0.8;
params.Tmax = 2;
params.v_stick_tol = 1e-2;
params.max_segments = 30;


%% Linear Constraints 
Aineq = []; Bineq = []; Aeq = []; Beq = []; LB = [] ; UB = [] ; 
LB = -inf(1,18)' ; 
LB(1:2) = 0 ; 
LB(3:5) = -2*pi ; 
UB = inf(1,18)' ;
UB(1) = 1 ; 
UB(3:5) = 2*pi ; 

%% Optimization
options = optimset('display','iter','MaxFunEvals',12000,'MaxIter',2000,'diffmaxchange',1.1*1e-5, ...
    'diffminchange',1e-5);
options = optimset(options,'MaxIter',25,'MaxFunEvals',800);

[z_star, cost_optimal] = fmincon(@periodic_orbit_obj, z0, Aineq, Bineq, ...
    Aeq, Beq, LB, UB , @constraints, options, params) ;

%% Simulate one orbit
params1 = params;
params1.alpha = z_star(11:14);
params1.beta  = z_star(15:18);
[t, x, x_plus] = simulate_single_orbit(z_star, params1);
disp('Finished simulate_single_orbit');
