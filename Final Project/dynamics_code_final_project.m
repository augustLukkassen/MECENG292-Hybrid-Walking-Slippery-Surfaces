% ME193B/293B Feedback Control of Legged Robots
% UC Berkeley

clear
%% Define symbolic variables for cofiguration variables and mechanical parameters

syms q1 q2 q3 x y real
syms dq1 dq2 dq3 dx dy real
syms u1 u2 real

% Position Variable vector
q = [x;y;q1;q2;q3];

% Velocity variable vector
dq = [dx;dy;dq1;dq2;dq3];

% State vector
s = [q;dq];

% Inputs
u = [u1;u2];

% parameters
          
lL = 1;
lT = 0.5;
mL = 5;
mT = 10;
JL = 0;
JT = 0;
mH = 15;
g = 9.81;

% # of Degrees of freedom
NDof = length(q);


%% Problem 1: Lagrangian Dynamics
%Find the CoM position of each link

% Torso
pComTorso = [x + lT*sin(q3);...
             y + lT*cos(q3)];

% Leg 1
pComLeg1 = [x - lL/2*cos(deg2rad(270) - (q1 + q3));...
            y - lL/2*sin(deg2rad(270) - (q1 + q3))];

% Leg 2
pComLeg2 = [x + lL/2*cos(q2 + q3 - deg2rad(90));...
            y - lL/2*sin(q2 + q3 - deg2rad(90))];


% Leg 1
pLeg1 = [x - lL*cos(deg2rad(270) - (q1 + q3));...
            y - lL*sin(deg2rad(270) - (q1 + q3))];

% Leg 2
pLeg2 = [x + lL*cos(q2 + q3 - deg2rad(90));...
            y - lL*sin(q2 + q3 - deg2rad(90))];

% Find the CoM velocity of each link

% Torso
dpComTorso = simplify(jacobian(pComTorso, q)*dq);

% Leg 1
dpComLeg1 = simplify(jacobian(pComLeg1, q)*dq);

% Leg 2
dpComLeg2 = simplify(jacobian(pComLeg2, q)*dq);


% Find absolute angular velocity associated with each link:
% Torso
dq3Absolute = dq3;
% Leg 1
dq1Absolute = dq3 + dq1;
% Leg 2
dq2Absolute = dq3 + dq2;


% Total Kinetic energy = Sum of kinetic energy of each link
% Torso
KETorso = 0.5*mT*dpComTorso(1)^2 + 0.5*mT*dpComTorso(2)^2 + 0.5*JT*dq3Absolute^2;

% Leg 1
KELeg1 = 0.5*mL*dpComLeg1(1)^2 + 0.5*mL*dpComLeg1(2)^2 + 0.5*JL*dq1Absolute^2;

% Leg 2
KELeg2 = 0.5*mL*dpComLeg2(1)^2 + 0.5*mL*dpComLeg2(2)^2 + 0.5*JL*dq2Absolute^2;

% Hip
KEHip = 0.5*mH*dx^2 + 0.5*mH*dy^2;

% Total KE
KE = simplify(KETorso + KELeg1 + KELeg2 + KEHip);


% Total potential energy = Sum of Potential energy of each link
% Torso
PETorso = mT*g*pComTorso(2);

%Leg 1
PELeg1 = mL*g*pComLeg1(2);

% Leg 2
PELeg2 = mL*g*pComLeg2(2);

% Hip
PEHip = mH*g*y;

% Total PE
PE = simplify(PETorso + PELeg1 + PELeg2 + PEHip);


% Lagrangian
L = KE - PE;


% Equations of Motion
% Actuated variables
qActuated = [q1;q2];

% D, C, G, and B matrices
[D, C, G, B] = LagrangianDynamics(KE, PE, q, dq, qActuated);


%%  Dynamics of Systems with Constraints
%Compute the Ground reaction Forces

% Compute the position of the stance foot (Leg 1) 
mu = 0.8; %define coefficient of friction
pst = [x - lL*cos(deg2rad(270) - (q1 + q3));...
       y - lL*sin(deg2rad(270) - (q1 + q3))];


% Compute the jacobian of the stance foot
JSt = jacobian(pst, q);


% Compute the time derivative of the Jacobian
dJSt = simplify(  jacobian(JSt*dq, q)  ) ;

% Compute the Stance Force
% [D   -JST';   [d2q ;     [-C*dq - G + B*u;
%  JSt   0]   *  FSt]   =   -dJSt * dq] ;
A = [D   -JSt';
     JSt  zeros(2)] ;
b = [-C*dq - G + B*u;
     -dJSt * dq] ;
d2q_FSt = A\b ;
FSt = simplify(  d2q_FSt(NDof+1:end)  ) ;

% Account for Friction
% Compute both sticking and slipping solutions symbolically
% The runtime code will decide which to use based on friction conditions
% split rows

Jx = JSt(1,:); Jz = JSt(2,:);
Jdotx = dJSt(1,:); Jdotz = dJSt(2,:);

% helper
H = C*dq + G;

% Sticking solve (assuming no slip)
A_stick = [D  -JSt'; JSt zeros(2)];
b_stick = [-C*dq - G + B*u; -dJSt*dq];
sol_stick = A_stick\b_stick;
lambda_stick = sol_stick(NDof+1:end);    % [lambda_x; lambda_z]
FSt_stick = lambda_stick;
ddq_stick = sol_stick(1:NDof);

% Slipping solve (with friction constraint)
% tangential slip velocity
vt = Jx * dq;                       % xdot_s

% For symbolic: sign function will be converted to piecewise by matlabFunction
% At runtime, this will be evaluated numerically
% Note: sign(0) = 0, which is handled correctly
signvt = sign(vt);

% Build scalar Schur term S = Jz * (D \ (Jz' - sign*mu*Jx'))
S = Jz * (D \ (Jz' - signvt*mu*Jx'));

% RHS for lambda_z: Jz * D^{-1} * (H - B*u) - Jdotz*dq
rhs = Jz * (D \ (H - B*u)) - Jdotz * dq;

% Compute slipping solution (S will be checked numerically at runtime)
% For symbolic generation, compute the expression (runtime will handle S=0 case)
lambda_z_slip = rhs / S;
lambda_x_slip = -signvt * mu * lambda_z_slip;
FSt_slip = [lambda_x_slip; lambda_z_slip];

% Compute accelerations for slipping case
ddq_slip = D \ ( -C*dq - G + B*u + Jx'*lambda_x_slip + Jz'*lambda_z_slip );

% Default: use sticking solution (runtime code will select based on friction)
% Export both solutions separately so runtime can choose
FSt = FSt_stick;  % Default export (runtime will use FSt_stick or FSt_slip based on condition)
ddq = ddq_stick;  % Default export

% Guard function to determine if sticking solution is valid
% This checks: |lambda_x| <= mu * lambda_z
% Positive value means sticking is valid, negative means slipping
guard_friction_stick = mu * lambda_stick(2) - abs(lambda_stick(1));



%% Impact Map

% Compute the swing leg position (leg 2)
pSw = [x + lL*cos(q2 + q3 - deg2rad(90));...
       y - lL*sin(q2 + q3 - deg2rad(90))];

JSw = jacobian(pSw, q);

% Split swing foot Jacobian into tangential (x) and normal (z) components
JSw_x = JSw(1,:);  % tangential (horizontal) component
JSw_z = JSw(2,:);  % normal (vertical) component

% Swing foot velocities at impact (pre-impact)
vSw_x = JSw_x * dq;  % tangential velocity of swing foot
vSw_z = JSw_z * dq;  % normal velocity of swing foot

% Contact/height quantities for domain constraints (stick/slip guards)
lambda_x = FSt(1);          % tangential contact force
lambda_z = FSt(2);          % normal contact force
zs      = pst(2);           % stance foot height
dzs     = JSt(2,:)*dq;      % stance foot vertical velocity
vxs     = JSt(1,:)*dq;      % stance foot tangential velocity
zns     = pSw(2);           % swing (non-stance) foot height

% Stick domain: zs = dzs = vxs = 0, zns >= 0, |lambda_x| <= mu*lambda_z, lambda_z >= 0
eq_stick   = [zs; dzs; vxs];
ineq_stick = [zns; mu*lambda_z - abs(lambda_x); lambda_z];

% Slip domain: zs = dzs = vxs = 0, zns >= 0, lambda_z >= 0 (vxs guard can be relaxed if you allow sliding)
eq_slip   = [zs; dzs; vxs];
ineq_slip = [zns; lambda_z];

%% Hybrid Impact Dynamics with Stick-Slip Transitions
% Following the paper: Dynamic Walking on Slippery Surfaces (arXiv:1812.04600)
% The impact map accounts for friction-bounded inelastic impacts
%
% Hybrid System Structure:
% - Domains: Stick domain (no slip) and Slip domain (sliding)
% - Impact Events: Sticking impact (friction sufficient) and Slipping impact (friction insufficient)
% - Guards: Contact detection (z_sw = 0) and friction conditions
% - Reset Maps: Impact maps that compute post-impact velocities based on friction

% First, compute the sticking impact solution (assuming no slip)
% This enforces both normal and tangential velocity constraints
A_stick = [D, -JSw';
           JSw, zeros(2)];
b_stick = [D*dq;
           zeros(2,1)];  % Both v_x and v_z set to zero after impact

postImpact_stick = A_stick \ b_stick;
dqPlus_stick = simplify(postImpact_stick(1:NDof));
Fimpact_stick = simplify(postImpact_stick(NDof+1:NDof+2));

% Check Coulomb friction condition for sticking impact
% Sticking requires: |F_impact_x| <= mu * F_impact_z
Fimpact_stick_x = Fimpact_stick(1);
Fimpact_stick_z = Fimpact_stick(2);

% For slipping impact, we enforce:
% 1. Normal velocity constraint: v_z^+ = 0
% 2. Friction constraint: F_x = -sign(v_x^-) * mu * F_z
% 3. Tangential velocity may be non-zero after impact

% For symbolic: sign function will be converted to piecewise by matlabFunction
% At runtime, this will be evaluated numerically
% Note: The regularization (eps_sign) should be applied in runtime code, not here
sign_vSw_x = sign(vSw_x);

% Slipping impact: solve with friction constraint
% The system becomes:
% D * dq^+ = D * dq^- + JSw_x' * F_x + JSw_z' * F_z
% JSw_z * dq^+ = 0  (normal velocity constraint)
% F_x = -sign(v_x^-) * mu * F_z  (Coulomb friction)

% Substitute F_x = -sign(v_x^-) * mu * F_z into the momentum equation
% D * dq^+ = D * dq^- + (JSw_z' - sign(v_x^-) * mu * JSw_x') * F_z
% JSw_z * dq^+ = 0

% Solve for F_z first using the Schur complement approach
% From: JSw_z * D^{-1} * (D*dq^- + (JSw_z' - sign*mu*JSw_x')*F_z) = 0
% Rearranging: JSw_z * D^{-1} * (JSw_z' - sign*mu*JSw_x') * F_z = -JSw_z * dq^-

S_slip = JSw_z * (D \ (JSw_z' - sign_vSw_x * mu * JSw_x'));
rhs_slip = -JSw_z * dq;

% Compute slipping impact solution symbolically
% Note: Runtime code should check if abs(S_slip) < 1e-9 and fallback to sticking
% For symbolic generation, compute the expression directly
Fimpact_slip_z = rhs_slip / S_slip;
Fimpact_slip_x = -sign_vSw_x * mu * Fimpact_slip_z;
Fimpact_slip = [Fimpact_slip_x; Fimpact_slip_z];

% Compute post-impact velocities for slipping case
dqPlus_slip = D \ (D*dq + JSw_x'*Fimpact_slip_x + JSw_z'*Fimpact_slip_z);
dqPlus_slip = simplify(dqPlus_slip);

% Impact guard: determine which impact type occurs
% Check if sticking impact satisfies friction constraint
% If |F_impact_stick_x| <= mu * F_impact_stick_z, use sticking impact
% Otherwise, use slipping impact

% Note: In practice, this decision is made at runtime based on the computed forces
% For symbolic generation, we provide both solutions

% Default impact map (will be selected based on friction condition at runtime)
% For now, export both versions - the actual selection happens in simulation
dqPlus = dqPlus_stick;  % Default to sticking (can be overridden)
Fimpact = Fimpact_stick;  % Default to sticking (can be overridden)

% Impact guard condition (for determining which impact map to use)
impact_guard_stick = mu * Fimpact_stick_z - abs(Fimpact_stick_x);  % >= 0 means sticking

%% Impact Event Guards
% Guard for impact event: swing foot touches ground
% Impact occurs when: z_sw = 0 and dz_sw < 0 (approaching ground)
impact_guard_contact = pSw(2);  % z_sw = 0
impact_guard_velocity = JSw_z * dq;  % dz_sw < 0 (checked numerically)

%% Domain Transition Guards (Continuous Phase)
% Guard for transition from stick to slip domain
% Transition occurs when: |lambda_x| > mu * lambda_z
guard_stick_to_slip = mu * lambda_z - abs(lambda_x);  % < 0 means transition to slip

% Guard for transition from slip to stick domain  
% Transition occurs when: v_x = 0 (tangential velocity becomes zero)
guard_slip_to_stick = vxs;  % = 0 means transition to stick

%% Summary of Hybrid System Components
% Complete hybrid system now includes:
% 1. Continuous dynamics: fvec, gvec (with stick-slip contact forces)
% 2. Domains: Stick domain (eq_stick, ineq_stick) and Slip domain (eq_slip, ineq_slip)
% 3. Impact maps: Sticking impact (dqPlus_stick, Fimpact_stick) and Slipping impact (dqPlus_slip, Fimpact_slip)
% 4. Guards: Impact guards (contact, velocity, friction) and domain transition guards
% 5. Reset maps: Impact maps + relabelling matrix R (defined below)
% 
% The system can transition between:
% - Stick domain <-> Slip domain (continuous phase transitions)
% - Any domain -> Impact event -> New domain (discrete transitions)
% - After impact, legs are relabelled using matrix R


%% Other functions

% swing foot velocity
dpSw = JSw*dq;

%% Export functions
if ~exist('./gen')
    mkdir('./gen')
end
addpath('./gen')

matlabFunction(FSt,      'File', 'gen/Fst_gen', 'Vars', {s, u});
matlabFunction(FSt_stick, 'File', 'gen/Fst_stick_gen', 'Vars', {s, u});
matlabFunction(FSt_slip,  'File', 'gen/Fst_slip_gen', 'Vars', {s, u});
matlabFunction(ddq_stick, 'File', 'gen/ddq_stick_gen', 'Vars', {s, u});
matlabFunction(ddq_slip,  'File', 'gen/ddq_slip_gen', 'Vars', {s, u});
matlabFunction(dqPlus,   'File', 'gen/dqPlus_gen', 'Vars', {s});
matlabFunction(dqPlus_stick, 'File', 'gen/dqPlus_stick_gen', 'Vars', {s});
matlabFunction(dqPlus_slip,  'File', 'gen/dqPlus_slip_gen', 'Vars', {s});
matlabFunction(Fimpact,  'File', 'gen/Fimpact_gen', 'Vars', {s});
matlabFunction(Fimpact_stick, 'File', 'gen/Fimpact_stick_gen', 'Vars', {s});
matlabFunction(Fimpact_slip,  'File', 'gen/Fimpact_slip_gen', 'Vars', {s});
matlabFunction(impact_guard_stick, 'File', 'gen/impact_guard_stick_gen', 'Vars', {s});
matlabFunction(impact_guard_contact, 'File', 'gen/impact_guard_contact_gen', 'Vars', {s});
matlabFunction(impact_guard_velocity, 'File', 'gen/impact_guard_velocity_gen', 'Vars', {s});
matlabFunction(guard_friction_stick, 'File', 'gen/guard_friction_stick_gen', 'Vars', {s, u});
matlabFunction(guard_stick_to_slip, 'File', 'gen/guard_stick_to_slip_gen', 'Vars', {s, u});
matlabFunction(guard_slip_to_stick, 'File', 'gen/guard_slip_to_stick_gen', 'Vars', {s});
matlabFunction(vSw_x,    'File', 'gen/vSw_x_gen', 'Vars', {s});
matlabFunction(vSw_z,    'File', 'gen/vSw_z_gen', 'Vars', {s});
matlabFunction(pSw,      'File', 'gen/pSw_gen', 'Vars', {s});
matlabFunction(dpSw,     'File', 'gen/dpSw_gen', 'Vars', {s});
matlabFunction(pst,      'File', 'gen/pSt_gen', 'Vars', {s});
matlabFunction(eq_stick, 'File', 'gen/eq_stick_gen', 'Vars', {s});
matlabFunction(ineq_stick,'File','gen/ineq_stick_gen','Vars',{s,u});
matlabFunction(eq_slip,  'File', 'gen/eq_slip_gen',  'Vars', {s});
matlabFunction(ineq_slip,'File','gen/ineq_slip_gen','Vars',{s,u});
matlabFunction(pComLeg1, 'File', 'gen/pComLeg1_gen', 'Vars', {s});
matlabFunction(pComLeg2, 'File', 'gen/pComLeg2_gen', 'Vars', {s});
matlabFunction(pComTorso,'File', 'gen/pComTorso_gen', 'Vars', {s});
matlabFunction(pLeg1,    'File', 'gen/pLeg1_gen', 'Vars', {s});
matlabFunction(pLeg2,    'File', 'gen/pLeg2_gen', 'Vars', {s});




%% [Part 1a] Compute the f and g vectors
% Split FSt into 2 components: 1. which depends on u and 2. which does not depend on u
% Note: FSt is linear in u

Fst_u = jacobian(FSt, u) ; % FSt = Fst_u*u + (Fst - Fst_u*u)
Fst_u = simplify(Fst_u) ;

Fst_nu = FSt - Fst_u*u ; % Fst_nu = (Fst - Fst_u*u)
Fst_nu = simplify(Fst_nu) ;


% Compute vector fields
fvec = [dq ;
        inv(D)*(-C*dq - G + JSt'*Fst_nu)];
fvec = simplify(fvec);

gvec = [zeros(NDof, length(u));
          inv(D)*(B + JSt'*Fst_u)];
gvec = simplify(gvec);

% Write out files
matlabFunction(fvec, 'File', 'gen/fvec_gen', 'Vars', {s});
matlabFunction(gvec, 'File', 'gen/gvec_gen', 'Vars', {s});


%% Change of Coordinates
% Transformation matrix:
T = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 1;
     0 0 0 1 1;
     0 0 0 0 1];
d = [0;
     0;
     -pi;
     -pi;
     0];

%% [Part 1b] Output dynamics
% Create absolute generalized coordinates
q_abs = T*q + d;

% Create symbolic variable needed for defining output
syms th3d real

% Define Output
y = [q_abs(2+3) - th3d;
     q_abs(2+2) + q_abs(2+1)];

% write out file
matlabFunction(y, 'File', 'gen/y_gen', 'Vars', {s, th3d});





%% [Part 1c] Lie Derivatives
% Define Lie Derivatives
Lfy = jacobian(y, s)*fvec;
Lf2y = jacobian(Lfy, s)*fvec;
LgLfy = jacobian(Lfy, s)*gvec;

% Write out files
matlabFunction(Lfy, 'File', 'gen/Lfy_gen', 'Vars', {s});
matlabFunction(Lf2y, 'File', 'gen/Lf2y_gen', 'Vars', {s});
matlabFunction(LgLfy, 'File', 'gen/LgLfy_gen', 'Vars', {s});






%% [Part 1d] Relabelling Matrix
params.R = [1, 0, 0, 0, 0;
            0, 1, 0, 0, 0;
            0, 0, 0, 1, 0;
            0, 0, 1, 0, 0;
            0, 0, 0, 0, 1];