
addpath('C:\Users\srivats\Documents\MATLAB\casadi-v3.5.1')
import casadi.*


%% Data
T = 0.1; % sampling time [s]
N = 200; % prediction horizon
L = 2.7;

v_max=10; v_min=-0.5*v_max;  
delta_max= 1.4; delta_min= -delta_max;
%% Problem Setup
x = SX.sym('x'); y = SX.sym('y'); psi = SX.sym('psi');
states = [x;y;psi]; n_states = length(states);

v = SX.sym('v'); delta = SX.sym('delta');
controls = [v;delta]; n_controls = length(controls);

%% f(x,u)
rhs = [v*cos(psi);v*sin(psi);(v/L)*tan(delta)]; 
f = Function('f',{states,controls},{rhs}); 

%% MPC setup
U = SX.sym('U',n_controls,N); 
P = SX.sym('P',n_states + n_states);
X = SX.sym('X',n_states,(N+1));
X(:,1) = P(1:3); % initial state
%{
Symbolically generating the state predictions for the next
N timesteps using euler-fwd. 
%} 
for k = 1:N
    st = X(:,k);  con = U(:,k);
    f_value  = f(st,con);
    st_next  = st+ (T*f_value);
    X(:,k+1) = st_next;
end
ff=Function('ff',{U,P},{X});

obj = 0; % Objective function
g = [];  % constraints vector

%% Weighting matrices and objectives over horizon
Q = zeros(3,3); Q(1,1) = 1;Q(2,2) = 5;Q(3,3) = 0.1; 
R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.05;

for k=1:N
    st = X(:,k);  con = U(:,k);
    obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; 
end

% compute constraints
for k = 1:N+1   % box constraints due to the map margins
    g = [g ; X(1,k)];   %state x
    g = [g ; X(2,k)];   %state y
end

%% NonLinear Problem and Solver
OPT_variables = reshape(U,2*N,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

%% System constraints

args = struct;

args.lbg = -5;  % lower bound of the states x and y
args.ubg = 25;   % upper bound of the states x and y 

% input constraints
args.lbx(1:2:2*N-1,1) = v_min; args.lbx(2:2:2*N,1)   = delta_min;
args.ubx(1:2:2*N-1,1) = v_max; args.ubx(2:2:2*N,1)   = delta_max;

%% Simulation

t0 = 0;
x0 = [0 ; 0 ; 0];    % initial condition.


xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);  % two control inputs 

sim_tim = 100; % Maximum simulation time

%% MPC Loop
mpciter = 0;
xx1 = [];
u_cl=[];

main_loop = tic;
while(norm((x0-xs),2) > 9e-2 && mpciter < sim_tim / T)
    args.p   = [x0;xs]; % set the values of the parameters vector
    args.x0 = reshape(u0',2*N,1); % initial value of the optimization variables
    %tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
    %toc
    u = reshape(full(sol.x)',2,N)';
    ff_value = ff(u',args.p); % compute OPTIMAL solution TRAJECTORY
    xx1(:,1:3,mpciter+1)= full(ff_value)';
    
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    [t0, x0, u0] = shift(T, t0, x0, u,f); % get the initialization of the next optimization step
    
    xx(:,mpciter+2) = x0;  
    mpciter
    mpciter = mpciter + 1;
end
main_loop_time = toc(main_loop);
ss_error = norm((x0-xs),2)
average_mpc_time = main_loop_time/(mpciter+1)

if xs(3)==pi/2 || xs(3)==-pi/2
   plot_car2(xx,xx1,xs,N,L,t,u_cl);
else
   plot_car (xx,xx1,xs,N,L,t,u_cl);
end