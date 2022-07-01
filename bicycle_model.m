clear all;
close all;
clc;

%% Constants
g = 9.81;                       % accel of gravity [m/s2]
rho = 1.225;                    % ambient air density [kg/m3]
dt = 0.001;                     % Time step of ss simulation [s]
T = 10.001;                     % Total period [s]
CGz = 0.271;                    % CG height [m]
wb = 1.538;                     % Wheelbase [m]
fwd = 0.479;                    % Percent weight distribution on front

%% Car Parameters
m = 261.8;                      % Mass of the car & driver [kg]
W = m*g;                        % Weight of the car [N]
[mu_long, mu_lat] = tire_model(W/2); ...
                                % Longitudinal tire friction coefficient 

Cl = 2.11;                      % Lift coefficient
Cd = 1.15;                      % Drag coefficient
Af = 1.0782;                    % Frontal Area [m2]
% aero_bal = 0.36;                % Front Aero balance

%% Kinematic Relations
% Bicycle model:
% v = v + a*dt;
% x = x + v*dt;
% 
% WT_long = m*a_long*CGz/wb;
% Fzf = W*fwd + F_downforce*aero_bal - WT_long;
% Fzr = W*(1-fwd) + F_downforce*(1-aero_bal) + WT_long;
% F_net = m*a;
% F_friction = mu_long*Fz;
% F_downforce = 1/2*rho*Af*Cl*v^2;
% F_drag = 1/2*rho*Af*Cd*v^2;
% F_engine = f_engine(v);
%
% Calculation:
% F_net = F_friction - F_drag
% Fz = mg + F_downforce
% ma = mu_long*Fz - 1/2*rho*Af*Cd*v^2
% a = F_net/m;

step = T/dt;

x = zeros(step + 1,1);
v = zeros(step + 1,1);
a = zeros(step + 1,1); 
t = zeros(step + 1,1);

WT_long     = zeros(step + 1, 1);       % Longitudinal wt tsfr to behind

F_downforce = zeros(step + 1, 1);
F_drag      = zeros(step + 1, 1);

Fz          = [zeros(step + 1, 1) zeros(step + 1, 1)]; ...       
                                        % Normal force on wheels
F_friction  = [zeros(step + 1, 1) zeros(step + 1, 1)]; ...
                                        % Traction limited friction
F_engine    = [zeros(step + 1, 1) zeros(step + 1, 1)]; ...
                                        % Engine force on wheels
F_cp        = [zeros(step + 1, 1) zeros(step + 1, 1)]; ...
                                        % Contact patch force
F_net       = zeros(step + 1, 1);       % Force used in F=ma

for i = 1:step
    F_downforce(i) = 1/2*rho*Af*Cl*v(i)^2;
    F_drag(i) = 1/2*rho*Af*Cd*v(i)^2;
    if i ~= 1
        WT_long(i) = W*a(i-1)*CGz/wb;
    end

    Fz(i, 1) = W*fwd + F_downforce(i)*aero_bal(v(i)) - WT_long(i);
    Fz(i, 2) = W*(1-fwd) + F_downforce(i)*(1-aero_bal(v(i))) ...
        + WT_long(i);

    F_friction(i, 1) = mu_long*Fz(i, 1);
    F_friction(i, 2) = mu_long*Fz(i, 2);
    
    F_engine(i, 2) = f_engine(v(i));
    
    F_cp(i, 1) = min(F_engine(i, 1), F_friction(i, 1));
    F_cp(i, 2) = min(F_engine(i, 2), F_friction(i, 2));
    
    F_net(i) = F_cp(i, 1) + F_cp(i, 2) - F_drag(i);
    a(i) = F_net(i)/(W);
    
    t(i+1) = t(i) + dt;
    v(i+1) = v(i) + a(i)*g*dt;
    x(i+1) = x(i) + v(i)*dt;

end

%% Plots

figure
plot(t, x, t, v, t, a.*g, 'LineWidth', 2);
hold on;
yline(75);
hold off;
grid on;
legend('Displacement', 'Velocity', 'Acceleration', 'location', ...
    'northwest');
xlim([0 t(step)]);
xlabel('Time (s)');
ylabel('Kinematic parameter');

figure
plot(v, F_downforce, v, F_drag, v, WT_long, v, Fz(:,1), v, Fz(:,2), ...
    v, F_net, 'LineWidth', 2);
xlim([0 v(step)]);
legend('Downforce', 'Drag', 'Long WT Tsfr', 'Fz front', 'Fz rear', ...
    'Net Force', 'location', 'northwest');

figure
plot(v, F_engine(:,1), v, F_engine(:,2), v, F_friction(:,1), ...
    v, F_friction(:,2), v, F_cp(:,1), v, F_cp(:,2), 'LineWidth', 2);
xlim([0 v(step)]);
legend('Power front', 'Power rear', 'Friction front', 'Friction rear', ...
    'Contact Patch front', 'Contact Patch rear', 'location', 'northwest');