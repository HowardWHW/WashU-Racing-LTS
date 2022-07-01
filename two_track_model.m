clear all;
close all;
clc;

%% Constants
g = 9.81;                       % accel of gravity [m/s2]
rho = 1.225;                    % ambient air density [kg/m3]
dt = 0.001;                     % Time step of ss simulation [s]
T = 10;                         % Total period [s]
CGz = 0.271;                    % CG height [m]
wb = 1.538;                     % Wheelbase [m]
tw = [1.232, 1.206];            % Track width (front, rear) [m]
fwd = 0.479;                    % Percent weight distribution on front
lwd = 0.507;                    % Percent weight distribution on left

%% Car Parameters
m = 261.8;                      % Mass of the car & driver [kg]
[mu_long, mu_lat] = tire_model(m*g/4); ...
                                % Longitudinal tire friction coefficient 
Cl = 2.11;                      % Lift coefficient
Cd = 1.15;                      % Drag coefficient
Af = 1.0782;                    % Frontal Area [m2]
% aero_bal = 0.36;                % Front Aero balance

%% Kinematic Relations
% Two track model:
% v = v + a*dt;
% x = x + v*dt;
% 
% WT_long = m*a_long*CGz/wb;
% WT_lat = m*a_lat*CGz/tw;
% Fzf = m*g*fwd + F_downforce*aero_bal - WT_long;
% Fzr = m*g*(1-fwd) + F_downforce*(1-aero_bal) + WT_long;
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
a = [zeros(step + 1,1) ones(step + 1, 1)]; % [Long, Lat] acceleration
t = zeros(step + 1,1);

WT_long     = zeros(step + 1, 1);       % Longitudinal wt tsfr to behind
WT_lat      = [zeros(step + 1, 1) zeros(step + 1, 1)]; ...
                                        % Lateral wt tsfr to right
F_downforce = zeros(step + 1, 1);
F_drag      = zeros(step + 1, 1);

Fz          = [zeros(step + 1, 1) zeros(step + 1, 1) ...
               zeros(step + 1, 1) zeros(step + 1, 1)]; ...       
                                        % Normal force on wheels
F_friction  = [zeros(step + 1, 1) zeros(step + 1, 1) ...
               zeros(step + 1, 1) zeros(step + 1, 1)]; ...
                                        % Traction limited friction
F_engine    = [zeros(step + 1, 1) zeros(step + 1, 1) ...
               zeros(step + 1, 1) zeros(step + 1, 1)]; ...
                                        % Engine force on wheels
F_cp        = [zeros(step + 1, 1) zeros(step + 1, 1) ...
               zeros(step + 1, 1) zeros(step + 1, 1)]; ...
                                        % Contact patch force
F_net       = [zeros(step + 1, 1) zeros(step + 1, 1)]; ...
                                        % Force used in F=ma

for i = 1:step
    F_downforce(i) = 1/2*rho*Af*Cl*v(i)^2;
    F_drag(i) = 1/2*rho*Af*Cd*v(i)^2;
    if i ~= 1
        WT_long(i) = m*g*a(i-1, 1)*CGz/wb;
    end

    Ff = m*g*fwd - WT_long(i);
    Fr = m*g*(1-fwd) + WT_long(i);
    
    if i ~= 1
        WT_lat(i, 1) = Ff*a(i-1, 2)*CGz/tw(1);
        WT_lat(i, 2) = Fr*a(i-1, 2)*CGz/tw(2);
    end

    Fz(i, 1) = m*g*fwd*lwd + F_downforce(i)*aero_bal(v(i))/2 ...
        - WT_lat(i, 1);
    Fz(i, 2) = m*g*fwd*lwd + F_downforce(i)*aero_bal(v(i))/2 ...
        + WT_lat(i, 1);
    Fz(i, 3) = m*g*(1-fwd)*lwd + F_downforce(i)*(1-aero_bal(v(i)))/2 ...
        - WT_lat(i, 2);
    Fz(i, 4) = m*g*(1-fwd)*lwd + F_downforce(i)*(1-aero_bal(v(i)))/2 ...
        + WT_lat(i, 2);

%     [mu_long, mu_lat] = tire_model(m*g); ...
                                % Longitudinal tire friction coefficient 

    % TODO: BALANCE LATERAL AND LONG FRICTION
    F_friction(i, 1) = mu_long*Fz(i, 1);
    F_friction(i, 2) = mu_long*Fz(i, 2);
    F_friction(i, 3) = mu_long*Fz(i, 1);
    F_friction(i, 4) = mu_long*Fz(i, 2);
    
    F_engine(i, 3) = f_engine(v(i))/2;
    F_engine(i, 4) = f_engine(v(i))/2;
    
    F_cp(i, 1) = min(F_engine(i, 1), F_friction(i, 1));
    F_cp(i, 2) = min(F_engine(i, 2), F_friction(i, 2));
    F_cp(i, 3) = min(F_engine(i, 3), F_friction(i, 1));
    F_cp(i, 4) = min(F_engine(i, 4), F_friction(i, 2));
    
    F_net(i) = F_cp(i, 1) + F_cp(i, 2) ...
        + F_cp(i, 3) + F_cp(i, 4) - F_drag(i);
    a(i, 1) = F_net(i)/(m*g);
    
    t(i+1) = t(i) + dt;
    v(i+1) = v(i) + a(i, 1)*g*dt;
    x(i+1) = x(i) + v(i)*dt;

end

%% Plots

figure
plot(t, x, t, v, t, a(:, 1), 'LineWidth', 2);
grid on;
legend('Displacement', 'Velocity', 'Acceleration', 'location', ...
    'northwest');
xlabel('Time (s)');
ylabel('Kinematic parameter');
xlim([0 t(step)]);

figure
plot(v, F_downforce, v, F_drag, ...
    v, WT_long, v, WT_lat(:, 1), v, WT_lat(:, 2), ...
    v, Fz(:,1), v, Fz(:,2), v, Fz(:,3), v, Fz(:,4), ...
    v, F_net, 'LineWidth', 2);
xlim([0 v(step)]);
legend('Downforce', 'Drag', ...
    'Long WT Tsfr', 'Lat WT TsfrF', 'Lat WT TsfrR', ...
    'Fz FL', 'Fz FR', 'Fz RL', 'Fz RR',...
    'Net Force', 'location', 'northwest');

figure
plot(v, F_engine(:,1), v, F_engine(:,3), ...
    v, F_friction(:,1), v, F_friction(:,2), ...
    v, F_friction(:,3), v, F_friction(:,4), ...
    v, F_cp(:,1), v, F_cp(:,2), v, F_cp(:,3), v, F_cp(:,4), ...
    'LineWidth', 2);
xlim([0 v(step)]);
legend('Power front', 'Power rear', ...
    'Friction FL', 'Friction FR', 'Friction RL', 'Friction RR', ...
    'Contact Patch FL', 'Contact Patch FR', ...
    'Contact Patch RL', 'Contact Patch RR', ...
    'location', 'northwest');