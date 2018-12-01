% Perform a simulation of a quadcopter
% open loop:
% 1. with the fixed angular velocities   -  you need to comment 73-80,
% remains only input function
% 2. closed loop:
% simulate(controller('pid'), 0, 3, 0.01) - start, time, step
% the resulting structure with all the parameters
% data =
%
%         x: [3xN double]
%     theta: [3xN double]
%       vel: [3xN double]
%    angvel: [3xN double]
%         t: [1xN double]
%     input: [4xN double]
%        dt: 0.0050
function result = simulate(controller, tstart, tend, dt)

fprintf('Initializing figures...\n')
h_fig = figure;
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]'); title('3D Path of the quadrotor');
set(gcf,'Renderer','OpenGL')

    % Physical constants taken from the paper by P.Corke
    g = 9.81; 
    m = 0.5;  % Hummingbird quadrotor
    L = 0.25;
    k = 3e-6; % dim. const.
    b = 1e-7; % drag coeff.
    I = diag([5e-3, 5e-3, 10e-3]);

    % Simulation times in seconds with the step 0.01
    if nargin < 4
        tstart = 0;
        tend = 3;
        dt = 0.01;
    end
    ts = tstart:dt:tend;

    % Number of points in the simulation.
    N = numel(ts);

    % intialization of the model
    % Output values, recorded as the simulation runs.
    xout = zeros(3, N);
    xdotout = zeros(3, N);
    thetaout = zeros(3, N);
    thetadotout = zeros(3, N);
    inputout = zeros(4, N);

    % Struct given to the controller. ans - for user use
    controller_params = struct('dt', dt, 'I', I, 'k', k, 'L', L, 'b', b, 'm', m, 'g', g);

    % Initial system state [x,y,z]
    x = [0; 0; 30];
    xdot = zeros(3, 1); % linear velocity
    theta = zeros(3, 1); % angles

    % If we are running without a controller, do not disturb the system.
    %if nargin == 0
        %thetadot = zeros(3, 1);
    %else
        % With a control, give a random deviation in the angular velocity.
        % Deviation is in degrees/sec.
        deviation = 200;
        thetadot = deg2rad(2 * deviation * rand(3, 1) - deviation); %angular velocity
    %end
    
    %initial input to draw the position
    x0 = [x xdot theta thetadot];
    x0= reshape(x0,[1,12]);
    x0(:,13)=0;
    x0=x0';
    
    ind = 0;

    for t = ts
        ind = ind + 1;
         % Initialize quad plot
         %
        if ind == 1
            QP = QuadPlot_my(1, x0, 0.1, 0.04, [0, 0.4470, 0.7410], N, h_3d);
        end    
        % Get input from built-in input or controller.
        if nargin == 0 % to simunlate without a controller
            i = input(t);
        else
            [i, controller_params] = controller(controller_params, thetadot); % simulation with the controller
        end
        % Compute forces, torques, and accelerations.
        omega = thetadot2omega(thetadot, theta); % omega in body frame
        a = acceleration(i, theta, xdot, m, g, k); 
        omegadot = angular_acceleration(i, omega, I, L, b, k);
        % Advance system state.
        % Euler integration method: dt - intehration interval time
        
        omega = omega + dt * omegadot;
        thetadot = omega2thetadot(omega, theta); 
        theta = theta + dt * thetadot;
        xdot = xdot + dt * a;
        x = x + dt * xdot;
        % Store simulation state for output.
        xout(:, ind) = x;  xdotout(:, ind) = xdot;  thetaout(:, ind) = theta;
        thetadotout(:, ind) = thetadot; inputout(:, ind) = i;      
          x_res = [x xdot theta thetadot]; x_res= reshape(x_res,[1,12]);  x_res(:,13)=0; x_res = x_res';
         QP.UpdateQuadPlot(x_res, dt);
    end

    % Put all simulation variables into an output struct.
    result = struct('x', xout, 'theta', thetaout, 'vel', xdotout, ...
                    'angvel', thetadotout, 't', ts, 'dt', dt, 'input', inputout);
% if we cancell animation this works to visualize the path remained                
% plot3(xout(1,:),xout(2,:), xout(3,:));grid on;title('3D Path of the quadrotor');
% hold on;plot3(xout(1,1),xout(2,1),xout(3,1),'g*')
% xlabel('x'); ylabel('y'); zlabel('z');
% grid on;figure;
% plot(ts,thetadotout(1,:));title('angvel velocity in body frame - yaw fi');
% xlabel('Time [s]'); ylabel('[rad/s]');
% grid on;figure;
% plot(ts,thetadotout(2,:));title('angvel velocity in body frame - pitch theta');
% xlabel('Time [s]'); ylabel('[rad/s]');
% grid on;figure;
% plot(ts,thetadotout(3,:));title('angvel velocity in body frame- roll ksi');
% xlabel('Time [s]'); ylabel('[rad/s]');


end

% Some input.
function in = input(t)
    in = zeros(4, 1);
    in(:) = 500;
    in(1) = in(1) + 150;
    in(3) = in(3) + 150;
    in = in .^ 2;
end

% Compute thrust given current inputs and thrust coefficient.
function T = thrust(inputs, k)
    T = [0; 0; k * sum(inputs)];
end

% Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
function tau = torques(inputs, L, b, k)
    tau = [
        L * k * (inputs(1) - inputs(3))
        L * k * (inputs(2) - inputs(4))
        b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
    ];
end

% Compute acceleration in inertial reference frame
% Parameters:
%   g: gravity acceleration
%   m: mass of quadcopter
%   k: thrust coefficient

% We neglected the drag forces because of it's small value
function a = acceleration(inputs, angles, vels, m, g, k)
    gravity = [0; 0; -g];
    R = rotation(angles);
    T = R * thrust(inputs, k);
    a = gravity + 1 / m * T;
end

% Compute angular acceleration in body frame
% Parameters:
%   I: inertia matrix
function omegad = angular_acceleration(inputs, omega, I, L, b, k)
    tau = torques(inputs, L, b, k);
    omegad = inv(I) * (tau - cross(omega, I * omega));
end

% Convert derivatives of roll, pitch, yaw to omega.
%Calcolation of the angular velocity  in the body frame
function omega = thetadot2omega(thetadot, angles)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];
    omega = W * thetadot;
end

% Convert omega to roll, pitch, yaw derivatives
function thetadot = omega2thetadot(omega, angles)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];
    thetadot = inv(W) * omega;
end
