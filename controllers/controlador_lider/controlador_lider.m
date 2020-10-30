%% MT 3006 Robï¿½tica 2
% Jacqueline Guarcax
% Alejandro Windevoxhel
% Proyecto 4: Redes lï¿½der-seguidor en Webots

%% Activar debugging
% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
desktop;
%keyboard;

%% Parï¿½metros de Webots
TIME_STEP = 64;
MAX_SPEED = 2;
MAX_SENSOR_NUMBER = 16;
WHEEL_RADIUS = (195 / 2000.0);
DISTANCE_FROM_CENTER = (381 / 2000.0);
MAX_SENSOR_VALUE = 1024;
range = MAX_SENSOR_VALUE / 2;
max_speed = 5.24;
SPEED_UNIT = max_speed / 1024;
MIN_DISTANCE = 0.11;
WHEEL_WEIGHT_THRESHOLD = 100;

%% Webots - Enable Devices

% Device IDs
time_step  = wb_robot_get_basic_time_step();
left_wheel  = wb_robot_get_device('left wheel');
right_wheel = wb_robot_get_device('right wheel');

position_sensor = wb_robot_get_device('gps');
orientation_sensor = wb_robot_get_device('compass');

% Set up wheels
wb_motor_set_position(left_wheel, Inf);
wb_motor_set_position(right_wheel, Inf);
wb_motor_set_velocity(left_wheel, 0.0);
wb_motor_set_velocity(right_wheel, 0.0);

% Enables GPS and compass
wb_gps_enable(position_sensor, TIME_STEP);
wb_compass_enable(orientation_sensor, TIME_STEP);

% get and enable all distance sensors
sonar = zeros(MAX_SENSOR_NUMBER, 1);
for k = 1:MAX_SENSOR_NUMBER
    sonar(k) = wb_robot_get_device(strcat('so', num2str(k - 1)));
    wb_distance_sensor_enable(sonar(k), time_step);
end

sensor_values = zeros(1, MAX_SENSOR_NUMBER);
%% Variables de PID
controlador = 1;
xi = 0;
zi = 0;

% PID posiciï¿½n
kpP = 10;%10 funciona bien para acercamiento exp
kiP = 0.0001;
kdP = 0;
EP = 0;
eP_1 = 0;

% PID orientaciï¿½n
kpO = 2;
kiO = 0.0001;
kdO = 0;
EO = 0;
eO_1 = 0;

alpha = 1.8;
% Trayectoria
n=0;

% main loop:
while wb_robot_step(TIME_STEP) ~= -1
    n=n+1;
    
    %Definiciï¿½n de trayectoria a seguir
    xg = 4*cos(0.001*2*pi*n);
    zg = 4*sin(0.001*2*pi*n);
    
    %Lectura de todos los sensores de distancia
    for k = 1:MAX_SENSOR_NUMBER
        sensor_values(k) = wb_distance_sensor_get_value(sonar(k));
    end
    %Lectura de posiciï¿½n angular
    north = wb_compass_get_values(orientation_sensor);
    rad = atan2(north(1, 1), north(1, 3));
    %Asignaciï¿½n de valores
    pos = wb_gps_get_values(position_sensor);
    xi = pos(1, 1);
    zi = pos(1, 3);
    
    if controlador == 1
        x = xi; z = zi; theta = rad;
        e = [xg - x; zg - z];
        thetag = atan2(e(2), e(1));
        
        eP = norm(e);
        eO = thetag - theta;
        eO = atan2(sin(eO), cos(eO));
        
        % Control de velocidad lineal
        keP = MAX_SPEED * (1-exp(-alpha*eP^2)) / eP;
        v = keP*eP;
        
        % Control de velocidad angular
        eO_D = eO - eO_1;
        EO = EO + eO;
        w = kpO*eO + kiO*EO + kdO*eO_D;
        eO_1 = eO;
        
        formatSpec = 'xg: %.2f zg: %.2f  xi: %.2f zi: %.2f\n';
        fprintf(formatSpec, xg, zg, xi, zi);
        % Asignaciï¿½n de controladores a velocidad de llantas
        left_speed = (v - w*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
        right_speed = (v + w*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
    
    end
    
    % send actuator commands, e.g.:
    wb_motor_set_velocity(left_wheel, left_speed);
    wb_motor_set_velocity(right_wheel, right_speed);
    
    % if your code plots some graphics, it needs to flushed like this:
    drawnow;
    
end

% cleanup code goes here: write data to files, etc.
