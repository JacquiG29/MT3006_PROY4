%% MT 3006 Rob�tica 2
% Jacqueline Guarcax
% Alejandro Windevoxhel
% Proyecto 4: Redes l�der-seguidor en Webots

%% Activar debugging
% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
desktop;
%keyboard;

%% Par�metros de Webots
TIME_STEP = 64;
MAX_SPEED = 5.24;
MAX_SENSOR_NUMBER = 16;
WHEEL_RADIUS = (195 / 2000.0);
DISTANCE_FROM_CENTER = (381 / 2000.0);
MAX_SENSOR_VALUE = 1024;
range = MAX_SENSOR_VALUE / 2;
max_speed = 5.24;
SPEED_UNIT = max_speed / 1024;
MIN_DISTANCE = 2;
WHEEL_WEIGHT_THRESHOLD = 100;

%Matriz de pesos de sensores de braitenberg
braitenberg_matrix = [
    150 0;
    200, 0;
    300, 0;
    600, 0;
    0, 600;
    0, 300;
    0, 200;
    0, 150;
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0 ];

%% Webots - Enable Devices
% Device IDs
time_step  = wb_robot_get_basic_time_step();
left_wheel  = wb_robot_get_device('left wheel');
right_wheel = wb_robot_get_device('right wheel');

% Set up wheels
wb_motor_set_position(left_wheel, Inf);
wb_motor_set_position(right_wheel, Inf);
wb_motor_set_velocity(left_wheel, 0.0);
wb_motor_set_velocity(right_wheel, 0.0);

orientation_sensor = wb_robot_get_device('compass');
wb_compass_enable(orientation_sensor, TIME_STEP);

% get and enable all distance sensors
sonar = zeros(MAX_SENSOR_NUMBER, 1);
for k = 1:MAX_SENSOR_NUMBER
    sonar(k) = wb_robot_get_device(strcat('so', num2str(k - 1)));
    wb_distance_sensor_enable(sonar(k), time_step);
end


%Variables para braitenberg
speed = zeros(1,2);
state = "f";
controlador = 3;

%Distancia al nuevo punto cercano
lo=0.001;

vel=[0;0];

while wb_robot_step(TIME_STEP) ~= -1
    
    %Lectura de posiciï¿½n angular
    north = wb_compass_get_values(orientation_sensor);
    theta = atan2(north(1, 1), north(1, 3));
    
    wheel_weight_total = zeros(1, 2);
    %  Lectura de todos los sensores
    for k = 1:MAX_SENSOR_NUMBER
        sensor_values(k) = wb_distance_sensor_get_value(sonar(k));
    end
    %distance =  5 * (1.0 - (sensor_values / MAX_SENSOR_VALUE));
        
    %bin_sens_val = sensor_values ~= zeros(size(sensor_values));
    %bin_sens_val(9:end) = zeros(1, 8);
    %distance =  5 * (1.0 - (sensor_values / MAX_SENSOR_VALUE));
    %bin_dist_val =  distance < MIN_DISTANCE;
    %speed_modifier = bin_sens_val.*bin_dist_val.*(1 - (distance / MIN_DISTANCE));
    %wheel_weight_total = wheel_weight_total + sum(speed_modifier'.*braitenberg_matrix);
    
    %for k = 1:MAX_SENSOR_NUMBER
    %    if distance(k) < MIN_DISTANCE
    %        controlador = 2;
    %    end
    %end
     
     distance =  5 * (1.0 - (sensor_values(5) / MAX_SENSOR_VALUE));
     
     if distance <= 1
       controlador = 2;
     end 
    

     if controlador == 2
        IRB = [cos(theta)  -sin(theta);
               sin(theta)   cos(theta)];
        
        M_lo=[1     0;
              0  1/lo];
        
        %translation 0.026383 0.114 -0.16417
        %rotation 0 1 0 1.39626
        
        iOs=[0.026383; 0.114; -0.16417];%vector de posición
        angulo=-0.16417;%ángulo de rotación del sensor
        iRs=[ cos(angulo)   0   sin(angulo);
                        0   1             0;
             -sin(angulo)   0   cos(angulo)];%matriz de rotación
        
        iTs=[iRs,iOs;0,0,0,1];%matriz de transforamción a centro del robot
        
        sxj=[distance;0;0;1];%vector respecto a marco de ref Si
        
        ixj_hat=iTs*sxj;%xj-xi
        
        ixj=[ixj_hat(1,1);ixj_hat(3,1)];%selección de coordenada x y z
        
        ui=distance*ixj;%velocidades de ecuación de consenso
        
        vel=M_lo*inv(IRB)*ui;
        
        v=vel(1,1);
        w=vel(2,1);
        
        % Asignaciï¿½n de controladores a velocidad de llantas
        left_speed = (v - w*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
        right_speed = (v + w*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
        
        formatSpec = 'v: %.2f w: %.2f  left_speed: %.2f right_speed: %.2f distancia: %.2f\n';
        fprintf(formatSpec, v, w, left_speed, right_speed,distance);
    
    elseif controlador == 1
        %speed_modifier = 1 - (sensor_values/range);
        %speed = speed + SPEED_UNIT*(speed_modifier)*braitenberg_matrix;
        [speed, state] = braitenberg(state, wheel_weight_total, speed, WHEEL_WEIGHT_THRESHOLD, MAX_SPEED);
        for k = 1:2
            if speed(k) < -max_speed
                speed(k) = -max_speed;
            elseif speed(k) > max_speed
                speed(k) = max_speed;
            end
        end
        left_speed = speed(1, 1);
        right_speed = speed(1, 2);
    
    elseif controlador == 3
        left_speed = 0;
        right_speed = 0;
        
    end
    
    
    wb_motor_set_velocity(left_wheel, left_speed);
    wb_motor_set_velocity(right_wheel, right_speed);
    
    drawnow;
    
end

% cleanup code goes here: write data to files, etc.
