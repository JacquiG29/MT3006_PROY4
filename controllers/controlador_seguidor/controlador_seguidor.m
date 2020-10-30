%% MT 3006 Rob�tica 2
% Jacqueline Guarcax
% Alejandro Windevoxhel
% Proyecto 4: Redes l�der-seguidor en Webots

%% Activar debugging
% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
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


% get and enable all distance sensors
sonar = zeros(MAX_SENSOR_NUMBER, 1);
for k = 1:MAX_SENSOR_NUMBER
    sonar(k) = wb_robot_get_device(strcat('so', num2str(k - 1)));
    wb_distance_sensor_enable(sonar(k), time_step);
end
% get and enable devices, e.g.:
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%  motor = wb_robot_get_device('motor');

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%

%Variables para braitenberg
speed = zeros(1,2);
state = "f";
controlador = 2;
while wb_robot_step(TIME_STEP) ~= -1
  wheel_weight_total = zeros(1, 2);
  %  Lectura de todos los sensores
  for k = 1:MAX_SENSOR_NUMBER
      sensor_values(k) = wb_distance_sensor_get_value(sonar(k));
  end
  
      bin_sens_val = sensor_values ~= zeros(size(sensor_values));
    bin_sens_val(9:end) = zeros(1, 8);
    distance =  5 * (1.0 - (sensor_values / MAX_SENSOR_VALUE));
    bin_dist_val =  distance < MIN_DISTANCE;
    speed_modifier = bin_sens_val.*bin_dist_val.*(1 - (distance / MIN_DISTANCE));
    wheel_weight_total = wheel_weight_total + sum(speed_modifier'.*braitenberg_matrix);
  
  for k = 1:MAX_SENSOR_NUMBER  
    if distance(k) < MIN_DISTANCE
        controlador = 2;
    end
  end
  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
      if controlador == 2
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
        
    end
    
    
  wb_motor_set_velocity(left_wheel, left_speed);
  wb_motor_set_velocity(right_wheel, right_speed);
  
  drawnow;

end

% cleanup code goes here: write data to files, etc.
