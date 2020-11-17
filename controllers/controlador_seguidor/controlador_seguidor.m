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
MAX_SPEED = 4;
MAX_SENSOR_NUMBER = 8;
WHEEL_RADIUS = (195 / 2000.0);
DISTANCE_FROM_CENTER = (381 / 2000.0);
MAX_SENSOR_VALUE = 1024;
range = MAX_SENSOR_VALUE / 2;
max_speed = 5.24;
SPEED_UNIT = max_speed / 1024;
MIN_DISTANCE = 0.5;
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

%vector de posici�n
iOs=[ -0.140208, 0.114, -0.0610447; %so0
      -0.122255,  0.114, -0.110288; %so1
      -0.0801434, 0.114, -0.145492; %so2
      -0.0285304, 0.114, -0.16417;  %so3
       0.026383,  0.114, -0.16417;  %so4
       0.0780296, 0.114, -0.145492; %so5
       0.120142,  0.114, -0.110288; %so6
       0.138094,  0.114, -0.0610446];%so7

angulo=[3.14159;2.44346;2.0944;1.74533;1.39626;1.0472;0.698132;0];%angulo de rot de so0 a so7
sensor_values = zeros(1, MAX_SENSOR_NUMBER);

ui=zeros(2,MAX_SENSOR_NUMBER);

%PID
g=zeros(2,1);
g_1=zeros(2,1);
 
xi = 0;
zi = 0;


% PID orientaciï¿½n
kpO = 5;
kiO = 0.0001;
kdO = 0;
EO = 0;
eO_1 = 0;

alpha = 0.5;

while wb_robot_step(TIME_STEP) ~= -1
    
    %Lectura de posicion angular
    north = wb_compass_get_values(orientation_sensor);
    theta = atan2(north(1, 1), north(1, 3));
    
    wheel_weight_total = zeros(1, 2);
    %Lectura de todos los sensores delanteros
    for k = 1:MAX_SENSOR_NUMBER
        sensor_values(k) = wb_distance_sensor_get_value(sonar(k));
    end
    %calculo de la distancia
    distance =  5 * (1.0 - (sensor_values / MAX_SENSOR_VALUE));
    
    for k = 3:6
        if distance(k) < MIN_DISTANCE
            controlador = 2;
        end
    end
    
    
    if controlador == 2
        IRB = [cos(theta)  -sin(theta);
               sin(theta)   cos(theta)];
        
        M_lo=[1     0;
              0  (1/lo)];
        
          for n = 3:6
          
              wij=0.015*sinh(15*distance(n)-2)/distance(n);%funcion de energia
              
              %wij=(distance(n)-dij)/distance(n);
              
              iRs=[ cos(angulo(n))   0   sin(angulo(n));
                                 0   1                0;
                   -sin(angulo(n))   0   cos(angulo(n))];%matriz de rotación
              
              iTs=[iRs,iOs(n,:)';0,0,0,1];%matriz de transforamción a centro del robot
              
              sxj=[distance(n);0;0;1];%vector respecto a marco de ref Si
              
              ixj_hat=iTs*sxj;%xj-xi
              
              ixj=[ixj_hat(3,1);ixj_hat(1,1)];%seleccion de coordenada x y z
              
              ui(:,n)=wij*ixj;%velocidades de ecuacion de consenso
              %duda con este signo
              x = ixj_hat(3,1); 
              z = ixj_hat(1,1);
          end
          
        %vel=M_lo*inv(IRB)*(sum(ui,2));
        vel=(sum(ui,2));
        
        g=g_1 + vel*TIME_STEP*1^(-3);
        g_1=g;
        
        xg=vel(1,1);
        zg=vel(2,1);
        
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
        
        %formatSpec = 'xg: %.2f zg: %.2f  xi: %.2f zi: %.2f\n';
        %fprintf(formatSpec, xg, zg, xi, zi);
        
        % Asignaciï¿½n de controladores a velocidad de llantas
        left_speed = (v - w*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
        right_speed = (v + w*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
        speed = [left_speed, right_speed];
        
        formatSpec = 'v: %.2f w: %.2f  left_speed: %.2f right_speed: %.2f distancia: %.2f\n';
        wb_console_print(sprintf(formatSpec, v, w, left_speed, right_speed,distance),WB_STDOUT)

        
    elseif controlador == 3
        left_speed = 0;
        right_speed = 0;
        speed = [left_speed, right_speed];        
    end
    

   for k = 1:2
       if speed(k) < -max_speed
           speed(k) = -max_speed;
        elseif speed(k) > max_speed
           speed(k) = max_speed;
         end
    end

    
    wb_motor_set_velocity(left_wheel,speed(1, 1));
    wb_motor_set_velocity(right_wheel, speed(1, 2));
    
    drawnow;
    
end

% cleanup code goes here: write data to files, etc.
