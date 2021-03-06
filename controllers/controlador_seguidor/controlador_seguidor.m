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
MAX_SENSOR_NUMBER = 8;
WHEEL_RADIUS = (195 / 2000.0);
DISTANCE_FROM_CENTER = (381 / 2000.0);
MAX_SENSOR_VALUE = 1024;
range = MAX_SENSOR_VALUE / 2;
max_speed = 5.24;
SPEED_UNIT = max_speed / 1024;
MIN_DISTANCE = 1;
WHEEL_WEIGHT_THRESHOLD = 100;

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
controlador = 1;

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
ixj = zeros(2,MAX_SENSOR_NUMBER);
ui=zeros(2,MAX_SENSOR_NUMBER);
%vi = [0; 0];
%PID
g=zeros(2,1);
g_1=zeros(2,1);

xi = 0;
zi = 0;


% PID orientaciï¿½n
kpO = 10;
kiO = 0.0001;
kdO = 0;
EO = 0;
eO_1 = 0;

alpha = 0.5;
distancia_vec=zeros(MAX_SENSOR_NUMBER,1);
while wb_robot_step(TIME_STEP) ~= -1
    
    %Lectura de posicion angular
    north = wb_compass_get_values(orientation_sensor);
    theta = atan2(north(1, 1), north(1, 3));
    
    if (theta < 0.0)
        theta = theta + (2*pi);
    end
    
    wheel_weight_total = zeros(1, 2);
    %Lectura de todos los sensores delanteros
    for k = 1:MAX_SENSOR_NUMBER
        sensor_values(k) = wb_distance_sensor_get_value(sonar(k));
    end
    distance = 5*(1 - (sensor_values/1024)); %valores de sensores pasan a distancias, en m.
    [~, ind_xi_xj] = sort(distance); %ordenar distancias de menor a mayor, obtener índices
    dist_values_in_range = distance ((distance  <= 3));%&(distance>DISTANCE_FROM_CENTER)
    
    
%     A=find(sensor_values);
%     
%     
%     for i = 1:numel(A)
%         dist_val=A;
%         distance(i) = 5.0 * (1 - (sensor_values(dist_val(i)) / 1024));
%     end
    
    %NECESITO QUE SI EL SENSOR NO LEA NADA NO ME AFECTE A LA DISTANCIA
    %EXCLUIR VALORES PARA LOS CUALES SENSOR VALUE ES 0 O LA DISTANCIA ES IGUAL
    %A 5
    %distance=distancia_vec;
    
    if (isempty(dist_values_in_range))
        controlador = 1;
    else
        controlador = 2;
    end
    
   
    if controlador==1
        left_speed = 0;
        right_speed = 0;
        speed = [left_speed, right_speed];
        
    elseif controlador==2
        IRB = [cos(theta)  -sin(theta);
            sin(theta)   cos(theta)];
        
        M_lo=[1     0;
            0  (1/lo)];
        
        for n = 1:MAX_SENSOR_NUMBER            
            iRs=[ cos(angulo(n))   0   sin(angulo(n));
                0   1                0;
                -sin(angulo(n))   0   cos(angulo(n))];%matriz de rotación
            
            iTs=[iRs,iOs(n,:)';0,0,0,1];%matriz de transforamción a centro del robot
            
            sxj=[distance(n);0;0;1];%vector respecto a marco de ref Si
            
            ixj_hat=iTs*sxj;%xj-xi
            
            ixj(:,n)=[-ixj_hat(3,1);-ixj_hat(1,1)];%seleccion de coordenada x y z
            
%             if distance(n) == 5
%                 ixj(:,n) = [0 0]';
%                 distance(n) = 0;
%             end
        end
        %WHEEL_RADIUS
        %tomar el valor del sensor que detecte el objeto más cercano.
        ind_xi_xj = ind_xi_xj(1);
        norm_xi_xj = distance(ind_xi_xj);
        
        neighbors = numel(norm_xi_xj);
        for n = 1:numel(neighbors)
            dij=1;
            wij=(distance(n)-dij)/distance(n);
            ui=wij*ixj;%velocidades de ecuacion de consenso
            %duda con este signo
        end
        %vel=M_lo*inv(IRB)*(sum(ui,2));
        vel=(sum(ui,2));
        %vel=ui;
        g=vel.*TIME_STEP*1^(-3);
        
        xg=vel(1,1);
        zg=vel(2,1);
        
        x = -ixj(1,1);
        z = -ixj(2,1);
        
        e = [x;z];
        thetag = atan2(e(2), e(1));
        
        eP = norm(e);
        eO = thetag;
        eO = atan2(sin(eO), cos(eO));
        
        % Control de velocidad lineal
        keP = MAX_SPEED * (1-exp(-alpha*eP^2)) / eP;
        v = keP*eP;
         
        if (eP < 0.08)
            v = 0.2;
        end
        
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
        
        formatSpec = 'left_speed: %.2f right_speed: %.2f distancia: %.2f\n';
        wb_console_print(sprintf(formatSpec,left_speed, right_speed,distance),WB_STDOUT)
        
    end
    
    
    wb_console_print(sprintf('Sensor: %.0f\n',sensor_values),WB_STDOUT)
    
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
