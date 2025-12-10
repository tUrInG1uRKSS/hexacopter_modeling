function [TB,x0] = generate_test_case(test_case, PARAMS)

Fg = PARAMS.m * PARAMS.g;
T_hover = Fg / 6;

switch test_case
    
    case 0  % Hover
        x0 = [zeros(3,1); zeros(3,1); zeros(3,1); zeros(3,1)];
        TB = T_hover * ones(6,1);

    case 1  % Yaw rotation
        x0 = [zeros(3,1); zeros(3,1); zeros(3,1); zeros(3,1)];
        T_yaw = 0.3;
        TB = [T_hover+T_yaw; T_hover-T_yaw; 
              T_hover+T_yaw; T_hover-T_yaw;
              T_hover+T_yaw; T_hover-T_yaw];

    case 2  % Takeoff
        x0 = [zeros(3,1); zeros(3,1); zeros(3,1); zeros(3,1)];
        TB = (T_hover + 0.3)*ones(6,1);

    case 3  % Pitch forward
        x0 = [zeros(3,1); zeros(3,1); zeros(3,1); zeros(3,1)];
        T_pitch = 0.01;
        TB = ones(6,1)*T_hover/cos(pi/4);
        TB(1) = T_hover - T_pitch;
        TB(4) = T_hover + T_pitch;

    case 4  % Roll left/right
        x0 = [zeros(3,1); zeros(3,1); zeros(3,1); zeros(3,1)];
        T_roll = 0.01;
        TB = ones(6,1)*T_hover/cos(pi/4);
        TB([2,3]) = T_hover - T_roll;
        TB([5,6]) = T_hover + T_roll;

    case 5  % Gimbal lock test
        x0 = [zeros(3,1); zeros(3,1); zeros(3,1); 0; pi/2; 0];
        T_pitch = 0.01;
        TB = ones(6,1)*T_hover;
        TB(1) = T_hover - T_pitch;
        TB(4) = T_hover + T_pitch;

    case 6  % Takeoff + yaw
        x0 = [zeros(3,1); zeros(3,1); zeros(3,1); zeros(3,1)];
        T_yaw = 0.3;
        T_hover = T_hover + 0.2;
        TB = [T_hover+T_yaw; T_hover-T_yaw; 
              T_hover+T_yaw; T_hover-T_yaw;
              T_hover+T_yaw; T_hover-T_yaw];

    otherwise
        error("Undefined test case")
end
