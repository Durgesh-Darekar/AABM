clear all
clc
number_of_waypoints = 1200; %enter the number of waypoints for the circle.

%starting co-ordinates
x_start = 0.0;
y_start = 0.0;

section_angle = 360/number_of_waypoints;

diameter = 2.3;
radius = diameter/2;

for i = 1:number_of_waypoints
    angle = section_angle * i;
    
    if (0<=angle && angle<=90)
        cos_x = radius * cosd(angle);
        sin_y = radius * sind(angle);
        x_pos = x_start + (radius - cos_x);
        y_pos = y_start + sin_y;
        X{i,1} = x_pos;
        X{i,2} = y_pos; 

    elseif (angle<=180)
        ang = angle - 90;
        sin_x = radius * sind(ang);
        cos_y = radius * cosd(ang);
        x_pos = x_start + (radius + sin_x);
        y_pos = y_start + cos_y;
        X{i,1} = x_pos;
        X{i,2} = y_pos; 
    
    elseif (angle<=270)
        ang = angle - 180;
        cos_x = radius * cosd(ang);
        sin_y = radius * sind(ang);
        x_pos = x_start + (radius + cos_x);
        y_pos = y_start - sin_y;
        X{i,1} = x_pos;
        X{i,2} = y_pos; 
        
    elseif (angle<=360)
        ang = angle - 270;
        sin_x = radius * sind(ang);
        cos_y = radius * cosd(ang);
        x_pos = x_start + (radius - sin_x);
        y_pos = y_start - cos_y;
        X{i,1} = x_pos;
        X{i,2} = y_pos; 
    end
end

for i = 2:number_of_waypoints
    dif_x{i} = X{i,1} - X{i-1,1};
    dif_y{i} = X{i,2} - X{i-1,2};
    length{i} = sqrt(dif_x{i}^2 + dif_y{i}^2);
end    

%  for i = 1:30
%     xpoint[i,1] = X{i,1};
%     ypoint[i,1] = X{i,2};
%     
%  end
A = cell2mat(X);
%mata = cellfun(@(od) double(od), A);
%  for i = 1:1:30
%     plot(xpoint{i},ypoint{i});
%     hold on
%  end
%  hold off

%plot (A(:,1),A(:,2))
% plot (A)

csvwrite('circle_waypoints_2_3_1200.csv',A);


