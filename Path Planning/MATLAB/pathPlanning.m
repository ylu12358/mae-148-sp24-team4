function GPS = pathPlanning(origin, pickup, dropoff, avoid, offset, cardyn)
% The following user defined function uses permutations and a Dubin's path
% algorithm to determine the optimal path to take between drop-off
% locations and a pickup. It will do so while also avoiding predefined
% rectangular obstacles. The function accomplishes this by taking in the
% pickup, drop-offs, and obstacles in a local coordinate frame defined by
% a user-defined origin. Once given this along with the origin in WGS84
% coordinates (lat, lon, alt), the function will return a list of
% sequential coordinates in the WGS84 frame.

%--------------------------------------------------------------------------%
%                             INPUTS GUIDE                                 %
%--------------------------------------------------------------------------%

% NOTE: the local frame should be speecified in dimensions of METERS

% 'origin'  :    a 1x3 vector containing the lat, lon, and alt of the
% origin that was used to define the local frame

% 'pickup'  :    a 1x2 vector containing the X and Y coordinates of the
% pickup location in the local frame

% 'dropoff' :    an nx2 matrix containing the X and Y coordinates of n
% drop-offs in the local frame

% 'avoid'   :    an mx4 matrix containing the X, Y, width, and height of m
% obstacles within the local frame

% 'offset'  :    a scalar value representing the distance the car should be
% kept away from obstacle boundaries in the local frame

% 'cardyn'  :    a structure array containing the minimum turning radius
% (meters), average car velocity (meters per second), and distance step
% (meters) of GPS coordinates



%--------------------------------------------------------------------------%
%                            OUTPUTS GUIDE                                 %
%--------------------------------------------------------------------------%

% NOTE: the WGS84 frame is the standard GPS frame specified by lat, lon,
% and alt
 
% 'GPS'     :    a px3 matrix containing the lat, lon, and alt of
% sequential GPS coordinates



%--------------------------------------------------------------------------%
%                         PERMUTATION LOOP                                 %
%--------------------------------------------------------------------------%

% Determine permutaion of every drop off index
prm = perms(1:length(dropoff));

% Generate lines for avoidance plotting and algorithm
avoid_lines_plot = zeros(5, 2, size(avoid, 1));
avoid_lines_main = zeros(5, 2, size(avoid, 1));
for i = 1:size(avoid, 1)

    w = avoid(i, 3)/2;
    h = avoid(i, 4)/2;
    wo = avoid(i, 3)/2 + offset;
    ho = avoid(i, 4)/2 + offset;
    x = avoid(i, 1);
    y = avoid(i, 2);
    
    bl = [x - w, y - h];
    tl = [x - w, y + h];
    tr = [x + w, y + h];
    br = [x + w, y - h];
    blo = [x - wo, y - ho];
    tlo = [x - wo, y + ho];
    tro = [x + wo, y + ho];
    bro = [x + wo, y - ho];
   
    avoid_lines_plot(:, : , i) = [bl; tl; tr; br; bl];
    avoid_lines_main(:, : , i) = [blo; tlo; tro; bro; blo];

end

% Initialize shortest var
shortest = 1e6;

% Generate the coordinate path by looping through every permutation order
% and determineing minimum total distance
for p = 1:length(prm)

    % Determine pathing index for this iteration
    path_index = prm(p, :);

    % Generate the coordinate path
    local_temp = pickup;
    c = 0;

    for i = 2:length(dropoff)+2
        if i == length(dropoff)+2
            local_temp = [local_temp; pickup];
        else
            local_temp = [local_temp; dropoff(path_index(i-1), :)];
        end
    
        % Determine direction of travel
        xs = local_temp(i + c, 1) - local_temp(i + c - 1, 1);
        ys = local_temp(i + c, 2) - local_temp(i + c - 1, 2);
    
        % Determine function of current path
        fx = @(x) (local_temp(i + c, 2) - local_temp(i + c - 1, 2))/(local_temp(i + c, 1) - local_temp(i + c - 1, 1)).*(x - local_temp(i + c - 1, 1)) + local_temp(i + c - 1, 2);
        fy = @(y) (local_temp(i + c, 1) - local_temp(i + c - 1, 1))/(local_temp(i + c, 2) - local_temp(i + c - 1, 2)).*(y - local_temp(i + c - 1, 2)) + local_temp(i + c - 1, 1);
    
        % Sort the obstacles in order from closest to farthest on current
        % pathway
        dist_dropoff_avoid = pdist2(local_temp(i + c - 1, :), avoid(:, 1:2));
        dist_dropoff_avoid_main = dist_dropoff_avoid;
    
        % Sort through the obstacles
        avoid_lines = zeros(size(avoid_lines_main));
        for m = 1:size(avoid, 1)
    
            [~, sort] = min(dist_dropoff_avoid);
            dist_dropoff_avoid_main(m) = dist_dropoff_avoid(sort);
            dist_dropoff_avoid(sort) = NaN;
            avoid_lines(:, :, m) = avoid_lines_main(:, :, sort);
    
        end
        dist_current = pdist2(local_temp(i + c - 1, :), local_temp(i + c, :));
    
        % Determine intersection points for each obstacle
        for j = 1:size(avoid, 1)
    
            % Determine case based on direction of travel
    
            % If current path moves right and up (additional conditions ensure proper avoidance)
            if xs >= 0 && ys >= 0 && local_temp(i + c - 1, 1) <= avoid_lines(3, 1, j) && local_temp(i + c - 1, 2) <= avoid_lines(3, 2, j) && dist_current > dist_dropoff_avoid_main(j)
                
                % Determine possible point of intersection for left side of
                % object
                x = avoid_lines(1, 1, j);
                fX = fx(x);
                
                % Determine possible point of intersection for bot side of
                % object
                y = avoid_lines(1, 2, j);
                fY = fy(y);
    
                % Check current path intersects with left side
                if fX >= avoid_lines(1, 2, j) && fX < avoid_lines(2, 2, j)
                    
                    % If left side intersection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if local_temp(i + c, 1) <= avoid_lines(3, 1, j) || local_temp(i + c, 2) >= avoid_lines(2, 2, j)
                  
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(2, :, j); local_temp(i + c, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(2, :, j); avoid_lines(3, :, j); local_temp(i + c, :)];
                        c = c + 2;
    
                    end
    
                % Check if current path intersects with bot side
                elseif fY > avoid_lines(5, 1, j) && fY < avoid_lines(4, 1, j)
    
                    % If bot side intersection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if local_temp(i + c, 2) <= avoid_lines(3, 2, j) || local_temp(i + c, 1) >= avoid_lines(3, 1, j)
                        
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(4, :, j); local_temp(i + c, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(4, :, j); avoid_lines(3, :, j); local_temp(i + c, :)];
                        c = c + 2;
    
                    end
    
                end
               
            % If current path moves right and down (additional conditions ensure proper avoidance)
            elseif xs >= 0 && ys <= 0 && local_temp(i + c - 1, 1) <= avoid_lines(3, 1, j) && local_temp(i + c - 1, 2) >= avoid_lines(1, 2, j) && dist_current > dist_dropoff_avoid_main(j)
    
                % Determine possible point of intersection for left side of
                % object
                x = avoid_lines(1, 1, j);
                fX = fx(x);
    
                % Determine possible point of intersection for top side of
                % object
                y = avoid_lines(2, 2, j);
                fY = fy(y);
    
                % Check if current path intersects with left side of object
                if fX > avoid_lines(1, 2, j) && fX <= avoid_lines(2, 2, j)
    
                    % If left side intersection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if  local_temp(i + c, 1) <= avoid_lines(4, 1, j) || local_temp(i + c, 2) <= avoid_lines(4, 2, j)
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(5, :, j); local_temp(i + c, :)];
                        c = c + 1;
                
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(5, :, j); avoid_lines(4, :, j); local_temp(i + c, :)];
                        c = c + 2;
                    
                    end
                
                % Check if current path intersects with top side of object
                elseif fY > avoid_lines(5, 1, j) && fY < avoid_lines(4, 1, j)
    
                    % If top side intersection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if local_temp(i + c, 2) >= avoid_lines(4, 2, j) || local_temp(i + c, 1) >= avoid_lines(4, 1, j)
                        
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(3, :, j); local_temp(i + c, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(3, :, j); avoid_lines(4, :, j); local_temp(i + c, :)];
                        c = c + 2;
                    
                    end
    
                end
    
     
            % If current path moves left and down (additional conditions ensure proper avoidance)
            elseif xs <= 0 && ys <= 0 && local_temp(i + c - 1, 1) >= avoid_lines(2, 1, j) && local_temp(i + c - 1, 2) >= avoid_lines(1, 2, j) && dist_current > dist_dropoff_avoid_main(j)
    
                % Determine possible point of intersection for right side of
                % object
                x = avoid_lines(3, 1, j);
                fX = fx(x);
    
                % Determine possible point of intersection for top side of
                % object
                y = avoid_lines(2, 2, j);
                fY = fy(y);
            
                % Check if current path intersects with right side of object
                if fX > avoid_lines(4, 2, j) && fX <= avoid_lines(3, 2, j)
    
                    % If right side intesection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if local_temp(i + c, 1) >= avoid_lines(5, 1, j) || local_temp(i + c, 2) <= avoid_lines(5, 2, j)
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(4, :, j); local_temp(i + c, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(4, :, j); avoid_lines(5, :, j); local_temp(i + c, :)];
                        c = c + 2;
    
                    end
                
                % Check if current path intersects with top side of object
                elseif fY > avoid_lines(2, 1, j) && fY < avoid_lines(3, 1, j)
    
                    % If top side intersection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if local_temp(i + c, 2) >= avoid_lines(1, 2, j) || local_temp(i + c, 1) <= avoid_lines(5, 1, j)
                        
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(2, :, j); local_temp(i + c, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(2, :, j); avoid_lines(1, :, j); local_temp(i + c, :)];
                        c = c + 2;
    
                    end
    
                end
    
            % If current path moves left and up (additional conditions ensure proper avoidance)
            elseif xs <= 0 && ys >= 0 && local_temp(i + c - 1, 1) >= avoid_lines(1, 1, j) && local_temp(i + c - 1, 2) <= avoid_lines(3, 2, j) && dist_current > dist_dropoff_avoid_main(j)
    
                % Determine possible point of intersection for right side of
                % object
                x = avoid_lines(3, 1, j);
                fX = fx(x);
    
                % Determine possible point of intersection for bot side of
                % object
                y = avoid_lines(1, 2, j);
                fY = fy(y);
    
                % Check if current path intersects with right side of object
                if fX >= avoid_lines(4, 2, j) && fX < avoid_lines(3, 2, j)
    
                    % If right side intersection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if local_temp(i + c, 1) >= avoid_lines(2, 1, j) || local_temp(i + c, 2) >= avoid_lines(2, 2, j)
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(3, :, j); local_temp(i + c, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(3, :, j); avoid_lines(2, :, j); local_temp(i + c, :)];
                        c = c + 2;
    
                    end
    
                % Check if current path intersects with bot side of object
                elseif fY > avoid_lines(5, 1, j) && fY < avoid_lines(4, 1, j)
    
                    % If right side intersection, determine proper mnaneuver
                    % for avoidance and update GPS list accordingly
                    if local_temp(i + c, 2) <= avoid_lines(2, 2, j) || local_temp(i + c, 1) <= avoid_lines(2, 1, j)
                        
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(1, :, j); local_temp(i + c, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(1, :, j); avoid_lines(2, :, j); local_temp(i + c, :)];
                        c = c + 2;
    
                    end
    
                end
    
            end
    
        end
    
    end

    % Determine total distance of current GPS path
    dist_tot = 0;
    for q = 2:length(local_temp)
        dist_tot = dist_tot + pdist2(local_temp(q - 1, :), local_temp(q, :));
    end
    
    % Check if the new path is a shorter distance than the previous
    % shortest path
    if shortest > dist_tot

        % If shorter, store path and store new shortest value
        GPS_shortest = local_temp;
        shortest = dist_tot;

    end

end



%--------------------------------------------------------------------------%
%                          FINAL COLLISION CHECK                           %
%--------------------------------------------------------------------------%

% Reloop through optimized path to ensure no collisions with obstacles
flag = true;
local_temp = GPS_shortest;

% Loop until no collisions detected
count = 0;
while flag
    flag = false;
    c = 0;
    count = count + 1;
    for i = 2:length(local_temp)
        
        % Determine direction of travel
        xs = local_temp(i + c, 1) - local_temp(i + c - 1, 1);
        ys = local_temp(i + c, 2) - local_temp(i + c - 1, 2);
    
        % Determine function of current path
        fx = @(x) (local_temp(i + c, 2) - local_temp(i + c - 1, 2))/(local_temp(i + c, 1) - local_temp(i + c - 1, 1)).*(x - local_temp(i + c - 1, 1)) + local_temp(i + c - 1, 2);
        fy = @(y) (local_temp(i + c, 1) - local_temp(i + c - 1, 1))/(local_temp(i + c, 2) - local_temp(i + c - 1, 2)).*(y - local_temp(i + c - 1, 2)) + local_temp(i + c - 1, 1);
    
        % Sort the obstacles in order from closest to farthest on current
        % pathway
        dist_dropoff_avoid = pdist2(local_temp(i + c - 1, :), avoid(:, 1:2));
        dist_dropoff_avoid_main = dist_dropoff_avoid;
    
        % Sort through the obstacles
        avoid_lines = zeros(size(avoid_lines_main));
        for m = 1:size(avoid, 1)
    
            [~, sort] = min(dist_dropoff_avoid);
            dist_dropoff_avoid_main(m) = dist_dropoff_avoid(sort);
            dist_dropoff_avoid(sort) = NaN;
            avoid_lines(:, :, m) = avoid_lines_main(:, :, sort);
    
        end
        dist_current = pdist2(local_temp(i + c - 1, :), local_temp(i + c, :));
    
        % Determine intersection points for each obstacle
        for j = 1:size(avoid, 1)
    
            % Determine case based on direction of travel
    
            % If current path moves right and up (additional conditions ensure proper avoidance)
            if xs >= 0 && ys >= 0 && local_temp(i + c - 1, 1) <= avoid_lines(3, 1, j) && local_temp(i + c - 1, 2) <= avoid_lines(3, 2, j) && dist_current > dist_dropoff_avoid_main(j)
                
                % Determine possible point of intersection for left side of
                % object
                x = avoid_lines(1, 1, j);
                fX = fx(x);
                
                % Determine possible point of intersection for bot side of
                % object
                y = avoid_lines(1, 2, j);
                fY = fy(y);

                % Check current path intersects with left side
                if fX > avoid_lines(1, 2, j) && fX < avoid_lines(2, 2, j)
                    
                    % If left side intersection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if local_temp(i + c, 1) <= avoid_lines(3, 1, j) || local_temp(i + c, 2) >= avoid_lines(2, 2, j)
                  
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(2, :, j); local_temp(i + c:end, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(2, :, j); avoid_lines(3, :, j); local_temp(i + c:end, :)];
                        c = c + 2;
    
                    end
    
                % Check if current path intersects with bot side
                elseif fY > avoid_lines(5, 1, j) && fY < avoid_lines(4, 1, j)
    
                    % If bot side intersection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if local_temp(i + c, 2) <= avoid_lines(3, 2, j) || local_temp(i + c, 1) >= avoid_lines(3, 1, j)
                        
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(4, :, j); local_temp(i + c:end, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(4, :, j); avoid_lines(3, :, j); local_temp(i + c:end, :)];
                        c = c + 2;
    
                    end
    
                end
               
            % If current path moves right and down (additional conditions ensure proper avoidance)
            elseif xs >= 0 && ys <= 0 && local_temp(i + c - 1, 1) <= avoid_lines(3, 1, j) && local_temp(i + c - 1, 2) >= avoid_lines(1, 2, j) && dist_current > dist_dropoff_avoid_main(j)
    
                % Determine possible point of intersection for left side of
                % object
                x = avoid_lines(1, 1, j);
                fX = fx(x);
    
                % Determine possible point of intersection for top side of
                % object
                y = avoid_lines(2, 2, j);
                fY = fy(y);
    
                % Check if current path intersects with left side of object
                if fX > avoid_lines(1, 2, j) && fX < avoid_lines(2, 2, j)
    
                    % If left side intersection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if  local_temp(i + c, 1) <= avoid_lines(4, 1, j) || local_temp(i + c, 2) <= avoid_lines(4, 2, j)
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(5, :, j); local_temp(i + c:end, :)];
                        c = c + 1;
                
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(5, :, j); avoid_lines(4, :, j); local_temp(i + c:end, :)];
                        c = c + 2;
                    
                    end
                
                % Check if current path intersects with top side of object
                elseif fY > avoid_lines(5, 1, j) && fY < avoid_lines(4, 1, j)
    
                    % If top side intersection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if local_temp(i + c, 2) >= avoid_lines(4, 2, j) || local_temp(i + c, 1) >= avoid_lines(4, 1, j)
                        
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(3, :, j); local_temp(i + c:end, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(3, :, j); avoid_lines(4, :, j); local_temp(i + c:end, :)];
                        c = c + 2;
                    
                    end
    
                end
    
     
            % If current path moves left and down (additional conditions ensure proper avoidance)
            elseif xs <= 0 && ys <= 0 && local_temp(i + c - 1, 1) >= avoid_lines(2, 1, j) && local_temp(i + c - 1, 2) >= avoid_lines(1, 2, j) && dist_current > dist_dropoff_avoid_main(j)
    
                % Determine possible point of intersection for right side of
                % object
                x = avoid_lines(3, 1, j);
                fX = fx(x);
    
                % Determine possible point of intersection for top side of
                % object
                y = avoid_lines(2, 2, j);
                fY = fy(y);
            
                % Check if current path intersects with right side of object
                if fX > avoid_lines(4, 2, j) && fX < avoid_lines(3, 2, j)
    
                    % If right side intesection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if local_temp(i + c, 1) >= avoid_lines(5, 1, j) || local_temp(i + c, 2) <= avoid_lines(5, 2, j)
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(4, :, j); local_temp(i + c:end, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(4, :, j); avoid_lines(5, :, j); local_temp(i + c:end, :)];
                        c = c + 2;
    
                    end
                
                % Check if current path intersects with top side of object
                elseif fY > avoid_lines(2, 1, j) && fY < avoid_lines(3, 1, j)
    
                    % If top side intersection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if local_temp(i + c, 2) >= avoid_lines(1, 2, j) || local_temp(i + c, 1) <= avoid_lines(5, 1, j)
                        
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(2, :, j); local_temp(i + c:end, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(2, :, j); avoid_lines(1, :, j); local_temp(i + c:end, :)];
                        c = c + 2;
    
                    end
    
                end
    
            % If current path moves left and up (additional conditions ensure proper avoidance)
            elseif xs <= 0 && ys >= 0 && local_temp(i + c - 1, 1) >= avoid_lines(1, 1, j) && local_temp(i + c - 1, 2) <= avoid_lines(3, 2, j) && dist_current > dist_dropoff_avoid_main(j)
    
                % Determine possible point of intersection for right side of
                % object
                x = avoid_lines(3, 1, j);
                fX = fx(x);
    
                % Determine possible point of intersection for bot side of
                % object
                y = avoid_lines(1, 2, j);
                fY = fy(y);
    
                % Check if current path intersects with right side of object
                if fX > avoid_lines(4, 2, j) && fX < avoid_lines(3, 2, j)
    
                    % If right side intersection, determine proper maneuver for
                    % avoidance and update GPS list accordingly
                    if local_temp(i + c, 1) >= avoid_lines(2, 1, j) || local_temp(i + c, 2) >= avoid_lines(2, 2, j)
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(3, :, j); local_temp(i + c:end, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(3, :, j); avoid_lines(2, :, j); local_temp(i + c:end, :)];
                        c = c + 2;
    
                    end
    
                % Check if current path intersects with bot side of object
                elseif fY > avoid_lines(5, 1, j) && fY < avoid_lines(4, 1, j)
    
                    % If right side intersection, determine proper mnaneuver
                    % for avoidance and update GPS list accordingly
                    if local_temp(i + c, 2) <= avoid_lines(2, 2, j) || local_temp(i + c, 1) <= avoid_lines(2, 1, j)
                        
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(1, :, j); local_temp(i + c:end, :)];
                        c = c + 1;
    
                    else
    
                        local_temp = [local_temp(1:i + c - 1, :); avoid_lines(1, :, j); avoid_lines(2, :, j); local_temp(i + c:end, :)];
                        c = c + 2;
    
                    end
    
                end
    
            end
    
        end
    
    end

    % If a collision was found, set flag back to true to retest updated
    % path
    % If 20 iterations hit, break loop
    count = count + 1;
    if c > 0 && count ~= 20
        flag = true;
    end
end

% Save optimized path
local = local_temp;



%--------------------------------------------------------------------------%
%                                PLOTTING                                  %
%--------------------------------------------------------------------------%

% Establish variable for use in plotting
p = [pickup; dropoff];
colors = rand(length(dropoff) + 1, 3);

% Plot all avoidance objects
figure(1)
hold on
for i = 1:size(avoid, 1)
    plot(avoid_lines_plot(:, 1, i), avoid_lines_plot(:, 2, i))
end

% Plot the pickup and drop off locations
for i = 1:length(dropoff) + 1
    if i == 1
        plot(p(i, 1), p(i, 2), '^', 'Color', colors(i, :), 'LineWidth', 2)
    else
        plot(p(i, 1), p(i, 2), 'o', 'Color', colors(i, :), 'LineWidth', 2)
    end
end

% Plot the path lines
plot(local(:, 1), local(:, 2), 'k--'), grid, xlabel('X'), ylabel('Y')



%--------------------------------------------------------------------------%
%                          PARAMETRIC INTERP                               %
%--------------------------------------------------------------------------%

% Define time step based on GPS coords distance input
%tstep = cardyn.dstep/cardyn.v_ave;
sp = cscvn(local');
tstep = cardyn.dstep/cardyn.vave;
t = 0:tstep:sp.breaks(end);

% Define the solution path and plot
sp_sol = ppval(sp, t);
plot(sp_sol(1, :), sp_sol(2, :))



%--------------------------------------------------------------------------%
%                               DUBINS PATH                                %
%                            WORK IN PROGRESS                              %
%--------------------------------------------------------------------------%

% Define a dubins connection object and turn radius parameter
dco = dubinsConnection;
dco.MinTurningRadius = cardyn.mtr;

% Define initial path vector and 
vec0 = local(2, :) - local(1, :);
vecf =  local(end, :) - local(end - 1, :);
theta_ent1 = atan2(vecf(2), vecf(1));
theta_ent2 = atan2(vec0(2), vec0(1));
theta_exit = atan2(1/2*(sin(theta_ent1) + sin(theta_ent2)), 1/2*(cos(theta_ent1) + cos(theta_ent2)));
theta_pickup = theta_exit;

for j = 1:length(local) - 2

    % Determine angle of desired entrance
    vec1 = local(j + 1, :) - local(j, :);
    vec2 = local(j + 2, :) - local(j + 1, :);
    theta_ent1 = atan2(vec1(2), vec1(1));
    theta_ent2 = atan2(vec2(2), vec2(1));
    theta_ent = atan2(1/2*(sin(theta_ent1) + sin(theta_ent2)), 1/2*(cos(theta_ent1) + cos(theta_ent2)));

    % Define start and end pose
    startPose = [local(j, :), theta_exit];
    endPose = [local(j + 1, :), theta_ent];

    % Define dubins path for current segment
    [pathSegObj, ~] = connect(dco, startPose, endPose);

    % Show the path
    %show(pathSegObj{1})

    % Resest next exit theta to current entrance theta
    theta_exit = theta_ent;

    % If at the end, finish the path for final segment
    if j == length(local) - 2
        startPose = [local(j + 1, :), theta_exit];
        endPose = [local(j + 2, :), theta_pickup];
        [pathSegObj, ~] = connect(dco, startPose, endPose);
        %show(pathSegObj{1})
    end
   
end
hold off



%--------------------------------------------------------------------------%
%                    Coordinate Conversion to WGS84                        %
%--------------------------------------------------------------------------%

% Convert to WGS84 Coordinates (keep height at 0 meters)
local = sp_sol';
z = zeros(size(local, 1), 1);
[lat, lon, alt] = local2latlon(local(:, 1), local(:, 2), z, origin);
GPS = [lat, lon, alt];

end
% End Function