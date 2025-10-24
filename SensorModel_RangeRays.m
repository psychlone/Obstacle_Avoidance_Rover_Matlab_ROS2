function [hitPoints, freeEnds] = SensorModel_RangeRays(map, pose, opts)
%#codegen
% simple 2D range sensor using ray marching in occupancyMap.
% pose = [x,y,theta]; opts: FOVdeg, range, nRays, ds

x0 = pose(1); y0 = pose(2); th = pose(3);

FOV   = deg2rad(opts.FOVdeg);
range = opts.range;
nRays = max(3, opts.nRays);
ds    = opts.ds;

hitPoints = zeros(nRays,2);
freeEnds  = zeros(nRays,2);

% Fan of angles centered on heading
a0 = th - FOV/2;
da = FOV / (nRays - 1);

for i = 1:nRays
    a = a0 + (i-1)*da;
    % March along the ray
    s = 0.0;
    hit = false;
    xi = x0; yi = y0;
    while s <= range
        xi = x0 + s*cos(a);
        yi = y0 + s*sin(a);
        % Stop if out of bounds
        if (xi < map.XWorldLimits(1)) || (xi > map.XWorldLimits(2)) || ...
           (yi < map.YWorldLimits(1)) || (yi > map.YWorldLimits(2))
            break;
        end
        % Occupancy check
        occ = checkOccupancy(map, [xi, yi]);
        if occ > 0.5
            hit = true;
            break;
        end
        s = s + ds;
    end
    if hit
        hitPoints(i,:) = [xi, yi];
        % Back up slightly to last free
        s2 = max(0.0, s - ds);
        freeEnds(i,:) = [x0 + s2*cos(a), y0 + s2*sin(a)];
    else
        hitPoints(i,:) = [NaN, NaN];
        freeEnds(i,:)  = [xi, yi]; % either last free in-range or boundary
    end
end
end
