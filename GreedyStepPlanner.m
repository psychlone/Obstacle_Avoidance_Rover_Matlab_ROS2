function heading = GreedyStepPlanner(x, y, theta, goalXY, map, ...
    stepLen, corridorHalfW, forwardMargin, sidestepDeg)
%#codegen
% Aim to goal; if forward corridor blocked, try candidates in sidestepDeg.
% Returns heading (rad) or NaN if none is feasible.

% Desired goal heading
gx = goalXY(1); gy = goalXY(2);
goalHeading = atan2(gy - y, gx - x);

% Build candidate headings: goal first, then ±sidesteps relative to goal
nC = numel(sidestepDeg);
cands = zeros(nC,1);
for i = 1:nC
    cands(i) = goalHeading + deg2rad(sidestepDeg(i));
end

% Check each candidate with a simple corridor collision test
for i = 1:nC
    hdg = cands(i);
    if corridorIsFree(map, [x,y], hdg, stepLen + forwardMargin, corridorHalfW)
        heading = wrapToPi(hdg);
        return;
    end
end

% If everything fails, optionally try current heading as last resort
if corridorIsFree(map, [x,y], theta, stepLen + forwardMargin, corridorHalfW)
    heading = wrapToPi(theta);
    return;
end

heading = NaN;  % stuck
end

function ok = corridorIsFree(map, p0, hdg, L, halfW)
%#codegen
% Check 3 rays: center, left edge, right edge (corridor approx).
% If all remain free, corridor is free. March with ds from map resolution.

ds = 1.0/map.Resolution;
% Edges: perpendicular offset
nx = -sin(hdg); ny = cos(hdg);

pC = p0;                                 % center
pL = [p0(1) + halfW*nx, p0(2) + halfW*ny];
pR = [p0(1) - halfW*nx, p0(2) - halfW*ny];

ok = rayFree(map, pC, hdg, L, ds) && ...
     rayFree(map, pL, hdg, L, ds) && ...
     rayFree(map, pR, hdg, L, ds);
end

function tf = rayFree(map, p0, hdg, L, ds)
%#codegen
% March along a ray and ensure cells are not occupied.

tf = true;
s  = 0.0;
while s <= L
    xi = p0(1) + s*cos(hdg);
    yi = p0(2) + s*sin(hdg);

    % Bounds check
    if (xi < map.XWorldLimits(1)) || (xi > map.XWorldLimits(2)) || ...
       (yi < map.YWorldLimits(1)) || (yi > map.YWorldLimits(2))
        tf = false; % treat out-of-bounds as blocked
        return;
    end

    occ = checkOccupancy(map, [xi, yi]);
    if occ > 0.5
        tf = false;
        return;
    end
    s = s + ds;
end
end
