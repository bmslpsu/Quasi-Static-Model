function R = rotMatX(q)
% 3D rotation matrix with rotation about the x-axis
% The angle q can be symbolic or a number in radians

R = [
    1,      0,       0
    0, cos(q), -sin(q)
    0, sin(q),  cos(q)];