function R = rotMatZ(q)
% 3D rotation matrix with rotation about the z-axis
% The angle q can be symbolic or a number in radians

R = [
    cos(q), -sin(q), 0
    sin(q),  cos(q), 0
         0,       0, 1];