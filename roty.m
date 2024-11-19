function R = roty(q)
% 3D rotation matrix with rotation about the y-axis
% The angle q can be symbolic or a number in radians

R = [
     cos(q), 0, sin(q)
          0, 1,      0
    -sin(q), 0, cos(q)];