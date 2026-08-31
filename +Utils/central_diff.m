function dx = central_diff(x)
% CENTRAL_DIFF - Differences and approximate derivatives
% This MATLAB function implements the central difference method.
%
% Syntax
%   dx = CENTRAL_DIFF(x)
%
% Input Arguments
%   x - Input array (at least three elements required)
%
% See also DIFF

dx = zeros(size(x));

% (x_{n+1} - x_{n})
simple_diff = diff(x);

% Calculate the central difference for the inner datapoints. Using
% simple_diff, this is effectively,
% ((x_{n} - x_{n-1}) + (x_{n+1} - x_{n}) ) / 2
% which is equivalent to the reduced and more common form,
% (x_{n+1} - x_{n-1}) / 2
dx(2:end-1) = 0.5 * (simple_diff(1:end-1) + simple_diff(2:end));

% Use the simple diff to fill the edges, where the central diff cannot be
% taken
dx(1)  = simple_diff(1);
dx(end)= simple_diff(end);

end