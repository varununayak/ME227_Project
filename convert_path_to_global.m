function [E, N, psi] = convert_path_to_global( path, s, e, dpsi )
% CONVERT_PATH_TO_GLOBAL converts s and e vectors along a path defined by
%   path into EN global coordinates

% Authors: Matt Brown, Vincent Laurense, John Subosits
% Adapted by: Nathan Spielberg, John Alsterda for Stanford ME227

n = length(s);
E = zeros(n,1);
N = zeros(n,1);

centE = interp1(path.s,path.posE, mod(s,path.s(end)));
centN = interp1(path.s,path.posN, mod(s,path.s(end)));
theta = interp1(path.s,path.psi, mod(s,path.s(end)));
psi   = theta + dpsi;

for i=1:n
    E(i) = centE(i) - e(i)*sin(pi/2-theta(i));
    N(i) = centN(i) - e(i)*cos(pi/2-theta(i));
end

end

