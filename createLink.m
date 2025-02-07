% createLink returns a link structure with the supplied members.
%
% L = createLink(a, d, alpha, theta, offset, centOfMass, mass, inertia)
%
% Outputs:
% L = Link structure with members: a, d, alpha, theta, offset, mass,
% inertia, com, and isRotary.
% Note: isRotary = 1 if joint is rotational, 0 if prismatic, -1 if static.
%
% Inputs:
% a = DH parameter for displacement along x(i) (meters)
% d = DH parameter for displacement along z(i-1) (meters)
% alpha = DH parameter for rotation about x(i) (meters)
% theta = DH parameter for rotation about z(i-1) (meters)
% offset =  Number of radians (or meters for prismatic) difference between the encoder orientation and the DH zero-angle. θ_DH = Θ - Θ_offset (equivalent equation for prismatic)
% centOfMass = Position of the link's center of mass
% mass = Link mass (kg)
% inertia = Link mass moment of inertia (kg m^2)
%
% Chris Larson
% Robot Mechanics
% 2023-10-17

function L = createLink(a, d, alpha, theta, offset, centOfMass, mass, inertia)

L.a = a;
L.d = d;
L.alpha = alpha;
L.theta = theta;
L.offset = offset;
L.com = centOfMass;
L.mass = mass;
L.inertia = inertia;

if not(isscalar(a)) || not(isscalar(d))
    L.isRotary = 0;
elseif not(isscalar(theta)) || not(isscalar(alpha))
    L.isRotary = 1;
else
    L.isRotary = -1;
end

end

