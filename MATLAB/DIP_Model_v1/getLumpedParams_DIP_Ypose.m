function lumped_params = getLumpedParams_DIP_Ypose(totalMass_kg, totalHeight_m, gender, plane, theta_rad)
%% Compute lumped parameters for double inverted pendulum 
% Input:  totalMass_kg  - total body mass [kg]
%         totalHeight_m - total body height [m]
%         gender        - 'F' or 'M'
%         plane         - 'frt' (frontal) or 'sgt' (sagittal)
%         theta_rad     - arm elevation angle from horizontal [rad]
%                         (0 = T-pose, pi/2 = arms straight up,
%                          -pi/2 = arms hanging down (I-pose),
%                          ~pi/4 = typical Y-pose)
%
% Output: lumped_params - struct with fields:
%           m1, m2  : link masses [kg]
%           c1, c2  : CoM positions from joint [m]
%           j1, j2  : moments of inertia about CoM [kg·m²]
%           L1, L2  : link lengths [m]
%
% The human is modelled as two rigid links:
%   Link 1: both legs (ankle to hip)
%   Link 2: trunk + head + both arms (hip upward)
%
% Anthropometric data from:
%   De Leva (1996) "Adjustments to Zatsiorsky-Seluyanov's segment inertia
%   parameters", J. Biomechanics 29(9):1223-1230.
% Shoulder width from:
%   NASA Man-Systems Integration Standards, Section 3.
%
% Conventions:
%   - All CoM positions are measured upward from the bottom of the link.
%   - De Leva reports CoM as % of segment length from the proximal end.
%     For segments whose proximal end is at the TOP of the kinematic chain
%     (shank proximal = knee, thigh proximal = hip, trunk proximal = C7/T1,
%     head proximal = vertex), the ratio is flipped (100 - value) so that
%     c is measured from the bottom (ankle for link 1, hip for link 2).
%   - Arm segment CoM is measured from the
%     proximal end (shoulder to elbow to wrist).
%   - Moment of inertia j = m·(L·r_ratio)² is the segment's own MoI about
%     its CoM. We use the radius of gyration for the selected plane.


%% Reference heights [m] for the De Leva 1996 reference subjects
H_F = 1.735;   % female reference height
H_M = 1.741;   % male reference height

%% Segment table from De Leva

%                  L_F     L_M    m%F   m%M    c%F          c%M          rFrtF  rFrtM  rSgtF  rSgtM
seg.shank    = [438.6,  440.3,  4.81,  4.33,  100-43.52,  100-43.95,   26.7,  25.1,  26.3,  24.6];
seg.thigh    = [368.5,  422.2, 14.78, 14.16,  100-36.12,  100-40.95,   36.9,  32.9,  36.4,  32.9];
seg.trunk    = [614.8,  603.3, 42.57, 43.46,  100-49.64,  100-51.38,   30.7,  32.8,  29.2,  30.6];
seg.head     = [243.7,  242.9,  6.68,  6.94,  100-48.41,  100-50.02,   27.1,  30.3,  29.5,  31.5];
seg.upperarm = [275.1,  281.7,  2.55,  2.71,   57.54,      57.72,      27.8,  28.5,  26.0,  26.9];
seg.forearm  = [264.3,  268.9,  1.38,  1.62,   45.59,      45.74,      26.1,  27.6,  25.7,  26.5];
seg.hand     = [ 78.0,   86.2,  0.56,  0.61,   74.74,      79.00,      53.1,  62.8,  45.4,  51.3];

%% Scale each segment to this participant 
names = fieldnames(seg);

if gender == 1
    gender = 'M';

else
    gender = 'F';
end

for k = 1:numel(names)
    s = seg.(names{k});
    switch gender
        case 'F'
            H_ref = H_F;
            col = [1, 3, 5, 7, 9];   % female columns odd-indexed
        case 'M'
            H_ref = H_M;
            col = [2, 4, 6, 8, 10];  % male columns even-indexed
    end
    L_seg = (s(col(1)) / (H_ref*1000)) * totalHeight_m;   % segment length m
    m_seg = (s(col(2)) / 100)          * totalMass_kg;     % segment mass kg
    c_seg = (s(col(3)) / 100)          * L_seg;            % CoM from ref end m

    switch plane
        case 'frt'; r = s(col(4)) / 100;
        case 'sgt'; r = s(col(5)) / 100;
    end
    j_seg = m_seg * (L_seg * r)^2;     % MoI about own CoM kg·m²

    p.(names{k}).m = m_seg;
    p.(names{k}).L = L_seg;
    p.(names{k}).c = c_seg;
    p.(names{k}).j = j_seg;
end

%% Shoulder joint center position relative to hip 
switch gender
    case 'F'
        l_sjc = (497.9/1735) * totalHeight_m;          % vertical m
    case 'M'
        l_sjc = (515.5/1741) * totalHeight_m;
end
switch plane
    case 'sgt'
        w_sjc = 0;                                      % lateral m
    case 'frt'
        switch gender
            case 'F'; w_sjc = 0.23/2 * totalHeight_m;
            case 'M'; w_sjc = 0.22/2 * totalHeight_m;
        end
end


% LINK 1: both legs 

Sh = p.shank;   Th = p.thigh;

link1.m = 2*(Sh.m + Th.m);
link1.L = Sh.L + Th.L;

% CoM position from ankle by using mass-weighted average
link1.c = (2*Sh.m*Sh.c + 2*Th.m*(Sh.L + Th.c)) / link1.m;

% MoI about link1.c using parallel axis theorem, ×2 for both legs
link1.j = 2*( Sh.j + Sh.m*(Sh.c - link1.c)^2+ Th.j + Th.m*((Sh.L + Th.c) - link1.c)^2 );


%LINK 2 : trunk + head + both arms, Y-pose 
Ua = p.upperarm;  Fa = p.forearm;  Ha = p.hand;
Tr = p.trunk;     Hd = p.head;

link2.m = 2*(Ua.m + Fa.m + Ha.m) + Tr.m + Hd.m;
link2.L = Tr.L + Hd.L;

% Distance along the arm chain from shoulder to each segment's CoM
d_ua = Ua.c;
d_fa = Ua.L + Fa.c;
d_ha = Ua.L + Fa.L + Ha.c;

% Vertical position of each arm-segment CoM above the hip
% shoulder height + upward projection of along-arm distance sin > 0 when arms are above horizontal
z_ua = l_sjc + d_ua * sin(theta_rad);
z_fa = l_sjc + d_fa * sin(theta_rad);
z_ha = l_sjc + d_ha * sin(theta_rad);

% CoM of link 2 from hip mass-weighted average
link2.c = ( 2*Ua.m*z_ua + 2*Fa.m*z_fa + 2*Ha.m*z_ha ...
          + Tr.m*Tr.c + Hd.m*(Tr.L + Hd.c) ) / link2.m;

% MoI of link 2 about link2.c 
% Parallel axis theorem for each segment.
%
% Vertical offset from composite CoM:
dz_ua = z_ua - link2.c;
dz_fa = z_fa - link2.c;
dz_ha = z_ha - link2.c;
dz_tr = Tr.c          - link2.c;
dz_hd = Tr.L + Hd.c   - link2.c;

% Mediolateral offset only matters in the frontal plane
% shoulder half-width + horizontal projection of along-arm distance
switch plane
    case 'frt'
        dy_ua = w_sjc + d_ua * cos(theta_rad);
        dy_fa = w_sjc + d_fa * cos(theta_rad);
        dy_ha = w_sjc + d_ha * cos(theta_rad);
    case 'sgt'
        % Arms extend laterally  direction is perpendicular to the
        % sagittal plane so does not contribute a moment arm.
        dy_ua = 0;
        dy_fa = 0;
        dy_ha = 0;
end

link2.j = 2*Ua.j + 2*Ua.m*(dz_ua^2 + dy_ua^2) ...
        + 2*Fa.j + 2*Fa.m*(dz_fa^2 + dy_fa^2) ...
        + 2*Ha.j + 2*Ha.m*(dz_ha^2 + dy_ha^2) ...
        + Tr.j   + Tr.m*dz_tr^2 ...
        + Hd.j   + Hd.m*dz_hd^2;

%% Pack output 
lumped_params.m1 = link1.m;
lumped_params.m2 = link2.m;
lumped_params.c1 = link1.c;
lumped_params.c2 = link2.c;
lumped_params.j1 = link1.j;
lumped_params.j2 = link2.j;
lumped_params.L1 = link1.L;
lumped_params.L2 = link2.L;

end