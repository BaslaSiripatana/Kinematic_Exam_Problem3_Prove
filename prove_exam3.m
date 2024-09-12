% Define the symbolic variables
syms q1 q2 q3 l1 l2 l3 l4 l5 Px Py Pz

%---------------Forward Kinematic----------------------%
% Define Forward Kinematics with transformation matrix
T0_1 = [cos(q1) -sin(q1) 0 0;
        sin(q1) cos(q1) 0 0;
        0 0 1 0;
        0 0 0 1];

T1_2 = [1 0 0 0;
        0 0 -1 0;
        0 1 0 l1;
        0 0 0 1];

T2_3 = [cos(q2) -sin(q2) 0 0;
        sin(q2) cos(q2) 0 0;
        0 0 1 0;
        0 0 0 1];

T3_4 = [cos(q3) -sin(q3) 0 sqrt(l2*l2 + l4*l4);
        sin(q3) cos(q3) 0 0;
        0 0 1 0;
        0 0 0 1];

T4_E = [1 0 0 sqrt(l3*l3 + l5*l5);
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

T0_E = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_E);

disp('The resulting matrix T0_E is:');
disp(T0_E);

%--------------------Inverse Kinematic----------------------%

% The equations for q1, q2, q3 are referred from my exam paper.

% Solutions for q1
q1_sol = [atan2(Py, Px), pi + atan2(Py, Px)]

% Solutions for q2
r = sqrt(Px^2 + Py^2 + (Pz - l1)^2);
cos_row = (r^2 + l2^2 + l4^2 - l3^2 - l5^2) / (2*r*sqrt(l2^2 + l4^2));
sin_row = [sqrt(1 - cos_row^2), -sqrt(1 - cos_row^2)];

row = [atan2(sin_row(1), cos_row), atan2(sin_row(2), cos_row)];
alpha = atan2(Pz - l1, sqrt(Px^2 + Py^2));

q2_sol = [alpha - row(1), alpha - row(2)]

% Solutions for q3
cos_3 = (r*r-l2*l2-l4*l4-l3*l3-l5*l5)/(2*sqrt((l2*l2+l4*l4)*(l3*l3+l5*l5)));
sin_3 = [sqrt(1-cos_3*cos_3), -sqrt(1-cos_3*cos_3)];

q3_sol = [atan2(sin_3(1),cos_3) atan2(sin_3(2),cos_3)]

%-----------------------------------Test case--------------------------------------%
% Define 5 test cases to validate my q1_sol, q2_sol, and q3_sol from my inverse kinematic equations
% All of my inverse kinematic equations for q1_sol, q2_sol, and q3_sol are based on the answers from my exam and are correct

test_cases = [
    % Test case 1
    0.4, 0.2, 0.3, 0.2, 0.5, 0.1, 0.05, 0.6;
    % Test case 2
    -0.65, -0.1, 1.2, 0.02, 0.15, 0.55, 0.35, 0.2;
    % Test case 3
    -0.2509, 1.3521, 0.8300, 0.1836, 0.1936, 0.1780, 0.0421, 0.6197;
    % Test case 4
    0.2022, 0.6242, -0.9485, 0.2913, 0.5995, 0.2062, 0.0891, 0.2100;
    % Test case 5
    -0.3915, 0.0743, 0.0799, 0.0945, 0.4671, 0.1697, 0.1310, 0.3198;
];

for i = 1:size(test_cases, 1)
    disp(['------------------Test case ', num2str(i), '------------------']);
    
    % define test case parameters
    q1_val = test_cases(i, 1);
    q2_val = test_cases(i, 2);
    q3_val = test_cases(i, 3);
    l1_val = test_cases(i, 4);
    l2_val = test_cases(i, 5);
    l3_val = test_cases(i, 6);
    l4_val = test_cases(i, 7);
    l5_val = test_cases(i, 8);
    
    % Calculate Px Py Pz from forward kinematic
    Px_val = double(subs(T0_E(1,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_val, q2_val, q3_val, l1_val, l2_val, l3_val, l4_val, l5_val}));
    Py_val = double(subs(T0_E(2,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_val, q2_val, q3_val, l1_val, l2_val, l3_val, l4_val, l5_val}));
    Pz_val = double(subs(T0_E(3,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_val, q2_val, q3_val, l1_val, l2_val, l3_val, l4_val, l5_val}));
    
    disp('Px, Py, Pz from forward kinematic');
    disp(['Px = ', num2str(Px_val)]);
    disp(['Py = ', num2str(Py_val)]);
    disp(['Pz = ', num2str(Pz_val)]);
    
    % Solve q1_sol, q2_sol, and q3_sol using inverse kinematics (based on Px, Py, and Pz from the forward kinematics above
    q1_sol1 = double(subs(q1_sol, {Px, Py}, {Px_val, Py_val}));
    q2_sol1 = double(subs(q2_sol, {Px, Py, Pz, l1, l2, l3, l4, l5}, {Px_val, Py_val, Pz_val, l1_val, l2_val, l3_val, l4_val, l5_val}));
    q3_sol1 = double(subs(q3_sol, {Px, Py, Pz, l1, l2, l3, l4, l5}, {Px_val, Py_val, Pz_val, l1_val, l2_val, l3_val, l4_val, l5_val}));
    
    disp('q1, q2, q3 from inverse kinematic');
    disp('q1_sol = '); % from inverse kinematics calculation
    disp(q1_sol1);
    disp(['q1_val = ', num2str(q1_val)]); % real data from test case
    
    disp('q2_sol = '); % from inverse kinematics calculation
    disp(q2_sol1);
    disp(['q2_val = ', num2str(q2_val)]); % real data from test case
    
    disp('q3_sol = '); % from inverse kinematics calculation
    disp(q3_sol1);
    disp(['q3_val = ', num2str(q3_val)]); % real data from test case
    
    % From q1_sol, q2_sol, and q3_sol calculated from the inverse kinematic equations.
    % All my solutions for q1_val, q2_val, and q3_val to reach Px, Py, and Pz are proven below.
    disp('Verifying Px, Py, and Pz from inverse kinematics');
    
    % solution 1
    Px_sol1 = double(subs(T0_E(1,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_sol1(1), q2_sol1(1), q3_sol1(1), l1_val, l2_val, l3_val, l4_val, l5_val}));
    Py_sol1 = double(subs(T0_E(2,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_sol1(1), q2_sol1(1), q3_sol1(1), l1_val, l2_val, l3_val, l4_val, l5_val}));
    Pz_sol1 = double(subs(T0_E(3,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_sol1(1), q2_sol1(1), q3_sol1(1), l1_val, l2_val, l3_val, l4_val, l5_val}));
    
    fprintf('Solution 1: q1 = %.4f, q2 = %.4f, q3 = %.4f\n', q1_sol1(1), q2_sol1(1), q3_sol1(1));
    disp(['Px_sol1 = ', num2str(Px_sol1)]);
    disp(['Py_sol1 = ', num2str(Py_sol1)]);
    disp(['Pz_sol1 = ', num2str(Pz_sol1)]);
    
    % solution 2
    Px_sol2 = double(subs(T0_E(1,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_sol1(1), q2_sol1(2), q3_sol1(2), l1_val, l2_val, l3_val, l4_val, l5_val}));
    Py_sol2 = double(subs(T0_E(2,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_sol1(1), q2_sol1(2), q3_sol1(2), l1_val, l2_val, l3_val, l4_val, l5_val}));
    Pz_sol2 = double(subs(T0_E(3,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_sol1(1), q2_sol1(2), q3_sol1(2), l1_val, l2_val, l3_val, l4_val, l5_val}));
    
    fprintf('Solution 2: q1 = %.4f, q2 = %.4f, q3 = %.4f\n', q1_sol1(1), q2_sol1(2), q3_sol1(2));
    disp(['Px_sol2 = ', num2str(Px_sol2)]);
    disp(['Py_sol2 = ', num2str(Py_sol2)]);
    disp(['Pz_sol2 = ', num2str(Pz_sol2)]);
    
    % solution 3
    Px_sol3 = double(subs(T0_E(1,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_sol1(2), pi()-q2_sol1(1), -q3_sol1(1), l1_val, l2_val, l3_val, l4_val, l5_val}));
    Py_sol3 = double(subs(T0_E(2,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_sol1(2), pi()-q2_sol1(1), -q3_sol1(1), l1_val, l2_val, l3_val, l4_val, l5_val}));
    Pz_sol3 = double(subs(T0_E(3,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_sol1(2), pi()-q2_sol1(1), -q3_sol1(1), l1_val, l2_val, l3_val, l4_val, l5_val}));
    
    fprintf('Solution 3: q1 = %.4f, q2 = %.4f, q3 = %.4f\n', q1_sol1(2), pi()-q2_sol1(1), -q3_sol1(1));
    disp(['Px_sol3 = ', num2str(Px_sol3)]);
    disp(['Py_sol3 = ', num2str(Py_sol3)]);
    disp(['Pz_sol3 = ', num2str(Pz_sol3)]);
    
    % solution 4
    Px_sol4 = double(subs(T0_E(1,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_sol1(2), pi()-q2_sol1(2), -q3_sol1(2), l1_val, l2_val, l3_val, l4_val, l5_val}));
    Py_sol4 = double(subs(T0_E(2,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_sol1(2), pi()-q2_sol1(2), -q3_sol1(2), l1_val, l2_val, l3_val, l4_val, l5_val}));
    Pz_sol4 = double(subs(T0_E(3,4), {q1, q2, q3, l1, l2, l3, l4, l5}, {q1_sol1(2), pi()-q2_sol1(2), -q3_sol1(2), l1_val, l2_val, l3_val, l4_val, l5_val}));
    
    fprintf('Solution 4: q1 = %.4f, q2 = %.4f, q3 = %.4f\n', q1_sol1(2), pi()-q2_sol1(2), -q3_sol1(2));
    disp(['Px_sol4 = ', num2str(Px_sol4)]);
    disp(['Py_sol4 = ', num2str(Py_sol4)]);
    disp(['Pz_sol4 = ', num2str(Pz_sol4)]);

end

% prove that my answer is all true
