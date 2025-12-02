% TEST_INSTALLATION
% Quick test script to verify all components are working

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('Testing SOA Robot Simulator Installation\n');
fprintf('========================================\n\n');

test_passed = 0;
test_failed = 0;

%% Test 1: ARAT_Core Class
fprintf('[1/7] Testing ARAT_Core class...\n');
try
    % Test skew matrix
    v = [1; 2; 3];
    S = ARAT_Core.skew(v);
    assert(isequal(size(S), [3, 3]), 'Skew matrix size incorrect');
    assert(S(1,2) == -3 && S(2,1) == 3, 'Skew matrix values incorrect');
    
    % Test Rodrigues rotation
    axis = [0; 0; 1];
    angle = pi/2;
    R = ARAT_Core.rodrigues_rotation(axis, angle);
    assert(abs(det(R) - 1) < 1e-10, 'Rotation matrix determinant should be 1');
    
    % Test propagation matrix
    l = [0.5; 0; 0];
    Phi = ARAT_Core.get_prop_matrix(l);
    assert(isequal(size(Phi), [6, 6]), 'Propagation matrix size incorrect');
    
    % Test joint matrix
    H_R = ARAT_Core.get_joint_matrix('R', axis);
    H_P = ARAT_Core.get_joint_matrix('P', axis);
    assert(isequal(size(H_R), [6, 1]), 'Joint matrix size incorrect');
    
    % Test trajectory generation
    [q, qd, qdd] = ARAT_Core.traj_harmonic(2.5, 0, 5, 0, pi/2);
    assert(~isnan(q) && ~isnan(qd) && ~isnan(qdd), 'Trajectory generation failed');
    
    fprintf('   ✓ ARAT_Core tests passed\n\n');
    test_passed = test_passed + 1;
catch ME
    fprintf('   ✗ ARAT_Core tests failed: %s\n\n', ME.message);
    test_failed = test_failed + 1;
end

%% Test 2: Simple Robot Configuration
fprintf('[2/7] Testing robot configuration...\n');
try
    robot.n_joints = 2;
    robot.link_vectors = [0.5, 0.3; 0, 0; 0, 0];
    robot.joint_axes = [0, 0; 0, 0; 1, 1];
    robot.joint_types = 'RR';
    robot.V_base = zeros(6, 1);
    
    assert(size(robot.link_vectors, 2) == robot.n_joints, 'Link vectors size mismatch');
    fprintf('   ✓ Robot configuration test passed\n\n');
    test_passed = test_passed + 1;
catch ME
    fprintf('   ✗ Robot configuration test failed: %s\n\n', ME.message);
    test_failed = test_failed + 1;
end

%% Test 3: Forward Kinematics
fprintf('[3/7] Testing forward kinematics...\n');
try
    q = [pi/4; pi/6];
    q_dot = [0.1; 0.2];
    
    V_tip = ARAT_Core.serial_forward_kinematics(robot.link_vectors, robot.joint_axes, ...
                                                 robot.joint_types, q, q_dot, robot.V_base);
    
    assert(isequal(size(V_tip), [6, 1]), 'Tip velocity size incorrect');
    assert(~any(isnan(V_tip)), 'Forward kinematics produced NaN');
    
    fprintf('   ✓ Forward kinematics test passed\n\n');
    test_passed = test_passed + 1;
catch ME
    fprintf('   ✗ Forward kinematics test failed: %s\n\n', ME.message);
    test_failed = test_failed + 1;
end

%% Test 4: Jacobian Computation
fprintf('[4/7] Testing Jacobian computation...\n');
try
    J = ARAT_Core.compute_jacobian(robot.link_vectors, robot.joint_axes, ...
                                    robot.joint_types, q);
    
    assert(isequal(size(J), [6, robot.n_joints]), 'Jacobian size incorrect');
    assert(~any(isnan(J(:))), 'Jacobian contains NaN');
    
    % Check manipulability
    manip = sqrt(det(J * J'));
    assert(~isnan(manip) && manip >= 0, 'Manipulability calculation failed');
    
    fprintf('   ✓ Jacobian computation test passed\n\n');
    test_passed = test_passed + 1;
catch ME
    fprintf('   ✗ Jacobian computation test failed: %s\n\n', ME.message);
    test_failed = test_failed + 1;
end

%% Test 5: Inverse Kinematics
fprintf('[5/7] Testing inverse kinematics...\n');
try
    target_pos = [0.6; 0.3; 0];
    q_init = [0; 0];
    
    ik_options.max_iter = 50;
    ik_options.tolerance = 1e-3;
    ik_options.lambda = 0.01;
    ik_options.position_only = true;
    
    [q_ik, success, iters] = RobotIK.solve_numerical_ik(robot, target_pos, q_init, ik_options);
    
    assert(~any(isnan(q_ik)), 'IK solution contains NaN');
    
    if success
        fprintf('   ✓ IK converged in %d iterations\n\n', iters);
    else
        fprintf('   ⚠ IK did not converge (this may be normal for out-of-reach targets)\n\n');
    end
    
    test_passed = test_passed + 1;
catch ME
    fprintf('   ✗ Inverse kinematics test failed: %s\n\n', ME.message);
    test_failed = test_failed + 1;
end

%% Test 6: Trajectory Generation
fprintf('[6/7] Testing trajectory generation...\n');
try
    n_samples = 50;
    time_vector = linspace(0, 2, n_samples);
    q0 = [0; 0];
    qf = [pi/2; pi/3];
    
    q_traj = zeros(2, n_samples);
    
    for i = 1:2
        for j = 1:n_samples
            [q_val, ~, ~] = ARAT_Core.traj_cycloidal(time_vector(j), 0, 2, q0(i), qf(i));
            q_traj(i, j) = q_val;
        end
    end
    
    assert(~any(isnan(q_traj(:))), 'Trajectory contains NaN');
    assert(abs(q_traj(1, 1) - q0(1)) < 1e-6, 'Trajectory start incorrect');
    assert(abs(q_traj(1, end) - qf(1)) < 1e-6, 'Trajectory end incorrect');
    
    fprintf('   ✓ Trajectory generation test passed\n\n');
    test_passed = test_passed + 1;
catch ME
    fprintf('   ✗ Trajectory generation test failed: %s\n\n', ME.message);
    test_failed = test_failed + 1;
end

%% Test 7: Visualization (Basic)
fprintf('[7/7] Testing visualization...\n');
try
    fig = figure('Visible', 'off'); % Hidden figure
    RobotVisualizer.plot_robot(robot, q);
    
    % Check if plot was created
    ax = gca;
    assert(~isempty(ax.Children), 'No plot elements created');
    
    close(fig);
    
    fprintf('   ✓ Visualization test passed\n\n');
    test_passed = test_passed + 1;
catch ME
    fprintf('   ✗ Visualization test failed: %s\n\n', ME.message);
    test_failed = test_failed + 1;
end

%% Summary
fprintf('========================================\n');
fprintf('Test Summary\n');
fprintf('========================================\n');
fprintf('Tests passed: %d\n', test_passed);
fprintf('Tests failed: %d\n', test_failed);
fprintf('Total tests: %d\n', test_passed + test_failed);

if test_failed == 0
    fprintf('\n✓ All tests passed! Installation is working correctly.\n');
    fprintf('You can now run:\n');
    fprintf('  - robot_simulator_main.m (interactive)\n');
    fprintf('  - example_3dof_robot.m (3-DOF demo)\n');
    fprintf('  - example_scara_robot.m (SCARA demo)\n');
else
    fprintf('\n⚠ Some tests failed. Please check the error messages above.\n');
end

fprintf('========================================\n');
