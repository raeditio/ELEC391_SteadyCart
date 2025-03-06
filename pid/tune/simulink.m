clc; clear; close all;

% Create a new Simulink model
modelName = 'InvertedPendulumPID';
new_system(modelName);
open_system(modelName);

% Add blocks
add_block('simulink/Sources/Step', [modelName, '/Step Input']);
add_block('simulink/Math Operations/Sum', [modelName, '/Sum'], ...
          'Inputs', '|+-');
add_block('simulink/Continuous/PID Controller', [modelName, '/PID']);
add_block('simulink/Continuous/Transfer Fcn', [modelName, '/Plant']);
add_block('simulink/Sinks/Scope', [modelName, '/Scope']);

% Set Transfer Function Parameters (Inverted Pendulum)
set_param([modelName, '/Plant'], 'Numerator', '1', ...
          'Denominator', '[4.4911, 0, -24.525]');

% Connect Blocks
add_line(modelName, 'Step Input/1', 'Sum/1');
add_line(modelName, 'Sum/1', 'PID/1');
add_line(modelName, 'PID/1', 'Plant/1');
add_line(modelName, 'Plant/1', 'Scope/1');
add_line(modelName, 'Plant/1', 'Sum/2');

% Save and Open the Model
save_system(modelName);
disp('Simulink model created! Open it and tune the PID block manually.');
