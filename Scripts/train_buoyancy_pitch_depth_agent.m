%% observation and action space

% Observation: [depth; depth_rate; pitch; pitch_rate,FL_dot,FR_dot,B_dot]
Controller.obsInfo = rlNumericSpec([12 1], 'LowerLimit', [-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf], 'UpperLimit', [inf;inf;inf;inf;inf;inf;inf;inf;inf;inf;inf;inf]);
Controller.obsInfo.Name = 'observations';

% Action: [Force_front; Force_rear] ∈ [–1, +1] (normalized forces)
Controller.actInfo = rlNumericSpec([2 1], 'LowerLimit', [-15;-15], 'UpperLimit',[20; 20]);
Controller.actInfo.Name = 'actions';


%% DDPG agent build

% Define the critic and actor networks
Controller.obsPath = [
    featureInputLayer(prod(Controller.obsInfo.Dimension), 'Normalization', 'none', 'Name', 'obs_input')
    fullyConnectedLayer(64, 'Name', 'fc1_obs')
    reluLayer('Name', 'relu1_obs')
    fullyConnectedLayer(64, 'Name', 'fc2_obs')
    ];

Controller.actPath = [
    featureInputLayer(prod(Controller.actInfo.Dimension), 'Normalization', 'none', 'Name', 'act_input')
    fullyConnectedLayer(64, 'Name', 'fc1_act')
    reluLayer('Name','relu1_act')
    fullyConnectedLayer(64,'Name','fc2_act')
    ];

Controller.commonPath = [
    additionLayer(2, 'Name', 'add')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(1, 'Name', 'fc_out')  % Scalar Q-value output
    ];

Controller.criticNetwork = layerGraph();
Controller.criticNetwork = addLayers(Controller.criticNetwork, Controller.obsPath);
Controller.criticNetwork = addLayers(Controller.criticNetwork, Controller.actPath);
Controller.criticNetwork = addLayers(Controller.criticNetwork, Controller.commonPath);

Controller.criticNetwork = connectLayers(Controller.criticNetwork, 'fc2_obs', 'add/in1');
Controller.criticNetwork = connectLayers(Controller.criticNetwork, 'fc2_act', 'add/in2');


Controller.criticOpts = rlRepresentationOptions('LearnRate',1e-03,'GradientThreshold',1,'UseDevice','gpu');
Controller.critic = rlQValueRepresentation(Controller.criticNetwork, Controller.obsInfo, Controller.actInfo, ...
    'Observation', {'obs_input'}, 'Action', {'act_input'}, Controller.criticOpts);

Controller.actorNetwork = [
    featureInputLayer(Controller.obsInfo.Dimension(1))
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(Controller.actInfo.Dimension(1))
    tanhLayer
    scalingLayer('Name','scale','Scale',15)];

Controller.actorOpts = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1,'UseDevice','gpu');
Controller.actor = rlDeterministicActorRepresentation(Controller.actorNetwork, Controller.obsInfo, Controller.actInfo, ...
    'Observation', {'input'}, 'Action', {'scale'}, Controller.actorOpts);


%% define DDPG agent
Controller.Sampletime = 0.002;
Controller.agentOpts = rlDDPGAgentOptions(...
    'SampleTime', Controller.Sampletime,...
    'TargetSmoothFactor', 1e-3,...
    'ExperienceBufferLength', 1e6,...
    'MiniBatchSize', 64,...
    'DiscountFactor', 0.9);

%% Define Agent
Controller.agent = rlDDPGAgent(Controller.actor, Controller.critic, Controller.agentOpts);

%% Environment
Controller.env = rlSimulinkEnv(...
    'Drone2_simulink', ...
    'Drone2_simulink/Controller/RL Agent', ...
    Controller.obsInfo, Controller.actInfo);
% env.ResetFcn = @() myResetFunction(); % If needed
% env.UseFastRestart = true;  % speeds up training

%% Training options
Controller.stepperEpisode = Environment.SimulationTime/Controller.Sampletime;
Controller.trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 600, ...
    'MaxStepsPerEpisode', Controller.stepperEpisode, ...
    'ScoreAveragingWindowLength', 20, ...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue', -10, ...
    'Verbose', true, ...
    'Plots','training-progress', ...
    'UseParallel',true, ...
    'SaveAgentCriteria','AverageReward', ...
    'SaveAgentValue',-10,'SaveAgentDirectory','SaveAgents');

Controller.trainingStats = train(Controller.agent, Controller.env, Controller.trainOpts);

%% for getting structure of actor and  critic only 
% actorNet = getModel(Controller.actor);
% analyzeNetwork(actorNet);
% criticNet = getModel(Controller.critic);
% analyzeNetwork(criticNet);
%%
Controller.simoptions = rlSimulationOptions('MaxSteps',Controller.stepperEpisode,'NumSimulations',1);
Controller.experience = sim(Controller.env,Controller.agent,Controller.simoptions);
plot(Controller.experience.Observation.observations);