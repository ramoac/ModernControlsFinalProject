clear; clc; close all;

t=0:0.01:5;

%% Motor
% Resistance
Rm = 8.4;
% Current-torque (N-m/A)
kt = 0.042;
% Back-emf constant (V-s/rad)
km = 0.042;
%
%% Rotary Arm
% Mass (kg)
mr = 0.095;
% Total length (m)
r = 0.085;
% Moment of inertia about pivot (kg-m^2)
Jr = mr*r^2/3;
% Equivalent Viscous Damping Coefficient (N-m-s/rad)
br = 1e-3; % damping tuned heuristically to match QUBE-Sero 2 response
%
%% Pendulum Link
% Mass (kg)
mp = 0.024;
% Total length (m)
Lp = 0.129;
% Pendulum center of mass (m)
l = Lp/2;
% Moment of inertia about pivot (kg-m^2)
Jp = mp*Lp^2/3;
% Equivalent Viscous Damping Coefficient (N-m-s/rad)
bp = 5e-5; % damping tuned heuristically to match QUBE-Sero 2 response
% Gravity Constant
g = 9.81;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Find Total Inertia
Jt = Jr*Jp - mp^2*r^2*l^2;
% 
% State Space Representation
A = [0 0 1 0;
     0 0 0 1;
     0 mp^2*l^2*r*g/Jt  -br*Jp/Jt   -mp*l*r*bp/Jt 
     0  mp*g*l*Jr/Jt    -mp*l*r*br/Jt   -Jp*bp/Jt];
%
B = [0; 0; Jp/Jt; mp*l*r/Jt];
C = [0 1 0 0];
D = 0;
% 
% Add actuator dynamics
A(3,3) = A(3,3) - km*km/Rm*B(3);
A(4,3) = A(4,3) - km*km/Rm*B(4);
A = [0 0 1 0;
     0 0 0 1;
     0  mp^2*l^2*r*g/Jt   A(3,3)   -mp*l*r*bp/Jt;
     0  mp*g*l*Jr/Jt      A(4,3)   -Jp*bp/Jt];
B = km * B / Rm;

% Define the performance criteria and objective function
% In this example, let's consider minimizing settling time, overshoot,
% steady-state error, and input signal
% You can adjust the weights according to the importance of each criterion
weights = [50, 1, 0, 0.1, 1];

% Define the ABC algorithm parameters
maxIterations = 10;  % Maximum number of iterations
numEmployedBees = 50; % Number of employed bees
numOnlookerBees = 50; % Number of onlooker bees
maxTrials = 5;         % Maximum number of trials for a scout bee

% Define the PID controller parameters search ranges
% These ranges may need adjustment depending on the specific problem
q11Range = [0.1, 50]; %rotary arm angle 
q22Range = [0.1, 50]; %pendulum angle
q33Range = [0.1, 50]; % rotary arm velocity
q44Range = [0.1, 50]; %pendulum velocity
rRange = [1, 10];

% Initialize the best solution
bestSolution = [];
bestFitness = Inf;

% Initialize the population of employed bees
employedBees = initializeBees(numEmployedBees, q11Range, q22Range, q33Range, q44Range, rRange, A, B, C, D, t, weights);

% Define the stagnation threshold
stagnationThreshold = 5; % Number of iterations without improvement to detect stagnation

% Initialize the stagnation counter
stagnationCounter = 0;

% Perform the ABC algorithm iterations
for iteration = 1:maxIterations
    % Employed bees phase
    for i = 1:numEmployedBees
        % Generate a neighbor solution
        neighborSolution = generateNeighborSolution(employedBees(i).solution, q11Range, q22Range, q33Range, q44Range, rRange);

        % Evaluate the neighbor solution fitness
        employedBees(i).neighborSolution = neighborSolution;
        employedBees(i).neighborFitness = evaluateFitness(neighborSolution, A, B, C, D, t, weights);

        % Compare the neighbor solution with the current solution
        if employedBees(i).neighborFitness < employedBees(i).fitness
            employedBees(i).solution = neighborSolution;
            employedBees(i).fitness = employedBees(i).neighborFitness;
            employedBees(i).trial = 0; % Reset the trial counter

            % Reset the stagnation counter
            % stagnationCounter = 0;
        else
            employedBees(i).trial = employedBees(i).trial + 1; % Increment the trial counter
        end
    end

    % Onlooker bees phase
    onlookerBees = selectOnlookerBees(employedBees, numOnlookerBees);

    for i = 1:numOnlookerBees
        % Generate a neighbor solution
        neighborSolution = generateNeighborSolution(onlookerBees(i).solution, q11Range, q22Range, q33Range, q44Range, rRange);

        % Evaluate the neighbor solution fitness
        onlookerBees(i).neighborSolution = neighborSolution;
        onlookerBees(i).neighborFitness = evaluateFitness(neighborSolution, A, B, C, D, t, weights);

        % Compare the neighbor solution with the current solution
        if onlookerBees(i).neighborFitness < onlookerBees(i).fitness
            onlookerBees(i).solution = neighborSolution;
            onlookerBees(i).fitness = onlookerBees(i).neighborFitness;
            onlookerBees(i).trial = 0; % Reset the trial counter

            % Reset the stagnation counter
            % stagnationCounter = 0;
        else
            onlookerBees(i).trial = onlookerBees(i).trial + 1; % Increment the trial counter
        end
    end

    % Scout bees phase
    for i = 1:numEmployedBees
        if employedBees(i).trial >= maxTrials
            % Generate a new random solution for the scout bee
            employedBees(i).solution = generateRandomSolution(q11Range, q22Range, q33Range, q44Range, rRange);
            employedBees(i).fitness = evaluateFitness(employedBees(i).solution, A, B, C, D, t, weights);
            employedBees(i).trial = 0; % Reset the trial counter
        end
    end

    % Memorize the best solution
    for i = 1:numEmployedBees
        if employedBees(i).fitness < bestFitness
            bestSolution = employedBees(i).solution;
            bestFitness = employedBees(i).fitness;
        end
    end

    % Check for stagnation
    if iteration > 1 && bestFitness == prevBestFitness
        stagnationCounter = stagnationCounter + 1;
    else
        stagnationCounter = 0;
    end

    % If stagnation is detected, perturb the best solution
    if stagnationCounter >= stagnationThreshold
        bestSolution = perturbSolution(bestSolution, q11Range, q22Range, q33Range, q44Range, rRange);
        bestFitness = evaluateFitness(bestSolution, A, B, C, D, t, weights);
        stagnationCounter = 0; % Reset the stagnation counter
    end

    % Store the current best fitness for comparison in the next iteration
    prevBestFitness = bestFitness;

    % Display the best solution in each iteration
    disp(['Iteration ', num2str(iteration), ': Best Solution = [q11 = ', num2str(bestSolution(1)), ...
        ', q22 = ', num2str(bestSolution(2)), ', q33 = ', num2str(bestSolution(3)), ...
		', q44 = ', num2str(bestSolution(4)), ', r = ', num2str(bestSolution(5)), ...
        '], Best Fitness = ', num2str(bestFitness)]);

    bobSolution(iteration,1:5)=bestSolution;
    bobFitness(iteration)=bestFitness;
end

% Apply the optimized LQR controller to the system
Q = diag([0.1,0.1,0.1,0.1]); R=1;

%Omar: This is an arbitrariy condition just to test code. May change if it doesn't work
while(Q(1,1)==0.1 && Q(2,2)==0.1 && Q(3,3)==0.1 && Q(4,4)==0.1 && R==1)
    [minFit, itr]=min(bobFitness);
    Q(1,1) = bobSolution(itr,1);
    Q(2,2) = bobSolution(itr,2);
    Q(3,3) = bobSolution(itr,3);
    Q(4,4) = bobSolution(itr,4);
    R = bobSolution(itr,5);
    % Omar: This is an arbitrariy condition just to test code. May change
    % if it doesn't work
    if(Q(1,1)==0 && Q(2,2)==0 && Q(3,3)==0 && Q(4,4)==0 && R==0)
        bobFitness(itr)=Inf;
    end
end

controller = lqr(A, B, Q, R);
[sysnum,sysden] = ss2tf(A-B*controller,B,C,D);
G=tf(sysnum,sysden);
sysClosedLoop = feedback(G, 1);
sysClosedLoopInput = feedback(1, G);
y = step(sysClosedLoop, t);
U = step(sysClosedLoopInput, t);

figure(1);
yout=step(sysClosedLoop, t);
plot(t,yout,'-r', 'LineWidth', 2);
title('Step Response of the System \theta(t)');
xlabel('time (s)');
ylabel('angle (rad)');
grid on; grid minor;

figure(2);
uout=step(sysClosedLoopInput, t);
plot(t,uout*0.01,'-k', 'LineWidth', 2); % 0.01 is radiu, T=rF
title('Input Signal of Plant u(t)');
xlabel('time (s)');
ylabel('magnitude');
grid on; grid minor;

figure(3)
plot(bobFitness,'-r')
title('Fitness of ABC')
xlabel('iterations')
ylabel('fitness')
grid on; grid minor

%save('optimalValues.mat','Q(1,1)','Q(2,2)','Q(3,3)','Q(4,4)','R');
%disp('Optimal Solution Saved!');