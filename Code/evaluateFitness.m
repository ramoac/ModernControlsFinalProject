% Function to evaluate the fitness of a solution
function fitness = evaluateFitness(solution, A, B, C, D, t, weights)
    Q = diag([solution(1), solution(2), solution(3), solution(4)]);
    R = solution(5);
    
    controller = lqr(A,B,Q,R);

    %Inputting gain value into simulink file
    simIn = Simulink.SimulationInput('SystemDiagram');
    simIn = setVariable(simIn,'k',controller);
    out = sim(simIn);
    %Pendulum angle
    Pang = out.yout{1}.Values.Data;
    %Motor input
    Minp = out.yout{2}.Values.Data;
    %Rotary arm angle
    Rang = out.yout{3}.Values.Data;
    tim = out.yout{3}.Values.Time;
    index = size(tim,1);
    
    %Searching through response outputs from end to beginning to find what
    %time the output stays within +\-2% of steady state.
    while ((Rang(index)>=0.98*30) && (Rang(index)<=1.02*30))
        index=index-1;
    end

    settlingTime = tim(index);
    overshoot = 100*(max(Rang) - Rang(end))/(Rang(end)-Rang(1));
    steadyStateError = abs(30-Rang(end));
    %Omar: I still don't think we're calculating the motor input correctly
    %because it looks nothing like what we got in the lab. And also
    %stupidly big like wtf do you mean we're inputting 3000?!?!
    inputSignal = max(Minp);
    % inputSignal=0;
    error=sum(abs(tim.*(30-Rang)));
    % error=0;
    
    fitness = weights(1) * settlingTime + weights(2) * overshoot + ...
        weights(3) * steadyStateError + weights(4) * inputSignal+ ...
        weights(5)*error;

    if isnan(fitness) || isinf(fitness)
        fitness=6.02*10^23;
    end
end
