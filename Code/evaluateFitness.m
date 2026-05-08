% Function to evaluate the fitness of a solution
function fitness = evaluateFitness(solution, A, B, C, D, t, weights)
    Q = diag([solution(1), solution(2), solution(3), solution(4)]);
    R = solution(5);
    
    controller = lqr(A,B,Q,R);
    simIn = Simulink.SimulationInput('SystemDiagram');
    simIn = setVariable(simIn,'k',controller);
    out = sim(simIn);
    Pang = out.yout{1}.Values.Data;
    Minp = out.yout{2}.Values.Data;
    Rang = out.yout{3}.Values.Data;
    tim = out.tout;
    index = size(tim,1);
    settlingTime = 4;

        while ((Rang(index)>=0.98*30) && (Rang(index)<=1.02*30))
            index=index-1;
            settlingTime = tim(index);
        end
    
    overshoot = max(Rang) - 1;
    steadyStateError = abs(30-Rang(end));
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
