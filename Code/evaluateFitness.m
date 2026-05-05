% Function to evaluate the fitness of a solution
function fitness = evaluateFitness(solution, A, B, C, D, t, weights)
    Q = diag([solution(1), solution(2), solution(3), solution(4)]);
    R = solution(5);
    
    controller = lqr(A,B,Q,R);
    [num,den] = ss2tf(A-B*controller,B,C,D);
    G=tf(num,den);
    sysClosedLoop = feedback(G, 1);
    sysClosedLoopInput = feedback(1, G);
    y = step(sysClosedLoop, t);
    U = step(sysClosedLoopInput, t);
    
    if y(end)>=0.98
        idxTime=find(y > 0.98, 1);
        settlingTime = t(idxTime);
    else
        settlingTime=t(end);
    end
    overshoot = max(y) - 1;
    steadyStateError = abs(1-y(end));
    inputSignal = max(abs(U));
    % inputSignal=0;
    error=sum(abs(t'.*(1-y)));
    % error=0;
    
    fitness = weights(1) * settlingTime + weights(2) * overshoot + ...
        weights(3) * steadyStateError + weights(4) * inputSignal+ ...
        weights(5)*error;

    if isnan(fitness) || isinf(fitness)
        fitness=6.02*10^23;
    end
end
