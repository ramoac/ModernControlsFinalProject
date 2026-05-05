% Function to initialize the population of bees
function bees = initializeBees(numBees, q11Range, q22Range, q33Range, q44Range, rRange, A, B, C, D, t, weights)
    bees(numBees).solution = [];
    bees(numBees).fitness = [];
    bees(numBees).trial = [];
    
    for i = 1:numBees
        bees(i).solution = generateRandomSolution(q11Range, q22Range, q33Range, q44Range, rRange);
        bees(i).fitness = evaluateFitness(bees(i).solution, A, B, C, D, t, weights);
        bees(i).trial = 0;
    end
end