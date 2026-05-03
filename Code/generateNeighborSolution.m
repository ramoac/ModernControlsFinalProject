% Function to generate a neighbor solution
function neighborSolution = generateNeighborSolution(solution, q11Range, q22Range, q33Range, q44Range, rRange)
    neighborSolution = solution;
    for i = 1:length(solution)
        % Modify each parameter by adding/subtracting a small random value
        neighborSolution(i) = neighborSolution(i) + (rand() - 0.5) * 0.2 * (q11Range(2) - q11Range(1));
        
        % Ensure the new value is within the search range
        neighborSolution(i) = max(min(neighborSolution(i), q11Range(2)), q11Range(1));
    end
end
