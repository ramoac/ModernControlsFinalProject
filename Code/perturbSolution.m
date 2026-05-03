function perturbedSolution = perturbSolution(solution, q11Range, q22Range, q33Range, q44Range, rRange)
    % Randomly perturb each component of the solution within the specified ranges

    % Perturb q11
    perturbedq11 = solution(1) + randInRange(-5, 5); % Adjust the range as desired
    perturbedq11 = clamp(perturbedq11, q11Range);

    % Perturb q22
    perturbedq22 = solution(2) + randInRange(-10, 10); % Adjust the range as desired
    perturbedq22 = clamp(perturbedq22, q22Range);

    % Perturb q33
    perturbedq33 = solution(3) + randInRange(-2, 2); % Adjust the range as desired
    perturbedq33 = clamp(perturbedq33, q33Range);
	
	% preturb q44
	preturbedq44 = solution(4) + randInRange(-10, 10); % Adjust the range as desired
	preturbedq44 = clamp(preturbedq44, q44Range);
	
	% preturb r
	preturbedr = solution(5) + randInRange(-5, 5); % Adjust the range as desired
	preturbedr = clamp(preturbedr, rRange);

    % Create the perturbed solution
    perturbedSolution = [preturbedq11, perturbedq22, perturbedq33, preturbedq44, preturbedr];
end