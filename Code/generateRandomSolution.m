% Function to generate a random solution
function solution = generateRandomSolution(q11Range, q22Range, q33Range, q44Range, rRange)
    q11 = rand() * (q11Range(2) - q11Range(1)) + q11Range(1);
    q22 = rand() * (q22Range(2) - q22Range(1)) + q22Range(1);
    q33 = rand() * (q33Range(2) - q33Range(1)) + q33Range(1);
	q44 = rand() * (q44Range(2) - q44Range(1)) + q44Range(1);
	r   = rand() * (rRange(2) - rRange(1)) + rRange(1);
    solution = [q11, q22, q33, q44, r];
end