function U = uunifast(U_total, n)
    U = zeros(1, n);
    sumU = U_total;
    for i = 1:n-1
        nextSumU = sumU * rand()^(1/(n-i));
        U(i) = sumU - nextSumU;
        sumU = nextSumU;
    end
    U(n) = sumU;
end