function U = bSpline_KnotVector(properties)
    k = properties.k;
    n = properties.n;
    delta_time = properties.delta_time;
    
    U = ones(1,1+(n+2*k))*0.5;
    U(1+(0):1+(k)) = zeros(1,k+1);
    U(1+(n+k):1+(n+2*k)) = ones(1,k+1);
    
    for i = 1+(k+1):1+(n+k-1)
        sum = 0;
        for j = 1+(0):1+(n-1)
            sum = sum + delta_time(j);
        end
        U(i) = U(i-1) + abs(delta_time(i-k-1))/sum;
    end
    U = round(U,4);
end