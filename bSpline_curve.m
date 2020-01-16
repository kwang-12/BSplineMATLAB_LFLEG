function dot = bSpline_curve(CP, u_value, u_vec, properties)
    n = properties.n;
    k = properties.k;
    dot = 0;
    for count = 1:n+1
        dot = dot + bSpline_Nik(u_value, count, k, u_vec)*CP(count);
    end
end