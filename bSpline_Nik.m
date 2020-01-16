function basis = bSpline_Nik(u_val, i, k, u_vec)
% u_val
% i
% k
% u_vec
    if k == 0
        if u_val ~= u_vec(end)
            if u_vec(i) <= u_val && u_val < u_vec(i+1)
                basis = 1;
            else
                basis = 0;
            end
        else
            if u_vec(i) <= u_val && u_val <= u_vec(i+1)
                basis = 1;
            else
                basis = 0;
            end
        end
    else
        if u_vec(i+k) - u_vec(i) == 0           %repeated knots
            a = 0;
        else
            a = (u_val - u_vec(i))/(u_vec(i+k)-u_vec(i));
        end
        if u_vec(i+k+1) - u_vec(i+1) == 0       %repeated knots
            b = 0;
        else
            b = (u_vec(i+k+1) - u_val)/(u_vec(i+k+1) - u_vec(i+1));
        end
        basis = a * bSpline_Nik(u_val, i, k-1, u_vec) + b * bSpline_Nik(u_val, i+1, k-1, u_vec);
    end
end