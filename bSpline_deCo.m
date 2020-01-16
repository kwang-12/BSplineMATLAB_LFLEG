function co = bSpline_deCo(choice, num, properties, U)
    switch choice
        case 'v'
            i = 1:properties.n;
            U = U(2:end-1);
            k = properties.k-1;
            co = (k+1)/(U(num+k+1) - U(num));
        case 'a'
            i = 1:properties.n-1;
            U = U(3:end-2);
            k = properties.k-2;
            co = (k+1)/(U(num+k+1) - U(num));
        case 'j'
            i = 1:properties.n-2;
            U = U(4:end-3);
            k = properties.k-3;
            co = (k+1)/(U(num+k+1) - U(num));
    end 
end