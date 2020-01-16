function co = bSpline_deBaCo(choice, num, val, properties, U)
    switch choice
        case 'v'
            co = bSpline_Nik(val, num, properties.k-1, U(2:end-1));
        case 'a'
            co = bSpline_Nik(val, num, properties.k-2, U(3:end-2));
        case 'j'
            co = bSpline_Nik(val, num, properties.k-3, U(4:end-3));
    end
end