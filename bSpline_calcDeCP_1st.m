function deCP = bSpline_calcDeCP_1st(CP, properties, U)
    for i = 1:size(CP,1)-1
        deCP(i) = bSpline_deCo('v', i, properties, U) * (CP(i+1) - CP(i));
    end
    deCP = deCP';
end