function ticks = findTimeTick(desired_time, input_time_vector)
    ticks = zeros(1, length(desired_time));
    for i = 1:length(desired_time)
        timeVal = desired_time(i);
        for j = 1:length(input_time_vector)
            if round(input_time_vector(j),4) == round(desired_time(i),4)
                ticks(i) = j;
            end
        end
    end
    ticks = ticks;
end