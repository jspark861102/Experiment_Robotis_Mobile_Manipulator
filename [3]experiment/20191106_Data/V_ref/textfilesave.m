function textfilesave(name,data)
    file_output = fopen(name,'w');
    for i = 1 : length(data)
        fprintf(file_output, '%f\n', data(i));
    end
    fclose(file_output);
