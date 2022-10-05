function data_o = f_pose_reader(file_path)
fid0 = fopen(file_path,'r');
data_o = [];
idx=1;
while true
    sub_data = [];
    a{idx} = fgetl(fid0);
    if ~ischar(a{idx})
        break;
    end
    x = split(a{idx},' ');
    for idx2=1:size(x,1)
        sub_data = [sub_data,str2num(x{idx2})];
    end
    data_o = [data_o;sub_data];
    idx = idx + 1;
end
fclose(fid0);
end