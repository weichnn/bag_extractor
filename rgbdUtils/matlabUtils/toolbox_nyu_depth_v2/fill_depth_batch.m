% input: root_dir, list_file
file_sync_list = strcat(root_dir, list_file);
output_dir = dense_dir;
overlap_dir = overlap_dir;
disp(file_sync_list)
disp(root_dir)
disp(dense_dir)
disp(overlap_dir)

mkdir(output_dir)
mkdir(overlap_dir)
fid = fopen(file_sync_list,'r');

file_names = textscan(fid,'%s%s');
color = string(file_names{1});
depth = string(file_names{2});

[rows, cols] = size(color);

for i = 1:rows
    color_name = color(i);
    depth_name = depth(i);
    
    names = split(color_name, '/');
    name = names(2);
    disp(name);
    disp(i);
    
    color_path = strcat(root_dir, color_name);
    depth_path = strcat(root_dir, depth_name);
    
    imgRGB = imread(color_path);
    imgDepth_raw = imread(depth_path);

    imgDepth = double(imgDepth_raw);
    coeff = 1.0;
    imgDepth = imgDepth / coeff;

    overlay = get_rgb_depth_overlay(imgRGB, imgDepth);
    imwrite(overlay, strcat(overlap_dir, name));
    
    imgDepth_dense = fill_depth_colorization(im2double((imgRGB)), (imgDepth));
    imgDepth_dense = uint16(imgDepth_dense * coeff);
    imwrite(imgDepth_dense, strcat(output_dir, name));

end
return;


