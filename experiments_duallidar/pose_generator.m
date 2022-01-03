x_min =1.3;
x_max = 1.9;
y_min = 1.3;
y_max=1.9;
z_min=0.8;
z_max=1.1;
roll_min=-1.9;
roll_max=-0.9;
pitch_min=0.9;
pitch_max=0.9;
yaw_min=-4.5;
yaw_max=-3.5;

pose_num=38;
poses=[];

for idx=1:pose_num
    x = x_min+rand(1)*(x_max-x_min);
    y = y_min+rand(1)*(y_max-y_min);
    z = z_min+rand(1)*(z_max-z_min);
    roll = roll_min+rand(1)*(roll_max-roll_min);
    pitch = pitch_min+rand(1)*(pitch_max-pitch_min);
    yaw = yaw_min+rand(1)*(yaw_max-yaw_min);
    pose = [x,y,z,roll,pitch,yaw];
    poses=[poses;pose];
end

dlmwrite('/home/ramlab/Documents/publication/unifiedCali/data/simu/dual-lidar/pose.txt',...
            poses,'delimiter', ' ','precision','%6.3f');


