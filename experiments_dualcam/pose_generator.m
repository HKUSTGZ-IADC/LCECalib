x_min =1.8;
x_max = 2.2;
y_min = -0.8;
y_max=1;
z_min=0.8;
z_max=1.3;
roll_min=-0.5;
roll_max=0.5;
pitch_min=1;
pitch_max=2;
yaw_min=-0.4;
yaw_max=0.6;

pose_num=37;
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

dlmwrite('/home/ramlab/Documents/publication/unifiedCali/data/simu/dual-camera/pose.txt',...
            poses,'delimiter', ' ','precision','%6.3f');


