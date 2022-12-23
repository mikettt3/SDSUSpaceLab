% Python sorta test code
% 12/18/2022
% Mapping YGE to Roboclaw motor drive

yawGyroErr = [-200:.1:200]';
mDrive = zeros(size(yawGyroErr));

for i=1:length(yawGyroErr)
    yGE = yawGyroErr(i);
    if yGE<= -150
        mDrive(i) = 0;
    elseif (yGE> -150) && (yGE<= -16)
        mDrive(i) = (yGE+150)*64/150;
    elseif (yGE> -16) && (yGE< 16)
        mDrive(i) = 64;
    elseif (yGE< 150) && (yGE>= 16)
        mDrive(i) = (yGE*64/150) + 63;
    elseif yGE>= 150
        mDrive(i) = 127;
    else
        disp('error');
    end
end

mDrive = round(mDrive);


plot(yawGyroErr, mDrive)
