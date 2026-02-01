function [ ind ] = JointName2Index( jointName, jointStateMsg )
% computes the index in a joint state message that corresponds to a joint name
ind=0;
for i=1:length(jointStateMsg.name)
    if strcmp(jointName,jointStateMsg.name{i})
        ind=i;
        break;
    end
end
end

