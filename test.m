clc,clear

d = [];
fail_flag = 'fail';
%iterate 10 times
episodes = 10;

for n = 1:episodes
    %make random road
    MakeRoad();
    DesignProblem04('Controller_tianyul2','datafile','test.mat','display',false);
    
    %load data
    load('test.mat');
    if(processdata.result(end) == 0)
        fail_flag
        d = [d 0]
        return;
    end
    %get distance
    d = [d processdata.t(end)]
end

mean(d)
