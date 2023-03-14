
load("data\networks\full-nets\simple_network_unit_testing\unittesting_384units_take1_learningdrop.mat");
resetState(net);
k = 25;
param_names = ["arm-added-mass", "arm-linear-drag", "vehicle-added-mass", "vehicle-linear-drag", "vehicle-quadratic-drag"];

rmses = zeros(5, 10);

for param_id = 1:length(param_names)

    param = param_names(param_id);

    for model_num = 1:10
        clear XData
        load(strcat("data\full-sim-with-hydro\single-model-50percent\", param, "\model", string(model_num), "_data_18chan.mat"));
        
        for n = 1:numel(XData)
            resp = zeros(k, size(XData{n}, 2)-k);
            if size(XData{n}, 2) > 1.1*k
                for t = 1:size(XData{n}, 2)-k
                    resp(:,t) = XData{n}(pitch_idx, t+1:t+k)';
                end
                Resp{n} = resp;
                Inputs{n} = [XData{n}(:,1:end-k); repmat(0.02, 1, size(XData{n},2)-k)];
            end
        end
        
        resetState(net);
        [net, Z] = predictAndUpdateState(net, Inputs, "ExecutionEnvironment","auto");
        
        RMSEs = zeros(1, numel(XData));
        
        for n = 1:numel(XData)
            RMSEs(n) = sqrt(immse(single(Resp{n}), Z{n})); 
        end
    
        rmses(param_id, model_num) = mean(RMSEs);
    end
end

disp(rmses)

%%
% load("data\full-data-matlab\channel_subgroups\no_goal_poses\no_manip_vels\no_goal_vels\data_without_xyz_poses.mat");
% load("data\networks\full-nets\simple_network_unit_testing\unittesting_384units_take1_learningdrop.mat");
%         
% for n = 1:numel(XTest)
%     resp = zeros(k, size(XTest{n}, 2)-k);
%     if size(XTest{n}, 2) > 1.1*k
%         for t = 1:size(XTest{n}, 2)-k
%             resp(:,t) = XTest{n}(pitch_idx, t+1:t+k)';
%         end
%         Resp{n} = resp;
%         Inputs{n} = XTest{n}(:,1:end-k);
%     end
% end
%         
% resetState(net);
% [net, Z] = predictAndUpdateState(net, Inputs, "ExecutionEnvironment","auto");
% 
% RMSEs = zeros(1, numel(XTest));
% 
% for n = 1:numel(XTest)
%     RMSEs(n) = sqrt(immse(single(Resp{n}), Z{n})); 
% end
% 
% mean(RMSEs)

