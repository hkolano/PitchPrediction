
load("data\networks\full-nets\simple_network_unit_testing\unittesting_256units_take1_learningdrop.mat");

k = 25;
param_names = ["arm-added-mass", "arm-linear-drag", "vehicle-added-mass", "vehicle-linear-drag", "vehicle-quadratic-drag"];

rmses = zeros(5, 3);

for param_id = 1:length(param_names)

    param = param_names(param_id);

    for model_num = 1:3
    
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
        
        resetState(net)
        [net, Z] = predictAndUpdateState(net, Inputs, "ExecutionEnvironment","auto");
        
        RMSEs = zeros(1, numel(XData));
        
        for n = 1:numel(XData)
            RMSEs(n) = sqrt(immse(single(Resp{n}), Z{n})); 
        end
    
        rmses(param_id, model_num) = mean(RMSEs);
    end
end

disp(rmses)

