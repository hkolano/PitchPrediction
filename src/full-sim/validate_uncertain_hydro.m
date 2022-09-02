load("data\full-sim-with-hydro\single-model-1percent\arm-added-mass\model1_data_18chan.mat");
load("data\networks\full-nets\simple_network_unit_testing\unittesting_256units_take1_learningdrop.mat");

k = 25;

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