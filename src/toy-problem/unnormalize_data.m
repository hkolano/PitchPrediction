function data = unnormalize_data(data, params)
    mu = params.mu;
    sig = params.sig;
    
    for i = 1:numel(data)
%         size(data{i})
        data{i}(1:6,:) = data{i}(1:6,:).*sig + mu;
    end
    
end