% Written by Mahmoud Afifi -- mafifi@eecs.yorku.ca | m.3afifi@gmail.com
% MIT License
% Requires Matlab 2019b or higher
classdef splittingLayer < nnet.layer.Layer
    
    properties
        target 
        n_inputs
    end
    
    properties (Learnable)
    end
    
    methods
        function layer = splittingLayer(name,target,num_inputs)
            layer.Name = name;
            layer.Description = "splittingLayer";
            layer.target = target;
            layer.n_inputs = num_inputs;
        end
        function Z = predict(layer, X)
%             disp('Got values: \n')
%             disp(X)
            switch layer.target
                case '1st'
                    Z = X(1:layer.n_inputs,:,:); 
                case '2nd' 
                    Z = X(layer.n_inputs+1:end,:,:); 
            end
        end
    end
end