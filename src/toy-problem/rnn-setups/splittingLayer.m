% Written by Mahmoud Afifi -- mafifi@eecs.yorku.ca | m.3afifi@gmail.com
% MIT License
% Requires Matlab 2019b or higher
classdef splittingLayer < nnet.layer.Layer
    
    properties
        target 
    end
    
    properties (Learnable)
    end
    
    methods
        function layer = splittingLayer(name,target)
            layer.Name = name;
            layer.Description = "splittingLayer";
            layer.target = target;
        end
        function Z = predict(layer, X)
%             disp('Got values: \n')
%             disp(X)
            switch layer.target
                case '1st'
                    Z = X(1:6,:,:); 
                case '2nd' 
                    Z = X(7:end,:,:); 
            end
        end
    end
end