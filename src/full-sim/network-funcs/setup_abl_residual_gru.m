function lgraph = setup_abl_residual_gru(num_channels)
    % Initialize layer graph object
    
    lgraph = layerGraph();
    
    % Top: input sequence layer
    tempLayers = [sequenceInputLayer(num_channels,"Name","State Input")
        gruLayer(384,"Name","GRU", 'OutputMode', 'sequence')
        fullyConnectedLayer(num_channels, "Name", "fc")];

    lgraph = addLayers(lgraph,tempLayers);

    % End: addition/output layers
    tempLayers = [additionLayer(2, "Name", "addition")
        regressionLayer("Name", "regressionoutput")];

    lgraph = addLayers(lgraph, tempLayers);

    % Make connections
    lgraph = connectLayers(lgraph, "fc", "addition/in1");
    lgraph = connectLayers(lgraph, "State Input", "addition/in2");    

    % clean up helper variable
    clear tempLayers;
end