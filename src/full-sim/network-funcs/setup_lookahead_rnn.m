function lgraph = setup_lookahead_rnn(num_channels, num_lookahead, num_units)
    % Initialize layer graph object
    layers = [
        sequenceInputLayer(num_channels,"Name","sequence")
        gruLayer(num_units,"Name","gru")
        fullyConnectedLayer(num_lookahead,"Name","fc")
        regressionLayer("Name","regressionoutput")];
    
    lgraph = layerGraph(layers);
end