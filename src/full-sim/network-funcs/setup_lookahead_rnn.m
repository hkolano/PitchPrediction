function lgraph = setup_lookahead_rnn(num_channels, num_lookahead)
    % Initialize layer graph object
    layers = [
        sequenceInputLayer(num_channels,"Name","sequence")
        gruLayer(512,"Name","gru")
        fullyConnectedLayer(num_lookahead,"Name","fc")
        regressionLayer("Name","regressionoutput")];
    
    lgraph = layerGraph(layers);
end