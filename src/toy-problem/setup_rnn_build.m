function [lgraph, options] = setup_rnn_build(numChannels, XTest, TTest)
    % Initialize layer graph object
    
    lgraph = layerGraph();
    
    split_1st = splittingLayer('Splitting-1st','1st');
    split_2nd = splittingLayer('Splitting-2nd','2nd');
    
    %% Build out network layers
    % Top: input sequence layer
    tempLayers = sequenceInputLayer(numChannels+8,"Name","State Input");
    lgraph = addLayers(lgraph,tempLayers);

    % Right side: forward the constant values
    lgraph = addLayers(lgraph,split_2nd);

    % Left side: LSTM on the sequence data
    tempLayers = [
        split_1st
        lstmLayer(128,"Name","LSTM", 'OutputMode', 'sequence')];
    lgraph = addLayers(lgraph,tempLayers);

    % Final stretch: FCN on LSTM output and consts
    tempLayers = [
%         concatenationLayer(1,2,"Name","concat")
        fullyConnectedLayer(6,"Name","fc")
        regressionLayer("Name","regressionoutput")];
    lgraph = addLayers(lgraph,tempLayers);

    % clean up helper variable
    clear tempLayers;

    % Make all the connections
    lgraph = connectLayers(lgraph,"State Input","Splitting-2nd");
    lgraph = connectLayers(lgraph,"State Input","Splitting-1st");
%     lgraph = connectLayers(lgraph,"Splitting-2nd","concat/in2");
%     lgraph = connectLayers(lgraph,"LSTM","concat/in1");
%     lgraph = connectLayers(lgraph, "State Input", "LSTM")
    lgraph = connectLayers(lgraph, "LSTM", "fc");
    
    options = trainingOptions("adam", ...
        MaxEpochs=100, ...
        SequencePaddingDirection="left", ...
        Shuffle="every-epoch", ...
        Plots="training-progress", ...
        Verbose=0, ...
        ValidationData={XTest, TTest}, ...
        ValidationFrequency = 25);
end