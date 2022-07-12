function [lgraph, options] = setup_recurrent_gru(numChannels, XTest, TTest)
    % Initialize layer graph object
    
    lgraph = layerGraph();
    
    split_1st = splittingLayer('Splitting-1st','1st');
    split_2nd = splittingLayer('Splitting-2nd','2nd');
    
    %% Build out network layers
    % Top: input sequence layer
    tempLayers = sequenceInputLayer(15,"Name","State Input");
    lgraph = addLayers(lgraph,tempLayers);

    % Right side: forward the constant values
    lgraph = addLayers(lgraph,split_2nd);
    lgraph = connectLayers(lgraph,"State Input","Splitting-2nd");
    
    % Left side: LSTM on the sequence data
    tempLayers = [
        split_1st
        gruLayer(128,"Name","GRU", 'OutputMode', 'sequence')];
    lgraph = addLayers(lgraph,tempLayers);
    lgraph = connectLayers(lgraph,"State Input","Splitting-1st");

    % Final stretch: FCN on LSTM output and consts
    tempLayers = [
        concatenationLayer(1,2,"Name","concat")
        fullyConnectedLayer(6,"Name","fc")];
    lgraph = addLayers(lgraph,tempLayers);
    lgraph = connectLayers(lgraph,"Splitting-2nd","concat/in2");
    lgraph = connectLayers(lgraph,"GRU","concat/in1");
    
    tempLayers = [
        additionLayer(2,"Name","addition")
        regressionLayer("Name","regressionoutput")];
    lgraph = addLayers(lgraph, tempLayers);
    lgraph = connectLayers(lgraph, "Splitting-1st", "addition/in2");
    lgraph = connectLayers(lgraph, "fc", "addition/in1");
    

    % clean up helper variable
    clear tempLayers;

    % Make all the connections
    
    options = trainingOptions("adam", ...
        MaxEpochs=100, ...
        SequencePaddingDirection="left", ...
        Shuffle="every-epoch", ...
        Plots="training-progress", ...
        Verbose=0, ...
        ValidationData={XTest, TTest}, ...
        ValidationFrequency = 25);
end