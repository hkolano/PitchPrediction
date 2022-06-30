function train_toy_data(folder)
    fds = fileDatastore(folder,"ReadFcn",@read_to_array);

    data = readall(fds);
%     figure 
%     plot(data{1}(5,:))
%     hold on
    data = smooth_velocities(data);
%     plot(data{1}(5,:))

    % Preview the data
    % figure
    % tiledlayout(2,2)
    % for i = 1:4
    %     nexttile
    %     stackedplot(data{i})
    % 
    %     xlabel("Time Step")
    % end

    % Divide into train and test data
    numObservations = numel(data);
    idxTrain = 1:floor(0.9*numObservations);
    idxTest = floor(0.9*numObservations)+1:numObservations;
    dataTrain = data(idxTrain);
    dataTest = data(idxTest);

    numChannels = size(dataTrain{1}, 1);

    for n = 1:numel(dataTrain)
        X = dataTrain{n};
        XTrain{n} = X(:,1:end-1);
        TTrain{n} = X(2:end,2:end);
    end

    for n = 1:numel(dataTest)
        X = dataTest{n};
        XTest{n} = X(:,1:end-1);
        TTest{n} = X(2:end,2:end);
    end

    layers = [
        sequenceInputLayer(numChannels)
        lstmLayer(128, 'OutputMode', 'sequence')
        fullyConnectedLayer(6)
        regressionLayer];

    options = trainingOptions("adam", ...
        MaxEpochs=100, ...
        SequencePaddingDirection="left", ...
        Shuffle="every-epoch", ...
        Plots="training-progress", ...
        Verbose=0, ...
        ValidationData={XTest, TTest}, ...
        ValidationFrequency = 25);
    
    disp(XTrain{1})

%     net = trainNetwork(XTrain,TTrain,layers,options);

    function data = read_to_array(input)
        data = readtable(input);
        data = table2array(data)';
    end

    function data = smooth_velocities(data)
        for i = 1:numel(data)
            data{i}(5,:) = movmean(data{i}(5,:), 67);
            data{i}(6,:) = movmean(data{i}(6,:), 67);
            data{i}(7,:) = movmean(data{i}(7,:), 67);
        end
    end

%     outputFile = fullfile("data/networks/toy-nets", 'netv1.mat');
%     save(outputFile, 'net');
%     
%     outputFile2 = fullfile("data/networks/toy-nets", 'netv1testdata.mat');
%     save(outputFile2, 'XTest', 'TTest')

end

% Net v1:
%     layers = [
%         sequenceInputLayer(numChannels)
%         lstmLayer(128)
%         fullyConnectedLayer(1)
%         regressionLayer];

% Net v2:
%     layers = [
%         sequenceInputLayer(numChannels)
%         lstmLayer(128, 'OutputMode', 'sequence')
%         fullyConnectedLayer(1)
%         regressionLayer];