[XTrain,YTrain] = japaneseVowelsTrainData;
miniBatchSize = 27;
fmt = 'CTB';
dsXTrain = arrayDatastore( XTrain, 'IterationDimension', 1, 'OutputType', 'same' );
dsYTrain = arrayDatastore( YTrain, 'IterationDimension', 1 );   
dsTrain = combine( dsXTrain, dsYTrain );
% setup the batches
mbqTrain = minibatchqueue(  dsTrain, 2, ...
                          'MiniBatchSize', miniBatchSize, ...
                          'PartialMiniBatch', 'discard', ...
                          'MiniBatchFcn', @concatSequenceData, ...
                          'MiniBatchFormat', {fmt, 'CB'});
function [x, y] = concatSequenceData(x, y)
x = padsequences(x,2);
y = onehotencode(cat(2,y{:}),1);
end