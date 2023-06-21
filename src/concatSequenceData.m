function [x, y] = concatSequenceData(x, y)
x = padsequences(x,2);
y = onehotencode(cat(2,y{:}),1);
end