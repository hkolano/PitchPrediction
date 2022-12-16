resetState(net)
tic
[new_net, Z] = predictAndUpdateState(net, XTest{155}(:,1:200));
toc