function [all_Z, net_state] = minibatch_forecast(net, XTest, ns, k, mbs, is_training)

% NET_STATE IS THE VERY END STATE
% NOT THE STATE AFTER N
% THIS IS DOWNRIGHT TRAGIC
net = resetState(net);
sorted_ns = sort(ns, 'ascend')
all_Z = dlarray(zeros(17, mbs, max(ns)+k-1), 'CBT');
current_tstep = 1;
forecasting_batch_idxs = [];
mb_num = 1;
next_stop_tstep = sorted_ns(mb_num);

% if is_training == false
    while mb_num < mbs
        [Z, net_state] = predict(net, XTest(:,:,current_tstep:next_stop_tstep), "Acceleration", 'none');
        all_Z(:,:,current_tstep:next_stop_tstep) = Z;
        net.State = net_state;
        current_tstep = next_stop_tstep;
        this_batch_id = find(ns==current_tstep);
        mb_num = mb_num + length(this_batch_id);
%         forecasting_batch_idxs = [forecasting_batch_idxs, this_batch_id];
        next_stop_tstep = sorted_ns(mb_num);
        for i = 1:k-1
            if current_tstep+i > next_stop_tstep
                disp("EDGE CASE!!! FIX THIS!!")
            else
                XTest(:,this_batch_id,current_tstep+i) = all_Z(:,this_batch_id, current_tstep);
                [Z, net_state] = predict(net, XTest(:,:,current_tstep+i));
                all_Z(:,:,current_tstep+i) = Z;
                net.State = net_state;
            end
        end
        current_tstep = current_tstep+k
    end
% else
%     [Z, net_state] = forward(net, XTest);
% end
% net.State = net_state;
% first_net_state = net_state;
% Z_og = extractdata(Z);
% 
% % Setup forecast arrays
% last_preds = zeros(17, mbs);
% all_preds = zeros(17, mbs, k-1);
% 
% % Get last prediction (at n)
% for i = 1:mbs
%     last_preds(:,i,1) = Z_og(:,i,ns(i));
% %     if size(Z_og(:,i,:),3) > ns(i)
% %         Z_og(:,i,ns(i)+1:end) = zeros(17, 1, size(Z_og,3)-ns(i));
% %     end
% end
% 
% % all_preds(:,:,1) = last_preds;
% last_preds_dl = dlarray(last_preds, 'CBT');
% for i = 1:k-1
%     if is_training == false
%         [output, net_state] = predict(net, last_preds_dl);
%     else
%         [output, net_state] = forward(net, last_preds_dl);
%     end
%     net.State = net_state;
%     last_preds_dl = output;
%     last_preds = extractdata(last_preds_dl);
%     all_preds(:,:,i) = last_preds;
% end
% 
% space_for_forecasts = zeros(17,mbs,k-1);
% Z_og = cat(3, Z_og, space_for_forecasts);
% for i = 1:mbs
%     Z_og(:,i,ns(i)+1:ns(i)+k-1) = all_preds(:,i,:);
% end
% 
% Z_dl = dlarray(Z_og, 'CBT');
    
end