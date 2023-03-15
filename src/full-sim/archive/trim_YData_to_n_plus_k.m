function trimmed_YData = trim_YData_to_n_plus_k(YData, val_ns, k)

for traj_idx = 1:numel(YData)
    YDat = YData{traj_idx};
    this_n = val_ns(traj_idx);
    trimmed_YData{traj_idx} = YDat(:,1:this_n+k-1);
end
    
trimmed_YData = trimmed_YData';

end