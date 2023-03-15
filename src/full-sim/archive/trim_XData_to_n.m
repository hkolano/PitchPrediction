function trimmed_XData = trim_XData_to_n(XData, val_ns)

for traj_idx = 1:numel(XData)
    XDat = XData{traj_idx};
    this_n = val_ns(traj_idx);
    trimmed_XData{traj_idx} = XDat(:,1:this_n);
end
    
trimmed_XData = trimmed_XData';

end