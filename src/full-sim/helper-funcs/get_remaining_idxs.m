%{
Removes indices of "elimd_gps" feature groups from list of inputs.

Inputs:
elimd_gps: list of strings of the group names to be eliminated from the
input pool. Ex: ["xyz_poses", "xyz_vels"]

Returns:
rem_idxs: list of indices of remaining feature groups. 
(1:41, minus the channels for the eliminated groups.)
%}
function rem_idxs = get_remaining_idxs(elimd_gps)
    load('data/channel_dict.mat', 'chan_idxs')
    chan_idxs = rmfield(chan_idxs, 'pitch');
    chan_idxs = rmfield(chan_idxs, 'dt');
    
    all_idxs = 1:1:41;

    for i = 1:length(elimd_gps)
        group_name = elimd_gps(i);
        all_idxs = all_idxs(~ismember(all_idcs, chan_idxs.(group_name)));
        chan_idxs = rmfield(chan_idxs, group_name);
    end
    rem_idxs = all_idxs;
end