%{ 
Takes the full dataset and parses it for the k length study.
Inputs are sliced by input_idxs (certain feature groups are removed).
Ouput is a k x traj_length vector of pitches, where k is the number of steps to look
forward on the pitch.

Last modified 12/5/22

Inputs:
data: XTrain or XTest. Includes all data channels.
sf: "stretch factor" sf=1 --> predicts every 0.02s
k: number of steps ahead to predict. Usually 25.
input_idxs: list of indexes to include as feature inputs
pitch_idx: index for the pitch (of the entire original feature set, usually
23)

Ouptuts:
Input_Data: cell array similar to "data", but only including the features in
input_idxs, and ending at the last feasible prediction given sf and k
Resp_Data: cell array recording pitch responses from the inputs. Each time
step has 25 response values, equal to the pitch at k times in the future,
spread apart by 0.02s*sf
%}
function [Input_Data, Resp_Data, Pitch] = transform_data_for_stretch_study(data, sf, k, input_idxs, pitch_idx)
    short_trajs = [];
    for n = 1:numel(data)
        if size(data{n}, 2) > (sf+1)*k
            Input_Data{n} = data{n}(input_idxs, 1:end-sf*k);
            traj_len = size(Input_Data{n}, 2);
            responses = zeros(k, traj_len);
            for t = 1:traj_len
                responses(:,t) = data{n}(pitch_idx, t+sf:sf:t+sf*k)';
            end
            Resp_Data{n} = responses;
        else
            Input_Data{n} = [];
            Resp_Data{n} = [];
            short_trajs = [short_trajs n];
        end
    end

    for idx = length(short_trajs):-1:1
        n = short_trajs(idx);
        Resp_Data(n) = [];
        Input_Data(n) = [];
    end
end