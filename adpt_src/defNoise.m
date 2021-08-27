function esym = defNoise(m, tsym, ampl, fNum, xInitNum)
% >> This function generates default exploration noise
% signals. Every noise signal is composed of sinusoidal
% signals with several different frequencies.
%
% e(t) = [e_1(t);
%         e_2(t);
%         ...;
%         e_xInitNum(t)].
%
% For i = 1, 2, ..., xInitNum,
% e_i(t) = [e_{i1}(t);
%           e_{i2}(t);
%           ...;
%           e_{im}(t)],
% where m is control dimension.
%
% For i = 1, ..., xInitNum, and j = 1, ..., m,
% e_{ij}(t) = \sum_{k=1}^{fNum} sin(omega_{ijk}*t).
%
% >> esym = defNoise(m, tsym, ampl, fNum, xInitNum)
% ======================== Input =========================
% m:    control dimension
% tsym: time
%       (symbolic representation)
% ampl: amplitude of sinusoidal signals
% fNum: number of frequencies
% ========================================================
% ======================== Output ========================
% esym: default exploration noise signals
%       (symbolic representation)
%       size(esym) = (xInitNum * m) * 1.
%       Each e_{ij}(t) is different from others.
% ========================================================

rNum = round(fNum/2); % number of rational omegas
iNum = fNum - rNum; % number of irrational omegas

% rational omega candidates
rPool = [1.1, 1.9, 3, 5, 7, 7.9, 11, 13, 17];
rPoolNum = numel(rPool);
% irrational omega candidates
iPool = [sqrt(3), sqrt(5), sqrt(7), sqrt(11),...
         sqrt(13), sqrt(15), sqrt(17), 3*pi, 7*pi];
iPoolNum = numel(iPool);

idx = zeros(m*xInitNum,fNum);

for i = 1:m*xInitNum
    % Generate indexes of omegas randomly.
    rIdx = randperm(rPoolNum,rNum);
    rIdx = sort(rIdx);
    iIdx = randperm(iPoolNum,iNum);
    iIdx = sort(iIdx);
    if i == 1
        idx(i,:) = [rIdx, iIdx];
    else
        % Ensure the uniqueness of each row.
        flag = 1;
        while flag == 1
            for j = 1:i-1
                if isequal(idx(j,1:rNum),rIdx) ||...
                   isequal(idx(j,rNum+1:end),iIdx)
                    flag = 1;
                    break
                else
                    flag = 0;
                end
            end
            % Generate indexes of omegas again.
            if flag == 1
                rIdx = randperm(rPoolNum,rNum);
                rIdx = sort(rIdx);
                iIdx = randperm(iPoolNum,iNum);
                iIdx = sort(iIdx);
            end
        end
        idx(i,:) = [rIdx, iIdx];
    end
end

omega = [rPool(idx(:,1:rNum)), iPool(idx(:,rNum+1:end))];
esym = ampl.*sum(sin(omega*tsym),2);
end