function [HFER, OA, zcr, DE] = fftresponse(u_vec,n,thresold,linetype,linewidth,legend,filename,newfig)
%% fftreponse will plot the fft of force function
% inputs:
% u_vec = force vector must be of size (:,2)
% n = sampling rate
% linetype = -k, --b etc 
% linewidth = thiknes of line 2 3 4 like
% legend  =  'display name'
% file name = 'A char of file name'
% Newfig = ture/false
% outpur
% HFER = High Frequncy Energy Ratio (higher value higher chattering)
% OA = oscilation Amplitude (higher value stronger chattering)
% zcr = zero crossing rate higher = more oscillations
% DE = derivative energy 9measures rapid varition
%
%



global basepath

% if size(u_vec,2) ~= 2 
%     error("u must be matrix of two column")
% end
% 
% if mod((length(u_vec)-1),n) ~= 0
%     error('must be choosen in such way it divide the u_vector properly')
% end
% 
% 
% 
% u = zeros(length(u_vec),1);
% u1 = zeros(length(u_vec),1);
% for i =1:length(u_vec)
%     u1(i) = sqrt(2*u_vec(i,1)^2+u_vec(i,2)^2);
%     u(i) = 2*u_vec(i,1)+u_vec(i,2);
% end
u_ref = u_vec(1:n:end);
u_ref = u_ref - mean(u_ref);
FS = 1000/n;

N = length(u_ref);

U = fft(u_ref);
p = (abs(U).^2)/N;

p = p(1:N/2+1);
p(2:end-1) = 2*p(2:end-1);

f = FS*(0:(N/2))/N;

%% plotiing fft
% if newfig == true
%     figure()
%     plotReducedSignal(f,p,N,linetype,linewidth,'Frequency (Hz)','Amplitude',legend,filename);
%     pause(5)
%     close()
% else
%     openfig(fullfile(basepath,[char(filename),'.fig']),"reuse");
%     plotReducedSignal(f,p,N,linetype,linewidth,'Frequency (Hz)','Amplitude',legend,filename);
% end


OA = std(u_ref);  %% oscilatiopn amplitude  higher value more chattring


eps_val = 1e-6;
u_clean = u_vec;
u_clean(abs(u_clean)<eps_val) = 0;
zcr = sum(abs(diff(sign(u_clean))))/(2*length(u_clean)); %Zero crossing: 
du = diff(u_vec);
DE = mean(du.^2); % Derivative energy


idx = f>thresold;

HF = sum(p(idx));
tot = sum(p);
HFER = HF/tot;

end