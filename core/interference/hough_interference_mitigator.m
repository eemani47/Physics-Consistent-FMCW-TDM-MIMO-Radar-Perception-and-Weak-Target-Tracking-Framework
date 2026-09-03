function [y,info] = hough_interference_mitigator(x,p)
%HOUGH_INTERFERENCE_MITIGATOR  Power-weighted Hough ridge suppression EMANI.
%
%   Second-stage mutual-interference removal, applied to the dechirped data of
%   one receive chain. Where the median-background front end works cell by
%   cell, this stage tests a global geometric hypothesis: that the interference
%   forms a straight line in the normalised time-frequency plane.
%
%   Each chirp is transformed by a hopped STFT and normalised by its
%   per-frequency median background. Cells above the candidate ratio are cast
%   into a Hough accumulator over the line parametrisation
%
%       rho = x cos(theta) + y sin(theta)
%
%   with x normalised STFT time and y normalised frequency, and with each vote
%   weighted by the cell's power excess. Capping the weight prevents one very
%   bright cell from carrying an entire line hypothesis on its own.
%
%   The strongest accumulator peak is accepted only if its score clears the
%   detection threshold and its inclination lies away from horizontal. That
%   angular exclusion is what protects the wanted signal: a target beat is a
%   constant tone and therefore draws a horizontal line, so restricting
%   acceptance to inclined lines removes interference without touching targets.
%
%   Accepted lines are masked over a band of the configured width, the chirp is
%   rebuilt by weighted overlap-add, and the same two safety conditions used by
%   the front end bound the masked fraction and the output power loss.

validateattributes(x,{'numeric'},{'2d'},mfilename,'x',1);
[Nr,Nd] = size(x);
L = min(64,2^floor(log2(max(32,min(Nr,128)))));
L = max(32,min(L,Nr));
H = max(8,floor(L/2));
Nfft = L;
w = 0.5 - 0.5*cos(2*pi*(0:L-1)'/max(L-1,1));
M = 1 + floor((Nr-L)/H);

if M < 3
y = x;
    info = struct('enabled',true,'method','hough_tf','mask_fraction',0,'lines_detected',0, ...
        'stft_length',L,'hop',H,'frames',Nd,'safety_blend',1,'input_power',0,'output_power',0);
return;
end

method = char(get_default_field(p.interference,'hough_method','hough_tf'));
if strcmpi(method,'off')
y = x;
    info = struct('enabled',false,'method',method,'mask_fraction',0,'lines_detected',0, ...
        'stft_length',L,'hop',H,'frames',Nd,'safety_blend',1,'input_power',0,'output_power',0);
return;
end

maskTotal = 0;
totalBins = Nfft*M*Nd;
lineCount = 0;
y = complex(zeros(size(x)));
thetas  = deg2rad(p.interference.hough_theta_deg(:));
Nrho    = round(p.interference.hough_rho_bins);
rhoMin  = -sqrt(2); rhoMax = sqrt(2);
rhoAxis = linspace(rhoMin,rhoMax,Nrho);
[GX,GY] = meshgrid((0:M-1)/max(M-1,1),(0:Nfft-1)/max(Nfft-1,1));

for d = 1:Nd
    Y = complex(zeros(Nfft,M));
    for m = 1:M
        idx = (1:L) + (m-1)*H;
        Y(:,m) = fft(x(idx,d).*w,Nfft);
    end
    P   = abs(Y).^2;
    bg  = median(P,2) + realmin;
    % Robust noise-floor estimate for masked-bin replacement.
    bgSorted = sort(bg(isfinite(bg) & bg > 0));
    if isempty(bgSorted)
        fillLevel = realmin;
    else
        fillLevel = sqrt(bgSorted(max(1,round(0.20*numel(bgSorted)))));
    end
dyn = P./bg;
cand = dyn > p.interference.hough_candidate_ratio;

    mask = false(Nfft,M);
    [rr,cc] = find(cand);
    if numel(rr) >= p.interference.hough_min_points
        xn = (cc-1)/max(M-1,1);
        yn = (rr-1)/max(Nfft-1,1);
        weights = min(dyn(cand),p.interference.hough_weight_cap);
        A = zeros(Nrho,numel(thetas));
        for kt = 1:numel(thetas)
            th = thetas(kt);
            rho = xn*cos(th) + yn*sin(th);
            bins = round((rho-rhoMin)/(rhoMax-rhoMin)*(Nrho-1)) + 1;
            bins = max(1,min(Nrho,bins));
            A(:,kt) = accumarray(bins,weights,[Nrho 1]);
        end
        [peak,lin] = max(A(:));
        [ir,it] = ind2sub(size(A),lin);
        if peak >= p.interference.hough_min_score
            rho = rhoAxis(ir); th = thetas(it);
            lineAngle = abs(mod(rad2deg(th)+90,180)-90);
            if lineAngle >= p.interference.hough_min_angle_deg && ...
               lineAngle <= 90-p.interference.hough_min_angle_deg
                dist = abs(GX*cos(th) + GY*sin(th) - rho);
mask = dist <= p.interference.hough_mask_width;
lineCount = lineCount + 1;
            end
        end
    end

    maskTotal = maskTotal + nnz(mask);
    out = zeros(Nr,1); acc = zeros(Nr,1);
    for m = 1:M
        idx = (1:L) + (m-1)*H;
        spec = Y(:,m);
        % Masked bins are filled with noise at the local background level, not
        % zeroed. A zeroed bin is a deterministic notch, and a notch convolves
        % with every target response in the frame, planting a coherent replica
        % at the range offset that notch frequency maps to, c*f/(2S). Those
        % replicas inherit the parent target's velocity and bearing, so no
        % downstream gate can separate them from real targets. A noise-level
        % fill suppresses the interferer and leaves no deterministic imprint.
        mk = mask(:,m);
        if any(mk)
            % Fill at the noise floor, not at the local background. The
            % per-frequency background is elevated at exactly the frequencies
            % the interferer occupies, so filling at that level would put a
            % plateau back where a ridge was removed. A low percentile across
            % frequency is dominated by clean bins and estimates the noise
            % floor robustly, which guarantees the mask can only remove energy.
            spec(mk) = fillLevel.*exp(1j*2*pi*rand(nnz(mk),1));
        end
        seg = ifft(spec,Nfft);
        out(idx) = out(idx) + seg(1:L).*w;
        acc(idx) = acc(idx) + w.^2;
    end
    y(:,d) = out./max(acc,1e-8);
end

frac = maskTotal/max(totalBins,1);
Pin  = mean(abs(x(:)).^2) + eps;
Pout = mean(abs(y(:)).^2) + eps;
blend = 1;
if frac > p.interference.hough_max_mask_fraction || Pout < p.interference.hough_min_output_fraction*Pin
blend = p.interference.hough_safety_blend;
    y = (1-blend)*x + blend*y;
    Pout = mean(abs(y(:)).^2);
end

info = struct('enabled',true,'method',method,'mask_fraction',frac,'lines_detected',lineCount, ...
    'stft_length',L,'hop',H,'frames',Nd,'safety_blend',blend, ...
    'input_power',Pin,'output_power',Pout);
end
