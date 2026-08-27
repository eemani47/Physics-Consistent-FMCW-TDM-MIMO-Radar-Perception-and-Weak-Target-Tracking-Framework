function [clean,info] = interference_mitigator(x,p)
%INTERFERENCE_MITIGATOR  Median-background STFT ridge suppression.
%
%   First-stage mutual-interference removal, applied per receive chain before
%   range compression. It exploits the structural asymmetry between wanted and
%   unwanted energy after dechirping: a target beat is a tone that holds its
%   frequency for the whole chirp, whereas an uncoordinated FMCW emitter sweeps
%   at the difference of the two slopes and traces a diagonal ridge in the
%   time-frequency plane.
%
%   For each chirp the signal is transformed by a hopped short-time Fourier
%   transform. A per-frequency median across STFT frames is a robust estimate
%   of the stationary background, because a sweeping ridge occupies any given
%   frequency for only a few frames. Cells whose power exceeds that background
%   by the configured ratio become candidates; a candidate is kept only if
%   neighbouring frames also contain candidates within a small frequency
%   offset, which enforces ridge continuity and rejects isolated spikes. The
%   surviving mask is dilated along STFT time, zeroed, and the chirp is
%   rebuilt by weighted overlap-add.
%
%   Two safety conditions bound the damage the mask can do: the masked
%   fraction of the plane and the ratio of output to input power. If either is
%   violated the output is blended back toward the input, so an unusual scene
%   degrades the measurement gracefully instead of erasing it.

[Nr,Nd] = size(x);
L = min(64,2^floor(log2(max(32,min(Nr,128)))));
L = max(L,32);
H = max(8,floor(L/2));
Nfft = L;
w = 0.5 - 0.5*cos(2*pi*(0:L-1)'/max(L-1,1));
M = 1 + floor((Nr-L)/H);

info = struct('mask_fraction',0,'detected_bins',0,'stft_length',L,'stft_hop',H, ...
    'frames',M,'suppression_proxy_db',0,'safety_blend',1,'input_power',0,'output_power',0);
if M < 3
clean = x;
return;
end

thr        = get_default_field(p.interference,'mask_threshold',7);
maskRadius = round(get_default_field(p.interference,'mask_radius',2));
minOutFrac = get_default_field(p.interference,'median_min_output_fraction',0.45);
maxMaskFrac= get_default_field(p.interference,'median_max_mask_fraction',0.18);
blendAmt   = get_default_field(p.interference,'median_safety_blend',0.35);

clean = complex(zeros(size(x)));
mask_total = 0;
total_bins = Nfft*M*Nd;
suppression_num = 0;
suppression_den = 0;

for d = 1:Nd
    Y = complex(zeros(Nfft,M));
    for m = 1:M
        idx = (1:L) + (m-1)*H;
        Y(:,m) = fft(x(idx,d).*w,Nfft);
    end
    P  = abs(Y).^2;
    bg = median(P,2) + realmin;
    % Robust noise-floor estimate for masked-bin replacement.
    bgSorted = sort(bg(isfinite(bg) & bg > 0));
    if isempty(bgSorted)
        fillLevel = realmin;
    else
        fillLevel = sqrt(bgSorted(max(1,round(0.20*numel(bgSorted)))));
    end
    cand = (P./bg) > thr;

    mask = false(Nfft,M);
    for m = 2:M-1
        idx = find(cand(:,m));
        for q = 1:numel(idx)
            k = idx(q);
if k <= 2 || k >= Nfft-1, continue;
end
            lo = max(1,k-3); hi = min(Nfft,k+3);
            if any(cand(lo:hi,m-1)) && any(cand(lo:hi,m+1))
                mask(max(1,k-maskRadius):min(Nfft,k+maskRadius),m) = true;
            end
        end
    end
    if M > 2
        mask(:,2:end-1) = mask(:,2:end-1) | mask(:,1:end-2) | mask(:,3:end);
    end

    suppression_num = suppression_num + sum(P(mask));
    suppression_den = suppression_den + sum(P(:));
    mask_total = mask_total + nnz(mask);

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
    clean(:,d) = out./max(acc,1e-8);
end

info.mask_fraction = mask_total/max(total_bins,1);
info.detected_bins = mask_total;
if suppression_num > 0
    info.suppression_proxy_db = 10*log10(max(suppression_den,eps)/max(suppression_den-suppression_num,eps));
end

Pin  = mean(abs(x(:)).^2) + eps;
Pout = mean(abs(clean(:)).^2) + eps;
info.input_power = Pin;
info.output_power = Pout;
if Pout < minOutFrac*Pin || info.mask_fraction > maxMaskFrac
    clean = (1-blendAmt)*x + blendAmt*clean;
info.safety_blend = blendAmt;
    info.output_power = mean(abs(clean(:)).^2);
end
end
