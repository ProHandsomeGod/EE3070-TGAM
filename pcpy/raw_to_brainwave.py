import json, numpy as np
from scipy.signal import welch, butter, filtfilt, iirnotch

FS = 512  # TGAM raw sample rate (512)
WINDOW_SEC = 2
STEP_SEC = 0.5

N = int(FS * WINDOW_SEC)      # 1024
STEP = int(FS * STEP_SEC)     # 256

def bandpower(f, pxx, lo, hi):
    idx = (f >= lo) & (f < hi)
    return np.trapz(pxx[idx], f[idx])

def preprocess(x, fs):
    x = x - np.mean(x)  # remove DC (x - mean)
    b, a = butter(4, [0.5/(fs/2), 45/(fs/2)], btype='band') # bandpass 0.5 - 45 Hz (EEG range)
    x = filtfilt(b, a, x) # notch 50 Hz (remove mains noise)
    bn, an = iirnotch(50/(fs/2), Q=30)
    x = filtfilt(bn, an, x)
    return x

raw = []
with open("tgam_raw_20260310_231706.jsonl","r",encoding="utf-8") as f:
    for line in f:
        obj = json.loads(line)
        if "raw" in obj:
            raw.extend(obj["raw"])

x = np.array(raw, dtype=float)
for i in range(0, len(x)-N+1, STEP):
    seg = preprocess(x[i:i+N], FS)
    f, pxx = welch(seg, fs=FS, nperseg=FS)  # freq, power
    bp = {
        "delta": bandpower(f, pxx, 0.5, 4),
        "theta": bandpower(f, pxx, 4, 8),
        "alpha": bandpower(f, pxx, 8, 13),
        "beta":  bandpower(f, pxx, 13, 30),
        "gamma": bandpower(f, pxx, 30, 45),
    }
    print({k: float(v) for k, v in bp.items()})
    