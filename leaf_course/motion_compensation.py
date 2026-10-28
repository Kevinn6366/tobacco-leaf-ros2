"""Constant-velocity feed-forward in calibrated world coordinates (SI units)."""
import numpy as np

def predict_position(detected, encoder_velocity, age_s, approach_s):
 if age_s<0 or approach_s<0:raise ValueError('Negative measurement age or approach latency')
 p=np.asarray(detected,dtype=float);v=np.asarray(encoder_velocity,dtype=float)
 if p.shape!=v.shape or p.shape[-1]!=3:raise ValueError('Expected matching world XYZ vectors')
 if not np.all(np.isfinite(p)) or not np.all(np.isfinite(v)):raise ValueError('Non-finite detection/encoder')
 return p+v*(age_s+approach_s)
