"""
MockTorch — lightweight fallback module to emulate minimal PyTorch APIs.
Allows importing Stable-Baselines3 and Gym-based RL code on Python 3.13
without official torch builds.
"""

import numpy as np
import types
import warnings

warnings.warn("[MockTorch] Using NumPy fallback — no GPU or autograd support.")

class Tensor(np.ndarray):
    """A minimal torch.Tensor replacement."""

    def __new__(cls, input_array):
        obj = np.asarray(input_array).view(cls)
        return obj

    def numpy(self):
        return np.asarray(self)

    def item(self):
        return float(self)

    def to(self, *_, **__):
        return self

    def detach(self):
        return self

    def cpu(self):
        return self

    def __repr__(self):
        return f"Tensor({np.array2string(self, precision=3)})"


# --- functional helpers ---
def tensor(x, dtype=None, device=None):
    return Tensor(x)

def zeros(*shape, **kwargs):
    return Tensor(np.zeros(shape))

def ones(*shape, **kwargs):
    return Tensor(np.ones(shape))

def randn(*shape, **kwargs):
    return Tensor(np.random.randn(*shape))

def manual_seed(seed):
    np.random.seed(seed)

def no_grad():
    """Mock context manager for no_grad."""
    return types.SimpleNamespace(__enter__=lambda *a, **k: None,
                                 __exit__=lambda *a, **k: None)


# --- nn.Module / layers ---
class Module:
    def __init__(self):
        pass
    def forward(self, *args, **kwargs):
        raise NotImplementedError
    def __call__(self, *args, **kwargs):
        return self.forward(*args, **kwargs)

class Linear(Module):
    def __init__(self, in_features, out_features):
        super().__init__()
        self.weight = np.random.randn(out_features, in_features) * 0.01
        self.bias = np.zeros(out_features)
    def forward(self, x):
        return x @ self.weight.T + self.bias

class ReLU(Module):
    def forward(self, x): return np.maximum(0, x)

class Sequential(Module):
    def __init__(self, *layers): self.layers = layers
    def forward(self, x):
        for layer in self.layers:
            x = layer(x)
        return x

# --- optimizers ---
class SGD:
    def __init__(self, params, lr=0.01): self.lr = lr
    def step(self): pass
    def zero_grad(self): pass

class Adam(SGD): pass

# --- devices ---
def cuda_is_available(): return False
device = "cpu"

# Create submodules to emulate `torch.nn`, `torch.optim`, etc.
nn = types.SimpleNamespace(Module=Module, Linear=Linear, ReLU=ReLU, Sequential=Sequential)
optim = types.SimpleNamespace(SGD=SGD, Adam=Adam)
no_grad = no_grad
manual_seed = manual_seed

# Expose top-level names to mimic torch
__all__ = [
    "Tensor", "tensor", "zeros", "ones", "randn",
    "manual_seed", "no_grad", "nn", "optim", "cuda_is_available"
]
