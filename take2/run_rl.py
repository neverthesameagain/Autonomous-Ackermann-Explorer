#!/usr/bin/env python3
"""
run_rl.py
Unified launcher for RL training/evaluation with mock fallbacks.
Supports --mode [ppo|dqn|eval]
"""

import sys
import os
import subprocess
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src" / "rl"

# ----------------------------------------------------------------------
# 1. Torch check
# ----------------------------------------------------------------------
try:
    import torch
    print("✅ PyTorch found.")
except ImportError:
    print("⚠️ PyTorch not found — loading mock_torch.py fallback.")
    spec = importlib.util.spec_from_file_location("torch", ROOT / "mock_torch.py")
    torch_mod = importlib.util.module_from_spec(spec)
    sys.modules["torch"] = torch_mod
    spec.loader.exec_module(torch_mod)
    print("✅ MockTorch active (NumPy fallback).")

# ----------------------------------------------------------------------
# 2. Stable-Baselines3 check
# ----------------------------------------------------------------------
try:
    import stable_baselines3
    print("✅ Stable-Baselines3 found.")
except ImportError:
    print("⚠️ SB3 not found — loading mock_stable_baselines3.py fallback.")
    spec = importlib.util.spec_from_file_location("stable_baselines3", ROOT / "mock_stable_baselines3.py")
    sb3_mod = importlib.util.module_from_spec(spec)
    sys.modules["stable_baselines3"] = sb3_mod
    spec.loader.exec_module(sb3_mod)
    print("✅ MockSB3 active (stub fallback).")

# ----------------------------------------------------------------------
# 3. Parse mode
# ----------------------------------------------------------------------
mode = "ppo"
if len(sys.argv) > 1 and sys.argv[1].startswith("--mode"):
    mode = sys.argv[1].split("=")[1] if "=" in sys.argv[1] else sys.argv[2]
print(f"\n🚀 Launching {mode.upper()} training ...")

script_map = {
    "ppo": SRC / "train_rl.py",
    "dqn": SRC / "train_dqn.py",
    "eval": SRC / "eval_policy.py"
}
target = script_map.get(mode.lower(), SRC / "train_rl.py")

# ----------------------------------------------------------------------
# 4. Ensure environment inherits our mocks
# ----------------------------------------------------------------------
env = dict(**os.environ)
env["PYTHONPATH"] = str(ROOT)
env["MOCK_TORCH_ACTIVE"] = "1"
env["MOCK_SB3_ACTIVE"] = "1"

# Launch subprocess with current interpreter
subprocess.run([sys.executable, str(target)], env=env)
