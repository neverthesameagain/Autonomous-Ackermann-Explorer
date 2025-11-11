"""
mock_stable_baselines3.py
--------------------------------------------------------
Fallback mock for environments that lack PyTorch/SB3.
Provides no-op versions of PPO, DQN, DummyVecEnv, and Monitor.
Allows RL scripts to run safely under Python 3.13 without torch.
"""
import warnings
import numpy as np

warnings.warn("[MockSB3] Using stub — no RL training will occur.")

# ---------------------------------------------------------------------
# Base mock class
# ---------------------------------------------------------------------
class _MockAlgo:
    def __init__(self, policy, env=None, **kwargs):
        self.policy = policy
        self.env = env
        self.kwargs = kwargs
        self.total_timesteps = 0
        print(f"[MockSB3] Initialized {self.__class__.__name__} (no real learning)")

    def learn(self, total_timesteps=10000, reset_num_timesteps=True):
        """Simulate training loop without any actual learning."""
        self.total_timesteps += total_timesteps
        for i in range(0, total_timesteps, 1000):
            print(f"[MockSB3] Simulating training... {i}/{total_timesteps}")
        print(f"[MockSB3] Completed {self.total_timesteps} fake timesteps.")
        return self

    def predict(self, obs, deterministic=True):
        """Random action fallback."""
        if self.env is None or not hasattr(self.env, "action_space"):
            return np.zeros(2), None
        return self.env.action_space.sample(), None

    def save(self, path):
        print(f"[MockSB3] Pretending to save model → {path}")
        with open(path, "w") as f:
            f.write("This is a mock SB3 model (no real weights).")

    @classmethod
    def load(cls, path):
        print(f"[MockSB3] Pretending to load model from {path}")
        return cls("MockPolicy", env=None)


# ---------------------------------------------------------------------
# Mock vectorized wrappers
# ---------------------------------------------------------------------
class DummyVecEnv:
    def __init__(self, env_fns):
        self.envs = [fn() for fn in env_fns]
        print("[MockSB3] DummyVecEnv initialized with", len(self.envs), "envs")

    def reset(self):
        return [env.reset()[0] for env in self.envs]

    def step(self, actions):
        obs, rewards, dones, infos = [], [], [], []
        for env, act in zip(self.envs, actions):
            o, r, t, tr, i = env.step(act)
            obs.append(o)
            rewards.append(r)
            dones.append(t or tr)
            infos.append(i)
        return obs, rewards, dones, infos


class Monitor:
    def __init__(self, env):
        self.env = env

    def __getattr__(self, name):
        return getattr(self.env, name)


# ---------------------------------------------------------------------
# Exported algorithm stubs
# ---------------------------------------------------------------------
class PPO(_MockAlgo): pass
class DQN(_MockAlgo): pass


__all__ = ["PPO", "DQN", "DummyVecEnv", "Monitor"]
