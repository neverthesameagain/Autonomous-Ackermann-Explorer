# train_dqn.py
import os
from stable_baselines3 import DQN
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor

from src.rl.ackermann_env import AckermannExploreEnv

def make_env(seed=0):
    def _thunk():
        return Monitor(AckermannExploreEnv(discrete=True, seed=seed))
    return _thunk

if __name__ == "__main__":
    os.makedirs("runs", exist_ok=True)
    env = DummyVecEnv([make_env(s) for s in range(4)])

    model = DQN(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=2.5e-4,
        buffer_size=200_000,
        learning_starts=10_000,
        batch_size=256,
        gamma=0.995,
        target_update_interval=2000,
        train_freq=(4, "step"),
        exploration_initial_eps=1.0,
        exploration_final_eps=0.05,
        exploration_fraction=0.3,
        tensorboard_log="runs/tb"
    )
    model.learn(total_timesteps=500_000)
    model.save("runs/dqn_avoidance")
    print("Saved policy at runs/dqn_avoidance.zip")
