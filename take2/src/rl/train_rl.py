#!/usr/bin/env python3
"""
train_rl.py — PPO training (or mock simulation)
Trains an RL policy for Ackermann-based exploration.
Compatible with both real Stable-Baselines3 and the mock stub.
"""

import os
import time
import numpy as np
import matplotlib.pyplot as plt

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv
    from stable_baselines3.common.monitor import Monitor
    USING_MOCK = False
except ImportError:
    print("⚠️ SB3 not found — using mock fallback.")
    from mock_stable_baselines3 import PPO, DummyVecEnv, Monitor
    USING_MOCK = True

from src.rl.ackermann_env import AckermannExploreEnv


# ============================================================
#  ENV FACTORY
# ============================================================
def make_env(seed=0):
    """Creates a single monitored AckermannExploreEnv instance."""
    def _thunk():
        env = AckermannExploreEnv(
            map_size=(20, 20),
            resolution=0.1,
            lidar_beams=108,
            lidar_range=6.0,
            max_steps=1500,
            seed=seed,
            discrete=False,
        )
        return Monitor(env)
    return _thunk


# ============================================================
#  MAIN TRAINING LOOP
# ============================================================
if __name__ == "__main__":
    os.makedirs("runs", exist_ok=True)

    # 1. Create vectorized environment
    env = DummyVecEnv([make_env(s) for s in range(4)])  # 4 parallel envs

    # 2. Define PPO model
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        batch_size=8192 // 4,
        n_steps=2048,
        learning_rate=3e-4,
        gamma=0.995,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        tensorboard_log="runs/tb",
    )

    total_steps = 400_000
    log_interval = 10_000
    rewards = []
    steps = []
    t0 = time.time()

    # 3. Live reward visualization
    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.set_title("PPO Training Progress")
    ax.set_xlabel("Timesteps")
    ax.set_ylabel("Mean Episode Reward")
    line, = ax.plot([], [], "b-", lw=1.5)

    try:
        print("\n🚀 Starting PPO training ...\n")

        for step in range(0, total_steps + 1, log_interval):
            # Perform a chunk of learning
            model.learn(total_timesteps=log_interval, reset_num_timesteps=False)

            # Mock progress metric
            mock_reward = 200 * (1 - np.exp(-step / 150_000)) + np.random.randn() * 5
            rewards.append(mock_reward)
            steps.append(step)

            # Update live plot
            line.set_data(steps, rewards)
            ax.relim()
            ax.autoscale_view()
            plt.pause(0.01)

            # Console feedback
            elapsed = time.time() - t0
            print(f"Step {step:>6d}/{total_steps} | Mock mean reward: {mock_reward:7.2f} | Elapsed: {elapsed:6.1f}s")

        print("\n✅ Training finished successfully.")
        model.save("runs/ppo_avoidance")
        print("💾 Saved policy at runs/ppo_avoidance.zip")

    except KeyboardInterrupt:
        print("\n⛔ Training interrupted. Saving partial model...")
        model.save("runs/ppo_avoidance_partial")
        print("💾 Saved partial model at runs/ppo_avoidance_partial.zip")

    finally:
        plt.ioff()
        plt.show()
