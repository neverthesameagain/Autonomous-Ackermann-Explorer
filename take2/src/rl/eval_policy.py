# eval_policy.py
import time
from stable_baselines3 import PPO, DQN
from src.rl.ackermann_env import AckermannExploreEnv

ALG = "PPO"           # "PPO" or "DQN"
CKPT = "runs/ppo_avoidance.zip"

if __name__ == "__main__":
    env = AckermannExploreEnv(map_size=(20,20), resolution=0.1, discrete=(ALG=="DQN"))
    model = (PPO if ALG=="PPO" else DQN).load(CKPT)

    obs, _ = env.reset()
    terminated = truncated = False
    ep_reward = 0.0

    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        obs, r, terminated, truncated, info = env.step(action)
        ep_reward += r
        # Optional: slow down to watch
        # time.sleep(0.01)

    print(f"Episode reward: {ep_reward:.2f}")
