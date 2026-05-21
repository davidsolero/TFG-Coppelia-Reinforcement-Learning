"""
Smoke test del entorno antes de entrenar exp_018.

Este script valida la ruta que usa PPO:
- reset() devuelve una observacion valida
- step() acepta acciones del espacio real del entorno
- la nueva observacion contiene el tiempo real de la ultima accion
- info expone last_action_elapsed_s

Uso:
    python auxiliares/test_episodes.py
"""

from pathlib import Path
import sys

import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from robot_env import RobotEnv


NUM_EPISODES = 3
MAX_STEPS_PER_EPISODE = 8
TRACE = False


def format_obs(obs):
    obs = np.asarray(obs, dtype=np.float32)
    return (
        f"nodo={obs[0]:.0f} bateria={obs[1]:.1f} "
        f"vis=[{obs[2]:.0f},{obs[3]:.0f},{obs[4]:.0f},{obs[5]:.0f}] "
        f"t_acc={obs[6]:.3f}s"
    )


def run_episode(env, episode_num):
    print(f"\n{'='*60}")
    print(f"EPISODIO {episode_num}")
    print(f"{'='*60}")

    obs, info = env.reset()
    print(f"Reset: {format_obs(obs)}")

    total_reward = 0.0
    terminated = False
    truncated = False

    for step in range(1, MAX_STEPS_PER_EPISODE + 1):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += float(reward)

        last_action_name = info.get("last_action_name", "unknown")
        last_action_elapsed_s = float(info.get("last_action_elapsed_s", 0.0))

        print(
            f"  Paso {step:02d}: action={action.tolist()}  {last_action_name:5s}  "
            f"reward={reward:+.2f}  {format_obs(obs)}  info_t_acc={last_action_elapsed_s:.3f}s"
        )

        if not np.isfinite(obs[6]):
            raise AssertionError("La observacion no contiene un tiempo finito en la ultima posicion")
        if obs[6] < 0:
            raise AssertionError("El tiempo de la ultima accion no puede ser negativo")
        if last_action_elapsed_s < 0:
            raise AssertionError("last_action_elapsed_s no puede ser negativo")

        if terminated or truncated:
            break

    print(
        f"Resumen: reward_total={total_reward:.2f} terminated={terminated} truncated={truncated} "
        f"pasos={step}"
    )


def main():
    env = RobotEnv(max_steps=MAX_STEPS_PER_EPISODE, trace=TRACE)

    try:
        print("\nSmoke test del entorno para exp_018")
        print(f"Observacion: {env.observation_space}")
        print(f"Accion: {env.action_space}")

        for episode_num in range(1, NUM_EPISODES + 1):
            run_episode(env, episode_num)

        print("\nSmoke test completado sin errores.")
    finally:
        env.close()


if __name__ == "__main__":
    main()