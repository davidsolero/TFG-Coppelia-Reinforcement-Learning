"""
Evaluación del agente PPO entrenado.
Usa evaluate_policy de SB3 para evaluar con las mismas condiciones que el entrenamiento.

Uso:
    1. Abre CoppeliaSim normalmente (con ventana)
    2. Ejecuta: python evaluate.py
"""

from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from robot_env import RobotEnv
import numpy as np

# ── Parámetros ────────────────────────────────────────────────────────────────

MODEL_PATH   = "./models/exp1_C_en_media/ppo_robot"
NUM_EPISODES = 10
MAX_STEPS    = 50

# ── Cargar entorno y modelo ───────────────────────────────────────────────────

print("Conectando a CoppeliaSim...")
env = RobotEnv(max_steps=MAX_STEPS, trace=False)

print(f"Cargando modelo: {MODEL_PATH}.zip")
model = PPO.load(MODEL_PATH, env=env)

# ── Evaluación ────────────────────────────────────────────────────────────────

print(f"\n{'='*60}")
print(f"EVALUACIÓN: {NUM_EPISODES} episodios (deterministic=True)")
print(f"{'='*60}\n")

mean_reward, std_reward = evaluate_policy(
    model,
    env,
    n_eval_episodes = NUM_EPISODES,
    deterministic   = True,
    render          = False,
)

print(f"\n{'='*60}")
print(f"RESUMEN FINAL ({NUM_EPISODES} episodios)")
print(f"{'='*60}")
print(f"Recompensa media: {mean_reward:.2f} ± {std_reward:.2f}")
print(f"{'='*60}")

env.close()
print("Evaluación completada.")