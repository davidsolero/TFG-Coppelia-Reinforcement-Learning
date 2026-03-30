"""
Evaluación profunda del agente PPO (modo explotación) con barra de progreso.

Objetivo:
    - Evaluar la política aprendida SIN aprendizaje (deterministic=True)
    - Ejecutar múltiples episodios (>=500 recomendado)
    - Calcular métricas relevantes del comportamiento
    - Generar histogramas y boxplots
    - Guardar resultados en CSV
    - Mostrar barra de progreso y tiempo estimado restante

Uso:
    python evaluate_deep.py
"""

from stable_baselines3 import PPO
from robot_env import RobotEnv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tqdm import tqdm
import time
import os

# ── Configuración de experimento ───────────────────────────────────────────────
EXP_NAME = "exp_001_Cmedia"  # Debe coincidir con train.py
BASE_DIR = f"./experiments/{EXP_NAME}"
os.makedirs(f"{BASE_DIR}/eval", exist_ok=True)

# ── Parámetros ────────────────────────────────────────────────────────────────
MODEL_PATH   = f"{BASE_DIR}/model/ppo_robot"
NUM_EPISODES = 500  # 1000/500 para evaluación profunda
MAX_STEPS    = 50
VERBOSE      = False  # True solo para debug

# ── Carga del entorno y modelo ─────────────────────────────────────────────────
print("Conectando a CoppeliaSim...")
env = RobotEnv(max_steps=MAX_STEPS, trace=False)

print(f"Cargando modelo: {MODEL_PATH}.zip")
model = PPO.load(MODEL_PATH, env=env)

# ── Métricas ─────────────────────────────────────────────────────────────────
all_rewards = []
all_steps = []
all_rooms = []
all_charges = []
all_full_coverage = []
all_time_in_C = []
all_min_battery = []

battery_depletions = 0

# ── Evaluación ───────────────────────────────────────────────────────────────
print(f"\nEvaluando {NUM_EPISODES} episodios...\n")
start_time = time.time()

for ep in tqdm(range(NUM_EPISODES), desc="Episodios evaluados"):
    obs, info = env.reset()

    ep_reward = 0
    ep_steps = 0
    rooms_visited = set()
    charges = 0
    time_in_C = 0
    min_battery = 100
    prev_node = None
    full_coverage_step = None

    while True:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(int(action))

        node = info['node']
        battery = info['battery']

        ep_reward += reward
        ep_steps += 1

        if node in ['Hab1', 'Hab2', 'Hab3']:
            rooms_visited.add(node)
        if node == 'C':
            time_in_C += 1
        if node == 'C' and prev_node != 'C':
            charges += 1
        min_battery = min(min_battery, battery)
        if len(rooms_visited) == 3 and full_coverage_step is None:
            full_coverage_step = ep_steps

        prev_node = node

        if VERBOSE:
            print(f"[{ep}] Paso {ep_steps}: {node}, bat={battery:.1f}")

        if terminated:
            battery_depletions += 1
            break
        if truncated:
            break

    full_coverage = (len(rooms_visited) == 3)
    if full_coverage_step is None:
        full_coverage_step = MAX_STEPS
    time_in_C_ratio = time_in_C / ep_steps

    all_rewards.append(ep_reward)
    all_steps.append(ep_steps)
    all_rooms.append(len(rooms_visited))
    all_charges.append(charges)
    all_full_coverage.append(int(full_coverage))
    all_time_in_C.append(time_in_C_ratio)
    all_min_battery.append(min_battery)

    if (ep + 1) % 50 == 0:
        elapsed = time.time() - start_time
        est_total = elapsed / (ep + 1) * NUM_EPISODES
        remaining = est_total - elapsed
        print(f" ~ Tiempo estimado restante: {remaining/60:.1f} min")

# ── Resultados numéricos ─────────────────────────────────────────────────────
print("\n" + "="*60)
print("RESUMEN GLOBAL")
print("="*60)

print(f"Reward medio:              {np.mean(all_rewards):.2f} ± {np.std(all_rewards):.2f}")
print(f"Habitaciones medias:       {np.mean(all_rooms):.2f} / 3")
print(f"Full coverage rate:        {np.mean(all_full_coverage)*100:.1f}%")
print(f"Tiempo medio en C:         {np.mean(all_time_in_C)*100:.1f}%")
print(f"Batería mínima media:      {np.mean(all_min_battery):.1f}%")
print(f"Cargas medias:             {np.mean(all_charges):.2f}")
print(f"Episodios con batería 0:   {battery_depletions}/{NUM_EPISODES}")

# ── Guardar CSV ──────────────────────────────────────────────────────────────
df = pd.DataFrame({
    "reward": all_rewards,
    "steps": all_steps,
    "rooms": all_rooms,
    "charges": all_charges,
    "full_coverage": all_full_coverage,
    "time_in_C": all_time_in_C,
    "min_battery": all_min_battery
})
df.to_csv(f"{BASE_DIR}/eval/results.csv", index=False)
print(f"\nResultados guardados en {BASE_DIR}/eval/results.csv")

# ── Gráficas ────────────────────────────────────────────────────────────────
plt.figure()
plt.hist(all_rooms, bins=4)
plt.title("Habitaciones visitadas")
plt.xlabel("Número")
plt.ylabel("Frecuencia")

plt.figure()
plt.hist(all_time_in_C, bins=20)
plt.title("Tiempo en C (%)")

plt.figure()
plt.boxplot([all_rewards, all_rooms, all_time_in_C, all_min_battery])
plt.xticks([1,2,3,4], ["Reward", "Rooms", "Time_C", "Battery"])
plt.title("Distribución de métricas")

plt.show()

# ── Cierre limpio ─────────────────────────────────────────────────────────────
env.close()
print("\nEvaluación completada.")