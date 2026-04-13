"""
Evaluación del agente PPO entrenado.
Ejecutar con CoppeliaSim abierto CON interfaz gráfica (no headless).

Uso:
    1. Abre CoppeliaSim normalmente (con ventana)
    2. Carga la escena entornoRobotVigilancia.ttt
    3. Ejecuta: python evaluate.py

Métricas reportadas por episodio:
    - Recompensa total
    - Habitaciones visitadas (Hab1, Hab2, Hab3)
    - Veces que fue a C voluntariamente
    - Tiempo en C y fuera de C
    - Paso de cobertura completa
    - Desbalance entre habitaciones
    - Si la batería se agotó
    - Pasos realizados
"""

from stable_baselines3 import PPO
from robot_env import RobotEnv
import numpy as np
from pathlib import Path


def room_balance_metrics(room_counts):
    values = np.array([room_counts['Hab1'], room_counts['Hab2'], room_counts['Hab3']], dtype=np.float32)
    return float(np.std(values)), float(np.max(values) - np.min(values))


def safe_ratio(numerator, denominator):
    return float(numerator) / float(denominator) if denominator else 0.0

# ── Configuración de experimento ──────────────────────────────────────────────
EXP_NAME = "exp_008_MediaHabSinCyConPenalFija"  # Debe coincidir con train.py
PROJECT_DIR = Path(__file__).resolve().parent
BASE_DIR = PROJECT_DIR / "experiments" / EXP_NAME

# ── Parámetros ────────────────────────────────────────────────────────────────
MODEL_PATH = BASE_DIR / "model" / "ppo_robot"
NUM_EPISODES = 10
MAX_STEPS    = 50

# ── Cargar entorno y modelo ───────────────────────────────────────────────────

print("Conectando a CoppeliaSim...")
env = RobotEnv(max_steps=MAX_STEPS, trace=False)

model_zip = MODEL_PATH.with_suffix(".zip")
print(f"Cargando modelo: {model_zip}")

if not model_zip.exists():
    env.close()
    raise FileNotFoundError(
        f"No se encontro el modelo en: {model_zip}\n"
        "Verifica que el entrenamiento haya generado ppo_robot.zip en la carpeta esperada."
    )

model = PPO.load(str(MODEL_PATH), env=env)

# ── Evaluación ────────────────────────────────────────────────────────────────

print(f"\n{'='*60}")
print(f"EVALUACIÓN: {NUM_EPISODES} episodios (deterministic=True)")
print(f"{'='*60}\n")

# Métricas globales
all_rewards      = []
all_steps        = []
all_rooms        = []
all_charges      = []
all_time_in_C    = []
all_time_out_C   = []
all_min_battery  = []
all_reward_per_step = []
all_full_coverage_step = []
all_room_std     = []
all_room_spread  = []
all_room1_counts = []
all_room2_counts = []
all_room3_counts = []
battery_depletions = 0

for ep in range(NUM_EPISODES):
    obs, info = env.reset()
    
    ep_reward    = 0.0
    ep_steps     = 0
    rooms_visited = set()
    room_counts = {'Hab1': 0, 'Hab2': 0, 'Hab3': 0}
    charges      = 0
    time_in_C    = 0
    min_battery  = 100.0
    full_coverage_step = None
    depleted     = False
    prev_node    = None

    print(f"--- Episodio {ep + 1}/{NUM_EPISODES} ---")

    while True:
        # Acción determinista: el agente elige lo que aprendió, sin ruido
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(int(action))

        ep_reward += reward
        ep_steps  += 1

        node = info['node']
        battery = info['battery']

        # Registrar visitas a habitaciones
        if node in ['Hab1', 'Hab2', 'Hab3']:
            rooms_visited.add(node)
            room_counts[node] += 1

        # Registrar visitas voluntarias a C
        if node == 'C' and prev_node != 'C':
            charges += 1

        if node == 'C':
            time_in_C += 1

        min_battery = min(min_battery, battery)

        if len(rooms_visited) == 3 and full_coverage_step is None:
            full_coverage_step = ep_steps

        prev_node = node

        print(f"  Paso {ep_steps:2d}: nodo={node:4s}  batería={battery:5.1f}%  "
              f"reward={reward:+.1f}  acum={ep_reward:.1f}")

        if terminated:
            depleted = True
            battery_depletions += 1
            print(f"  !! BATERÍA AGOTADA")
            break
        if truncated:
            print(f"  -- Episodio completado ({MAX_STEPS} pasos)")
            break

    # Resumen del episodio
    time_in_C_ratio = safe_ratio(time_in_C, ep_steps)
    time_out_C_ratio = 1.0 - time_in_C_ratio
    reward_per_step = safe_ratio(ep_reward, ep_steps)
    room_std, room_spread = room_balance_metrics(room_counts)

    print(f"  Resumen: reward={ep_reward:.1f}  pasos={ep_steps}  "
          f"habitaciones={len(rooms_visited)}/3  "
          f"cargas={charges}  time_in_C={time_in_C_ratio*100:.1f}%  "
          f"full_coverage_step={full_coverage_step if full_coverage_step is not None else 'N/A'}  "
          f"room_std={room_std:.2f}  room_spread={room_spread:.0f}  "
          f"min_battery={min_battery:.1f}%  depleted={'SÍ' if depleted else 'NO'}\n")

    all_rewards.append(ep_reward)
    all_steps.append(ep_steps)
    all_rooms.append(len(rooms_visited))
    all_charges.append(charges)
    all_time_in_C.append(time_in_C_ratio)
    all_time_out_C.append(time_out_C_ratio)
    all_min_battery.append(min_battery)
    all_reward_per_step.append(reward_per_step)
    all_full_coverage_step.append(full_coverage_step if full_coverage_step is not None else MAX_STEPS)
    all_room_std.append(room_std)
    all_room_spread.append(room_spread)
    all_room1_counts.append(room_counts['Hab1'])
    all_room2_counts.append(room_counts['Hab2'])
    all_room3_counts.append(room_counts['Hab3'])

# ── Resumen final ─────────────────────────────────────────────────────────────

print(f"{'='*60}")
print(f"RESUMEN FINAL ({NUM_EPISODES} episodios)")
print(f"{'='*60}")
print(f"Recompensa media:              {np.mean(all_rewards):.2f} ± {np.std(all_rewards):.2f}")
print(f"Recompensa máxima:             {np.max(all_rewards):.2f}")
print(f"Recompensa mínima:             {np.min(all_rewards):.2f}")
print(f"Pasos medios por episodio:     {np.mean(all_steps):.1f}")
print(f"Habitaciones medias visitadas: {np.mean(all_rooms):.1f} / 3")
print(f"Cargas voluntarias medias:     {np.mean(all_charges):.1f}")
print(f"Tiempo medio en C:             {np.mean(all_time_in_C)*100:.1f}%")
print(f"Tiempo medio fuera de C:       {np.mean(all_time_out_C)*100:.1f}%")
print(f"Cobertura completa media en:    {np.mean(all_full_coverage_step):.1f} pasos")
print(f"Recompensa media por paso:      {np.mean(all_reward_per_step):.3f}")
print(f"Desbalance medio (std):         {np.mean(all_room_std):.2f}")
print(f"Desbalance medio (max-min):     {np.mean(all_room_spread):.2f}")
print(f"Batería mínima media:           {np.mean(all_min_battery):.1f}%")
print(f"Episodios con batería agotada: {battery_depletions} / {NUM_EPISODES}")
print(f"{'='*60}")

env.close()
print("Evaluación completada.")