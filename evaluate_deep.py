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
import sys
import atexit


def room_balance_metrics(room_counts):
    values = np.array([room_counts['Hab1'], room_counts['Hab2'], room_counts['Hab3']], dtype=np.float32)
    return float(np.std(values)), float(np.max(values) - np.min(values))


def safe_ratio(numerator, denominator):
    return float(numerator) / float(denominator) if denominator else 0.0


def percentiles(values):
    array = np.asarray(values, dtype=np.float32)
    return np.percentile(array, [10, 50, 90])

# ── Configuración de experimento ───────────────────────────────────────────────
EXP_NAME = "exp_008_MediaHabSinCyConPenalFija"  # Debe coincidir con train.py
BASE_DIR = f"./experiments/{EXP_NAME}"
os.makedirs(f"{BASE_DIR}/deep_evaluation", exist_ok=True)
PLOTS_DIR = f"{BASE_DIR}/deep_evaluation/plotsv3"
os.makedirs(PLOTS_DIR, exist_ok=True)
LOG_PATH = f"{BASE_DIR}/deep_evaluation/deepevaluation_logv3.txt"


class TeeStream:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for stream in self.streams:
            stream.write(data)
            stream.flush()

    def flush(self):
        for stream in self.streams:
            stream.flush()


log_file = open(LOG_PATH, "w", encoding="utf-8")
atexit.register(log_file.close)
sys.stdout = TeeStream(sys.__stdout__, log_file)
sys.stderr = TeeStream(sys.__stderr__, log_file)
print(f"Guardando log en: {LOG_PATH}")

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
all_full_coverage_step = []
all_time_in_C = []
all_time_out_C = []
all_min_battery = []
all_reward_per_step = []
all_room_std = []
all_room_spread = []
all_room1_counts = []
all_room2_counts = []
all_room3_counts = []
all_recharge_battery_mean = []
all_recharge_battery_min = []
all_recharge_battery_max = []
all_recharge_battery_std = []
all_first_recharge_step = []
all_recharge_gap_mean = []
all_recharge_gap_min = []
all_recharge_gap_max = []
all_recharge_from_hab1 = []
all_recharge_from_hab2 = []
all_recharge_from_hab3 = []
all_recharge_from_other = []

decision_battery_samples = []
decision_recharge_flags = []
all_recharge_event_battery = []

battery_depletions = 0

# ── Evaluación ───────────────────────────────────────────────────────────────
print(f"\nEvaluando {NUM_EPISODES} episodios...\n")
start_time = time.time()

for ep in tqdm(range(NUM_EPISODES), desc="Episodios evaluados"):
    obs, info = env.reset()

    ep_reward = 0
    ep_steps = 0
    rooms_visited = set()
    room_counts = {'Hab1': 0, 'Hab2': 0, 'Hab3': 0}
    charges = 0
    time_in_C = 0
    min_battery = 100
    prev_node = None
    prev_battery = float(info.get('battery', 100.0))
    full_coverage_step = None
    first_recharge_step = np.nan
    recharge_steps = []
    recharge_batteries = []
    recharge_from_counts = {'Hab1': 0, 'Hab2': 0, 'Hab3': 0, 'other': 0}

    while True:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(int(action))

        node = info['node']
        battery = info['battery']

        ep_reward += reward
        ep_steps += 1

        if node in ['Hab1', 'Hab2', 'Hab3']:
            rooms_visited.add(node)
            room_counts[node] += 1
        if node == 'C':
            time_in_C += 1
        did_recharge = (node == 'C' and prev_node != 'C')
        decision_battery_samples.append(prev_battery)
        decision_recharge_flags.append(int(did_recharge))
        if did_recharge:
            charges += 1
            recharge_steps.append(ep_steps)
            recharge_batteries.append(float(prev_battery))
            all_recharge_event_battery.append(float(prev_battery))
            if np.isnan(first_recharge_step):
                first_recharge_step = ep_steps
            if prev_node in ['Hab1', 'Hab2', 'Hab3']:
                recharge_from_counts[prev_node] += 1
            else:
                recharge_from_counts['other'] += 1
        min_battery = min(min_battery, battery)
        if len(rooms_visited) == 3 and full_coverage_step is None:
            full_coverage_step = ep_steps

        prev_node = node
        prev_battery = float(battery)

        if VERBOSE:
            print(f"[{ep}] Paso {ep_steps}: {node}, bat={battery:.1f}")

        if terminated:
            battery_depletions += 1
            break
        if truncated:
            break

    full_coverage = (len(rooms_visited) == 3)
    if full_coverage_step is None:
        full_coverage_step = np.nan
    time_in_C_ratio = safe_ratio(time_in_C, ep_steps)
    time_out_C_ratio = 1.0 - time_in_C_ratio
    reward_per_step = safe_ratio(ep_reward, ep_steps)
    room_std, room_spread = room_balance_metrics(room_counts)
    if recharge_batteries:
        recharge_battery_mean = float(np.mean(recharge_batteries))
        recharge_battery_min = float(np.min(recharge_batteries))
        recharge_battery_max = float(np.max(recharge_batteries))
        recharge_battery_std = float(np.std(recharge_batteries))
    else:
        recharge_battery_mean = np.nan
        recharge_battery_min = np.nan
        recharge_battery_max = np.nan
        recharge_battery_std = np.nan

    if len(recharge_steps) >= 2:
        recharge_gaps = np.diff(recharge_steps)
        recharge_gap_mean = float(np.mean(recharge_gaps))
        recharge_gap_min = float(np.min(recharge_gaps))
        recharge_gap_max = float(np.max(recharge_gaps))
    else:
        recharge_gap_mean = np.nan
        recharge_gap_min = np.nan
        recharge_gap_max = np.nan

    all_rewards.append(ep_reward)
    all_steps.append(ep_steps)
    all_rooms.append(len(rooms_visited))
    all_charges.append(charges)
    all_full_coverage.append(int(full_coverage))
    all_full_coverage_step.append(full_coverage_step)
    all_time_in_C.append(time_in_C_ratio)
    all_time_out_C.append(time_out_C_ratio)
    all_min_battery.append(min_battery)
    all_reward_per_step.append(reward_per_step)
    all_room_std.append(room_std)
    all_room_spread.append(room_spread)
    all_room1_counts.append(room_counts['Hab1'])
    all_room2_counts.append(room_counts['Hab2'])
    all_room3_counts.append(room_counts['Hab3'])
    all_recharge_battery_mean.append(recharge_battery_mean)
    all_recharge_battery_min.append(recharge_battery_min)
    all_recharge_battery_max.append(recharge_battery_max)
    all_recharge_battery_std.append(recharge_battery_std)
    all_first_recharge_step.append(first_recharge_step)
    all_recharge_gap_mean.append(recharge_gap_mean)
    all_recharge_gap_min.append(recharge_gap_min)
    all_recharge_gap_max.append(recharge_gap_max)
    all_recharge_from_hab1.append(recharge_from_counts['Hab1'])
    all_recharge_from_hab2.append(recharge_from_counts['Hab2'])
    all_recharge_from_hab3.append(recharge_from_counts['Hab3'])
    all_recharge_from_other.append(recharge_from_counts['other'])

    if (ep + 1) % 50 == 0:
        elapsed = time.time() - start_time
        est_total = elapsed / (ep + 1) * NUM_EPISODES
        remaining = est_total - elapsed
        print(f" ~ Tiempo estimado restante: {remaining/60:.1f} min")

# ── Resultados numéricos ─────────────────────────────────────────────────────
print("\n" + "="*60)
print("RESUMEN GLOBAL")
print("="*60)

reward_p10, reward_p50, reward_p90 = percentiles(all_rewards)
time_in_c_p10, time_in_c_p50, time_in_c_p90 = percentiles(all_time_in_C)
time_out_c_p10, time_out_c_p50, time_out_c_p90 = percentiles(all_time_out_C)
min_battery_p10, min_battery_p50, min_battery_p90 = percentiles(all_min_battery)
reward_per_step_p10, reward_per_step_p50, reward_per_step_p90 = percentiles(all_reward_per_step)
coverage_steps = np.array(all_full_coverage_step, dtype=np.float32)
coverage_steps = coverage_steps[~np.isnan(coverage_steps)]
if coverage_steps.size:
    coverage_step_p10, coverage_step_p50, coverage_step_p90 = np.percentile(coverage_steps, [10, 50, 90])
else:
    coverage_step_p10 = coverage_step_p50 = coverage_step_p90 = np.nan

print(f"Reward medio:              {np.mean(all_rewards):.2f} ± {np.std(all_rewards):.2f}")
print(f"Reward p10/p50/p90:        {reward_p10:.2f} / {reward_p50:.2f} / {reward_p90:.2f}")
print(f"Reward por paso medio:     {np.mean(all_reward_per_step):.3f} ± {np.std(all_reward_per_step):.3f}")
print(f"Reward/step p10/p50/p90:   {reward_per_step_p10:.3f} / {reward_per_step_p50:.3f} / {reward_per_step_p90:.3f}")
print(f"Habitaciones medias:       {np.mean(all_rooms):.2f} / 3")
print(f"Full coverage rate:        {np.mean(all_full_coverage)*100:.1f}%")
print(f"Paso medio de cobertura:   {np.nanmean(all_full_coverage_step):.1f}")
print(f"Cobertura p10/p50/p90:     {coverage_step_p10:.1f} / {coverage_step_p50:.1f} / {coverage_step_p90:.1f}")
print(f"Tiempo medio en C:         {np.mean(all_time_in_C)*100:.1f}%")
print(f"Tiempo en C p10/p50/p90:   {time_in_c_p10*100:.1f}% / {time_in_c_p50*100:.1f}% / {time_in_c_p90*100:.1f}%")
print(f"Tiempo fuera de C medio:   {np.mean(all_time_out_C)*100:.1f}%")
print(f"Tiempo fuera de C p10/p50/p90: {time_out_c_p10*100:.1f}% / {time_out_c_p50*100:.1f}% / {time_out_c_p90*100:.1f}%")
print(f"Batería mínima media:      {np.mean(all_min_battery):.1f}%")
print(f"Batería mínima p10/p50/p90: {min_battery_p10:.1f}% / {min_battery_p50:.1f}% / {min_battery_p90:.1f}%")
print(f"Cargas medias:             {np.mean(all_charges):.2f}")
print(f"Hab1/Hab2/Hab3 medias:     {np.mean(all_room1_counts):.2f} / {np.mean(all_room2_counts):.2f} / {np.mean(all_room3_counts):.2f}")
print(f"Desbalance medio (std):    {np.mean(all_room_std):.2f}")
print(f"Desbalance medio (max-min): {np.mean(all_room_spread):.2f}")
print(f"Episodios con batería 0:   {battery_depletions}/{NUM_EPISODES}")
if all_recharge_event_battery:
    print(f"Batería media al recargar: {np.mean(all_recharge_event_battery):.1f}%")
else:
    print("Batería media al recargar: n/a (sin recargas detectadas)")

# ── Guardar CSV ──────────────────────────────────────────────────────────────
df = pd.DataFrame({
    "episode": np.arange(1, NUM_EPISODES + 1),
    "reward": all_rewards,
    "reward_per_step": all_reward_per_step,
    "steps": all_steps,
    "rooms": all_rooms,
    "charges": all_charges,
    "full_coverage": all_full_coverage,
    "full_coverage_step": all_full_coverage_step,
    "time_in_C": all_time_in_C,
    "time_out_C": all_time_out_C,
    "min_battery": all_min_battery,
    "hab1_visits": all_room1_counts,
    "hab2_visits": all_room2_counts,
    "hab3_visits": all_room3_counts,
    "room_std": all_room_std,
    "room_spread": all_room_spread,
    "recharge_battery_mean": all_recharge_battery_mean,
    "recharge_battery_min": all_recharge_battery_min,
    "recharge_battery_max": all_recharge_battery_max,
    "recharge_battery_std": all_recharge_battery_std,
    "first_recharge_step": all_first_recharge_step,
    "recharge_gap_mean": all_recharge_gap_mean,
    "recharge_gap_min": all_recharge_gap_min,
    "recharge_gap_max": all_recharge_gap_max,
    "recharge_from_hab1": all_recharge_from_hab1,
    "recharge_from_hab2": all_recharge_from_hab2,
    "recharge_from_hab3": all_recharge_from_hab3,
    "recharge_from_other": all_recharge_from_other,
})
df.to_csv(f"{BASE_DIR}/deep_evaluation/deepevaluation_resultsv3.csv", index=False)
print(f"\nResultados guardados en {BASE_DIR}/deep_evaluation/deepevaluation_resultsv3.csv")

# ── Gráficas ────────────────────────────────────────────────────────────────
plt.figure()
room_categories = np.array([0, 1, 2, 3], dtype=np.int32)
room_counts_plot = np.bincount(np.clip(np.asarray(all_rooms, dtype=np.int32), 0, 3), minlength=4)
room_perc_plot = room_counts_plot / max(1, len(all_rooms)) * 100.0
bars = plt.bar(room_categories, room_counts_plot, color="#4C78A8", width=0.75)
for idx, bar in enumerate(bars):
    plt.text(
        bar.get_x() + bar.get_width() / 2.0,
        bar.get_height() + 0.05,
        f"{room_counts_plot[idx]}\n({room_perc_plot[idx]:.1f}%)",
        ha="center",
        va="bottom",
        fontsize=8,
    )
plt.title("Habitaciones distintas visitadas por episodio")
plt.xlabel("Numero de habitaciones (0-3)")
plt.ylabel("Frecuencia")
plt.xticks(room_categories)
plt.savefig(os.path.join(PLOTS_DIR, "hist_rooms.png"), dpi=150, bbox_inches="tight")

plt.figure()
all_time_in_C_pct = np.asarray(all_time_in_C, dtype=np.float32) * 100.0
plt.hist(all_time_in_C_pct, bins=np.arange(0, 105, 5), color="#54A24B")
plt.title("Tiempo en C por episodio")
plt.xlabel("Tiempo en C (%)")
plt.ylabel("Frecuencia")
plt.xlim(0, 100)
plt.savefig(os.path.join(PLOTS_DIR, "hist_time_in_c.png"), dpi=150, bbox_inches="tight")

plt.figure()
plt.hist(all_room_std, bins=20)
plt.title("Desbalance entre habitaciones (std)")
plt.xlabel("Std de visitas")
plt.ylabel("Frecuencia")
plt.savefig(os.path.join(PLOTS_DIR, "hist_room_std.png"), dpi=150, bbox_inches="tight")

plt.figure()
room_labels = ["Hab1", "Hab2", "Hab3"]
room_means = [
    float(np.mean(all_room1_counts)),
    float(np.mean(all_room2_counts)),
    float(np.mean(all_room3_counts)),
]
dominant_idx = int(np.argmax(room_means))
colors = ["#4C78A8", "#F58518", "#54A24B"]
colors[dominant_idx] = "#E45756"
bars = plt.bar(room_labels, room_means, color=colors)
for idx, bar in enumerate(bars):
    plt.text(
        bar.get_x() + bar.get_width() / 2.0,
        bar.get_height() + 0.05,
        f"{room_means[idx]:.2f}",
        ha="center",
        va="bottom",
    )
plt.title(f"Balance entre habitaciones (mas visitada: {room_labels[dominant_idx]})")
plt.ylabel("Visitas medias por episodio")
plt.savefig(os.path.join(PLOTS_DIR, "balance_habs.png"), dpi=150, bbox_inches="tight")

plt.figure()
plt.hist(all_reward_per_step, bins=20)
plt.title("Reward por paso")
plt.xlabel("Reward/paso")
plt.ylabel("Frecuencia")
plt.savefig(os.path.join(PLOTS_DIR, "hist_reward_per_step.png"), dpi=150, bbox_inches="tight")

plt.figure()
plt.hist(all_min_battery, bins=20)
plt.title("Batería mínima")
plt.xlabel("Batería mínima (%)")
plt.ylabel("Frecuencia")
plt.xlim(0, 100)
plt.savefig(os.path.join(PLOTS_DIR, "hist_min_battery.png"), dpi=150, bbox_inches="tight")

plt.figure()
plt.boxplot([all_rewards, all_reward_per_step, all_time_in_C_pct, all_min_battery, all_room_std])
plt.xticks([1, 2, 3, 4, 5], ["Reward", "Reward/step", "Time_C(%)", "Battery(%)", "Room_std"])
plt.title("Distribución de métricas")
plt.ylabel("Valor (unidades mixtas)")
plt.savefig(os.path.join(PLOTS_DIR, "boxplot_metrics.png"), dpi=150, bbox_inches="tight")

if all_recharge_event_battery:
    plt.figure()
    plt.hist(all_recharge_event_battery, bins=15)
    plt.title("Batería al decidir recargar")
    plt.xlabel("Batería (%)")
    plt.ylabel("Frecuencia")
    plt.savefig(os.path.join(PLOTS_DIR, "hist_battery_at_recharge.png"), dpi=150, bbox_inches="tight")

first_recharge_values = np.array(all_first_recharge_step, dtype=np.float32)
first_recharge_values = first_recharge_values[~np.isnan(first_recharge_values)]
if first_recharge_values.size:
    plt.figure()
    plt.hist(first_recharge_values, bins=15)
    plt.title("Paso de la primera recarga")
    plt.xlabel("Paso")
    plt.ylabel("Frecuencia")
    plt.savefig(os.path.join(PLOTS_DIR, "hist_first_recharge_step.png"), dpi=150, bbox_inches="tight")

if decision_battery_samples:
    decisions_df = pd.DataFrame({
        "battery_before": decision_battery_samples,
        "recharged": decision_recharge_flags,
    })
    battery_bins = np.arange(0, 110, 10)
    decisions_df["battery_bin"] = pd.cut(
        decisions_df["battery_before"],
        bins=battery_bins,
        right=False,
        include_lowest=True,
    )
    recharge_prob_by_bin = decisions_df.groupby("battery_bin", observed=False)["recharged"].mean() * 100.0
    recharge_count_by_bin = decisions_df.groupby("battery_bin", observed=False)["recharged"].count()

    plt.figure(figsize=(10, 4))
    x_labels = [str(interval) for interval in recharge_prob_by_bin.index]
    plt.bar(x_labels, recharge_prob_by_bin.values, color="#4C78A8")
    for idx, val in enumerate(recharge_prob_by_bin.values):
        n_bin = int(recharge_count_by_bin.iloc[idx])
        plt.text(idx, val + 0.4, f"{val:.1f}%\n(n={n_bin})", ha="center", va="bottom", fontsize=8)
    plt.title("Probabilidad de recargar por batería previa")
    plt.xlabel("Bin de batería previa")
    plt.ylabel("Probabilidad de recarga (%)")
    plt.ylim(0, min(100, max(5, np.nanmax(recharge_prob_by_bin.values) + 6)))
    plt.xticks(rotation=30)
    plt.tight_layout()
    plt.savefig(os.path.join(PLOTS_DIR, "prob_recharge_by_battery_bin.png"), dpi=150, bbox_inches="tight")

plt.show()

# ── Cierre limpio ─────────────────────────────────────────────────────────────
env.close()
print("\nEvaluación completada.")