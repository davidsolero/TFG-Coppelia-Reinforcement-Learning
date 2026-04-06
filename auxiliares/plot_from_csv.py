"""
Script auxiliar para reconstruir gráficas desde CSV de evaluaciones de RobotEnv.
No requiere conexión a CoppeliaSim, solo pandas y matplotlib.

Uso:
    python plot_from_csv.py
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# ── Configuración ─────────────────────────────────────────────────────────────
EXPERIMENT_NAME = "exp_006_MediaHabSinCyConPenalAdaptativaBalance"
CSV_PATH = f"./experiments/{EXPERIMENT_NAME}/deep_evaluation/deepevaluation_resultsv2.csv"
PLOTS_DIR = f"./experiments/{EXPERIMENT_NAME}/deep_evaluation/plotsv2"

if not os.path.exists(CSV_PATH):
    raise FileNotFoundError(f"No se encontró el CSV en {CSV_PATH}")

os.makedirs(PLOTS_DIR, exist_ok=True)

# ── Carga de datos ────────────────────────────────────────────────────────────
df = pd.read_csv(CSV_PATH)


def col(name, fallback=None):
    if name in df.columns:
        return df[name]
    if fallback is not None and fallback in df.columns:
        return df[fallback]
    raise KeyError(f"No se encontró la columna '{name}' en el CSV")

# ── Gráficas (idénticas a evaluate_deep.py) ─────────────────────────────────
plt.figure()
plt.hist(col('rooms'), bins=4)
plt.title("Habitaciones visitadas")
plt.xlabel("Número")
plt.ylabel("Frecuencia")
plt.savefig(os.path.join(PLOTS_DIR, "hist_rooms.png"), dpi=150, bbox_inches="tight")

plt.figure()
plt.hist(col('time_in_C'), bins=20)
plt.title("Tiempo en C (%)")
plt.savefig(os.path.join(PLOTS_DIR, "hist_time_in_c.png"), dpi=150, bbox_inches="tight")

if 'room_std' in df.columns:
    plt.figure()
    plt.hist(df['room_std'], bins=20)
    plt.title("Desbalance entre habitaciones (std)")
    plt.savefig(os.path.join(PLOTS_DIR, "hist_room_std.png"), dpi=150, bbox_inches="tight")

if all(col_name in df.columns for col_name in ['hab1_visits', 'hab2_visits', 'hab3_visits']):
    plt.figure()
    room_labels = ["Hab1", "Hab2", "Hab3"]
    room_means = [
        float(df['hab1_visits'].mean()),
        float(df['hab2_visits'].mean()),
        float(df['hab3_visits'].mean()),
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

if 'reward_per_step' in df.columns:
    plt.figure()
    plt.hist(df['reward_per_step'], bins=20)
    plt.title("Reward por paso")
    plt.savefig(os.path.join(PLOTS_DIR, "hist_reward_per_step.png"), dpi=150, bbox_inches="tight")

if 'min_battery' in df.columns:
    plt.figure()
    plt.hist(df['min_battery'], bins=20)
    plt.title("Batería mínima")
    plt.savefig(os.path.join(PLOTS_DIR, "hist_min_battery.png"), dpi=150, bbox_inches="tight")

plt.figure()
boxplot_series = [col('reward'), col('rooms'), col('time_in_C'), col('min_battery')]
boxplot_labels = ["Reward", "Rooms", "Time_C", "Battery"]
if 'reward_per_step' in df.columns:
    boxplot_series.insert(1, df['reward_per_step'])
    boxplot_labels.insert(1, "Reward/step")
if 'room_std' in df.columns:
    boxplot_series.append(df['room_std'])
    boxplot_labels.append("Room_std")
plt.boxplot(boxplot_series)
plt.xticks(list(range(1, len(boxplot_labels) + 1)), boxplot_labels)
plt.title("Distribución de métricas")
plt.savefig(os.path.join(PLOTS_DIR, "boxplot_metrics.png"), dpi=150, bbox_inches="tight")

plt.show()

# ── Métricas numéricas rápidas ───────────────────────────────────────────────
print("\nResumen rápido:")
print(f"Reward medio:          {df['reward'].mean():.2f} ± {df['reward'].std():.2f}")
print(f"Habitaciones medias:   {df['rooms'].mean():.2f} / 3")
print(f"Tiempo medio en C:     {df['time_in_C'].mean()*100:.1f}%")
print(f"Batería mínima media:  {df['min_battery'].mean():.1f}%")
print(f"Gráficas guardadas en: {PLOTS_DIR}")