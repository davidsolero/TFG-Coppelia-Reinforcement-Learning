"""Benchmark de 10 episodios con plan fijo y trazas para comparar comportamiento.

Uso:
    python auxiliares/benchmark_10_ep_tiempos.py

Flujo recomendado para comparar SIM_ACCELERATION=1 vs 5:
1) Ejecuta con SIM_ACCELERATION=1 y RUN_TAG='sim1'.
2) Ejecuta con SIM_ACCELERATION=5 y RUN_TAG='sim5' + COMPARE_WITH_FILE apuntando al JSON de sim1.
"""

from time import perf_counter
from statistics import mean, pstdev
from pathlib import Path
from datetime import datetime
from collections import Counter
import hashlib
import json
import sys

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from robot_env import RobotEnv


NUM_EPISODIOS = 10
MAX_STEPS = 50
TRACE = False
SEED = 0

# Cambia este tag manualmente para identificar cada corrida (ej: sim1, sim5).
RUN_TAG = "sim6_1acc_confirmation"

# Ruta opcional a JSON previo para comparar (deja "" para no comparar).
COMPARE_WITH_FILE = ""

OUTPUT_DIR = ROOT_DIR / "auxiliares" / "benchmark_runs"

# Mismo plan de decisiones en todos los episodios.
# Formato: (action_type, duration)
# 0=Hab1, 1=Hab2, 2=Hab3, 3=C, 4=stop
ACTION_PLAN = [
    (0, 0),
    (1, 0),
    (2, 0),
    (4, 5),
    (3, 0),
    (4, 2),
    (0, 0),
    (1, 0),
]


def build_episode_signature(step_trace):
    parts = []
    for s in step_trace:
        parts.append(
            f"{s['action_type']}:{s['duration']}|{s['node']}|{s['battery']:.1f}|"
            f"{s['status']}|{int(s['timed_out'])}|{int(s['terminated'])}|{int(s['truncated'])}"
        )
    payload = "||".join(parts)
    return hashlib.sha1(payload.encode("utf-8")).hexdigest()[:12]


def run_episode(env, episode_index):
    obs, info = env.reset(seed=SEED)
    start = perf_counter()

    steps = 0
    total_reward = 0.0
    terminated = False
    truncated = False
    step_trace = []

    for action in ACTION_PLAN:
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        steps += 1

        step_trace.append(
            {
                "step": steps,
                "action_type": int(action[0]),
                "duration": int(action[1]),
                "node": info.get("node", "unknown"),
                "battery": float(info.get("battery", 100.0)),
                "reward": float(reward),
                "status": info.get("last_action_final_status", "unknown"),
                "timed_out": bool(info.get("last_action_timed_out", False)),
                "terminated": bool(terminated),
                "truncated": bool(truncated),
            }
        )

        if terminated or truncated:
            break

    elapsed = perf_counter() - start
    signature = build_episode_signature(step_trace)

    return {
        "episode": episode_index,
        "steps": steps,
        "reward": total_reward,
        "elapsed": elapsed,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "final_node": info.get("node", "unknown"),
        "battery": float(info.get("battery", 100.0)),
        "timed_out_steps": sum(1 for s in step_trace if s["timed_out"]),
        "signature": signature,
        "trace": step_trace,
    }


def compute_summary(results):
    times = [r["elapsed"] for r in results]
    rewards = [r["reward"] for r in results]
    steps = [r["steps"] for r in results]
    batteries = [r["battery"] for r in results]
    terminated_count = sum(1 for r in results if r["terminated"])
    truncated_count = sum(1 for r in results if r["truncated"])
    timed_out_total = sum(r["timed_out_steps"] for r in results)
    node_counts = Counter(r["final_node"] for r in results)
    signature_counts = Counter(r["signature"] for r in results)

    return {
        "time_mean": mean(times),
        "time_std": pstdev(times),
        "time_total": sum(times),
        "time_min": min(times),
        "time_max": max(times),
        "reward_mean": mean(rewards),
        "steps_mean": mean(steps),
        "final_battery_mean": mean(batteries),
        "terminated_count": terminated_count,
        "truncated_count": truncated_count,
        "timed_out_steps_total": timed_out_total,
        "final_node_counts": dict(node_counts),
        "unique_signatures": len(signature_counts),
        "signature_counts": dict(signature_counts),
    }


def print_summary(summary):
    print("\nResumen")
    print(f"Tiempo medio: {summary['time_mean']:.3f}s")
    print(f"Desviacion tipica: {summary['time_std']:.3f}s")
    print(f"Tiempo total: {summary['time_total']:.3f}s")
    print(f"Minimo: {summary['time_min']:.3f}s")
    print(f"Maximo: {summary['time_max']:.3f}s")
    print(f"Reward medio: {summary['reward_mean']:.3f}")
    print(f"Pasos medios: {summary['steps_mean']:.2f}")
    print(f"Bateria final media: {summary['final_battery_mean']:.2f}%")
    print(f"Episodios terminated: {summary['terminated_count']}")
    print(f"Episodios truncated: {summary['truncated_count']}")
    print(f"Pasos con timeout de accion: {summary['timed_out_steps_total']}")
    print(f"Nodos finales: {summary['final_node_counts']}")
    print(f"Firmas unicas de episodio: {summary['unique_signatures']}")


def compare_with_reference(current, reference):
    print("\nComparacion con referencia")
    ref_summary = reference.get("summary", {})
    cur_summary = current.get("summary", {})

    def d(key):
        return cur_summary.get(key, 0.0) - ref_summary.get(key, 0.0)

    print(f"Delta tiempo medio: {d('time_mean'):+.3f}s")
    print(f"Delta reward medio: {d('reward_mean'):+.3f}")
    print(f"Delta pasos medios: {d('steps_mean'):+.3f}")
    print(f"Delta bateria final media: {d('final_battery_mean'):+.3f}%")
    print(f"Delta terminated: {d('terminated_count'):+.0f}")
    print(f"Delta timed_out_steps_total: {d('timed_out_steps_total'):+.0f}")

    ref_eps = {e["episode"]: e for e in reference.get("results", [])}
    cur_eps = {e["episode"]: e for e in current.get("results", [])}
    same_signature = 0
    shared = sorted(set(ref_eps.keys()) & set(cur_eps.keys()))
    for ep in shared:
        if ref_eps[ep].get("signature") == cur_eps[ep].get("signature"):
            same_signature += 1
    print(f"Episodios con firma identica: {same_signature}/{len(shared)}")


def main():
    env = RobotEnv(max_steps=MAX_STEPS, trace=TRACE)
    results = []

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    out_file = OUTPUT_DIR / f"benchmark_{RUN_TAG}.json"

    try:
        print("Benchmark de 10 episodios con la misma secuencia de acciones\n")
        print(f"RUN_TAG: {RUN_TAG}")
        print(f"Plan fijo: {ACTION_PLAN}\n")

        for ep in range(1, NUM_EPISODIOS + 1):
            result = run_episode(env, ep)
            results.append(result)
            print(
                f"EP {ep:02d}: tiempo={result['elapsed']:.3f}s | pasos={result['steps']:2d} | "
                f"reward={result['reward']:.2f} | nodo={result['final_node']} | bateria={result['battery']:.1f}% | "
                f"timeouts={result['timed_out_steps']} | sig={result['signature']}"
            )

        summary = compute_summary(results)
        print_summary(summary)

        payload = {
            "run_tag": RUN_TAG,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "config": {
                "num_episodes": NUM_EPISODIOS,
                "max_steps": MAX_STEPS,
                "seed": SEED,
                "action_plan": ACTION_PLAN,
            },
            "summary": summary,
            "results": results,
        }

        with out_file.open("w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

        print(f"\nResultados guardados en: {out_file}")

        if COMPARE_WITH_FILE:
            ref_path = Path(COMPARE_WITH_FILE)
            if not ref_path.is_absolute():
                ref_path = ROOT_DIR / ref_path
            if ref_path.exists():
                with ref_path.open("r", encoding="utf-8") as f:
                    reference = json.load(f)
                compare_with_reference(payload, reference)
            else:
                print(f"\nNo existe el archivo de referencia: {ref_path}")
    finally:
        env.close()


if __name__ == "__main__":
    main()
