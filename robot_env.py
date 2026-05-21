"""
Entorno Gymnasium para el control de alto nivel del robot de vigilancia.
Tarea simplificada: el robot debe visitar Hab1, Hab2, Hab3 y gestionar su batería
yendo a C cuando sea necesario. El episodio termina si la batería llega a 0.

Observación:
    - nodo_actual   : entero 0-3 (Hab1=0, Hab2=1, Hab3=2, C=3)
    - bateria       : float 0.0-100.0
    - visitas_hab1  : entero (veces visitada en el episodio)
    - visitas_hab2  : entero
    - visitas_hab3  : entero
    - visitas_c     : entero
    - tiempo_ep     : float 0.0-1.0 (progreso normalizado del episodio)

Acciones (MultiDiscrete([5, 51])):
    action[0] (tipo de acción):
        0 -> goTo:Hab1
        1 -> goTo:Hab2
        2 -> goTo:Hab3
        3 -> goTo:C
        4 -> stop (duración en action[1])
    action[1] (duración si es stop):
        0-50 segundos (ignorado si acción != 4)

Recompensa:
    - exp_011 (dispersa con C): recompensa 0.0 en pasos intermedios.
    - Al final por truncación: suma de dos términos:
      1) Balance habitaciones en [0, 10]: 10 * ((2/3 - desbalanceo) / (2/3))
      2) Penalización cuadrática C: -10 * p_C^2 donde p_C = visitas_C / suma(visitas_totales)
    - Batería agotada (terminated): -10.0
"""

import gymnasium as gym
import numpy as np
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# Mapeo acción (entero) -> nodo destino o acción especial
ACTION_TO_NODE = {
    0: 'Hab1',
    1: 'Hab2',
    2: 'Hab3',
    3: 'C',
    4: 'stop'  # Acción especial: parada
}

# Mapeo nodo -> entero para la observación (R excluido, el agente no vuelve ahí)
NODE_TO_IDX = {
    'Hab1': 0,
    'Hab2': 1,
    'Hab3': 2,
    'C':    3,
    'R':    3   # Solo ocurre al inicio; lo tratamos como C a efectos de observación
}

ROOM_NODES = ['Hab1', 'Hab2', 'Hab3']  # Nodos con recompensa de visita
ALL_NODES = ['Hab1', 'Hab2', 'Hab3', 'C']  # Todos los nodos visitables
C_PENALTY_COEFF = 50  # Coeficiente de penalización cuadrática para uso de C (exp_012: 50, exp_011: 10)


class RobotCoppeliaSim:
    """
    Capa de comunicación con CoppeliaSim via ZeroMQ.
    Encapsula toda la lógica de señales para mantener RobotEnv limpio.
    """

    def __init__(self):
        self._client = RemoteAPIClient()
        self._sim = self._client.require('sim')

        # Arrancar simulación si está parada
        if self._sim.getSimulationState() == self._sim.simulation_stopped:
            self._sim.startSimulation()
            time.sleep(1)

    def get_state(self):
        """Devuelve el estado actual del robot como diccionario."""
        battery  = self._sim.getInt32Signal('robot_battery')
        node     = self._sim.getStringSignal('robot_currentNode')
        status   = self._sim.getStringSignal('robot_status')
        depleted = self._sim.getInt32Signal('robot_batteryDepleted') == 1
        action_duration = self._sim.getFloatSignal('robot_last_action_duration')

        return {
            'battery':  battery  if battery  is not None else 100,
            'node':     node     if node     else 'unknown',
            'status':   status   if status   else 'unknown',
            'depleted': depleted,
            'last_action_duration': action_duration if action_duration is not None else 0.0
        }

    def send_command(self, command):
        self._sim.setStringSignal('robot_command', command)

    def wait_for_status(self, target_statuses, timeout=120):
        """Bloquea hasta que el robot alcanza uno de los estados objetivo."""
        if isinstance(target_statuses, str):
            target_statuses = [target_statuses]
        start = time.time()
        while time.time() - start < timeout:
            state = self.get_state()
            if state['status'] in target_statuses:
                return state
            time.sleep(0.05)
        # Timeout: devolver estado actual igualmente
        return self.get_state()

    def reset(self):
        """Reinicia el entorno en Coppelia y espera confirmación."""
        self.send_command('reset')
        self.wait_for_status(['ready', 'idle'], timeout=15)

    def go_to(self, node):
        """Envía al robot a un nodo y espera a que llegue o falle."""
        self.send_command(f'goTo:{node}')
        return self.wait_for_status(['arrived', 'idle', 'error', 'depleted'], timeout=120)

    def stop(self, duration):
        """Pausa el robot durante duration segundos."""
        self.send_command(f'stop:{duration}')
        # Confirmación: en modo teleport puede pasar muy rápido a 'idle'
        ack_state = self.wait_for_status(['stopped', 'idle', 'error', 'depleted'], timeout=5)
        if ack_state['status'] in ['idle', 'error', 'depleted']:
            return ack_state
        # Esperar a que termine la parada y vuelva a 'idle'
        stop_timeout = (duration + 10) if duration else 30
        return self.wait_for_status(['idle', 'error', 'depleted'], timeout=stop_timeout)

    def stop_simulation(self):
        self._sim.stopSimulation()


class RobotEnv(gym.Env):
    """
    Entorno Gymnasium para el robot de vigilancia en CoppeliaSim.
    Tarea: visitar habitaciones eficientemente gestionando la batería.
    """

    metadata = {"render_modes": []}

    def __init__(self, max_steps=50, trace=False):
        super().__init__()

        # Espacio de observación: [nodo(0-3), bateria(0-100), vis1, vis2, vis3, vis_c, tiempo_ultima_accion_s]
        self.observation_space = gym.spaces.Box(
            low  = np.array([0,    0.0, 0, 0, 0, 0, 0.0], dtype=np.float32),
            high = np.array([3, 100.0, 999, 999, 999, 999, 180.0], dtype=np.float32),
            dtype = np.float32
        )
        print("Observation space: {}".format(self.observation_space))

        # Espacio de acciones: MultiDiscrete([5, 51])
        # Componente 0: acción (0=Hab1, 1=Hab2, 2=Hab3, 3=C, 4=stop)
        # Componente 1: duración en segundos si es stop (0-50)
        self.action_space = gym.spaces.MultiDiscrete([5, 51])
        print("Action space: {}".format(self.action_space))

        # Parámetros del episodio
        self._max_steps = max_steps
        self._trace     = trace

        # Contadores internos (inicializados a valores válidos, no None)
        self._numep             = -1
        self._numstepsinepisode = 0
        self._accreward         = 0.0
        self._visit_counts      = {node: 0 for node in ALL_NODES}
        self._last_action_type  = None
        self._last_action_name  = 'reset'
        self._last_stop_duration = 0
        self._last_action_elapsed_s = 0.0
        self._episode_elapsed_s = 0.0
        self._last_action_timed_out = False
        self._last_action_final_status = 'idle'

        # Conexión con CoppeliaSim
        self._robot = RobotCoppeliaSim()

    def close(self):
        self._robot.stop_simulation()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self._numep += 1
        self._numstepsinepisode = 0
        self._accreward = 0.0
        self._visit_counts = {node: 0 for node in ALL_NODES}
        self._last_action_type  = None
        self._last_action_name  = 'reset'
        self._last_stop_duration = 0
        self._last_action_elapsed_s = 0.0
        self._episode_elapsed_s = 0.0
        self._last_action_timed_out = False
        self._last_action_final_status = 'idle'

        self._robot.reset()

        observation = self._get_obs()
        info        = self._get_info()

        if self._trace:
            print(f"[reset] ep={self._numep} obs={observation}")

        return observation, info

    def step(self, action):
        terminated = False
        truncated  = False
        reward     = 0.0

        # Desempacar acción hybrid: [acción_tipo, duración]
        action_type = int(action[0])
        stop_duration = int(action[1]) if len(action) > 1 else 0

        action_name = ACTION_TO_NODE.get(action_type, 'unknown')
        self._last_action_type = action_type
        self._last_action_name = action_name
        self._last_stop_duration = stop_duration if action_name == 'stop' else 0

        if self._trace:
            if action_name == 'stop':
                print(f"\tAction: {action_type} -> stop:{stop_duration}s")
            else:
                print(f"\tAction: {action_type} -> goTo:{action_name}")

        # Ejecutar acción en Coppelia y esperar resultado
        if action_name == 'stop':
            final_state = self._robot.stop(stop_duration)
            expected_statuses = ['idle', 'error', 'depleted']
        else:
            final_state = self._robot.go_to(action_name)
            expected_statuses = ['arrived', 'idle', 'error', 'depleted']

        self._last_action_final_status = final_state['status']
        self._last_action_timed_out = final_state['status'] not in expected_statuses
        self._last_action_elapsed_s = float(final_state.get('last_action_duration', 0.0))
        self._episode_elapsed_s += self._last_action_elapsed_s

        # --- Calcular recompensa ---

        if final_state['depleted']:
            reward     = -10.0
            terminated = True
        else:
            # Solo contar visitas a nodos, no a 'stop'
            if action_name != 'stop':
                self._visit_counts[action_name] += 1

        self._numstepsinepisode += 1
        if (not terminated) and self._numstepsinepisode >= self._max_steps:
            truncated = True
            reward = self._room_balance_reward() + self._c_quadratic_penalty()

        self._accreward += reward

        observation = self._get_obs()
        info        = self._get_info()

        if self._trace:
            print(f"\treward={reward:.2f}  accrew={self._accreward:.2f}  "
                  f"battery={final_state['battery']}  node={final_state['node']}")

        return observation, reward, terminated, truncated, info

    # ----- métodos privados -----

    def _get_obs(self):
        state = self._robot.get_state()
        node_idx = NODE_TO_IDX.get(state['node'], 3)
        battery  = float(state['battery'])
        vis = [float(self._visit_counts[node]) for node in ALL_NODES]
        last_action_elapsed = float(np.clip(self._last_action_elapsed_s, 0.0, 180.0))
        return np.array([node_idx, battery] + vis + [last_action_elapsed], dtype=np.float32)

    def _get_info(self):
        """Información auxiliar para logging."""
        state = self._robot.get_state()
        return {
            'node':       state['node'],
            'battery':    float(state['battery']),
            'acc_reward': float(self._accreward),
            'last_action_type': self._last_action_type,
            'last_action_name': self._last_action_name,
            'last_stop_duration': int(self._last_stop_duration),
            'last_action_elapsed_s': float(self._last_action_elapsed_s),
            'episode_elapsed_s': float(self._episode_elapsed_s),
            'last_action_timed_out': bool(self._last_action_timed_out),
            'last_action_final_status': self._last_action_final_status,
        }

    def _room_balance_reward(self):
        """Recompensa dispersa final basada en ajuste a distribución uniforme de habitaciones."""
        visits = np.array([self._visit_counts[node] for node in ROOM_NODES], dtype=np.float32)
        total_visits = float(np.sum(visits))

        if total_visits <= 0.0:
            return 0.0

        probabilities = visits / total_visits
        num_rooms = float(len(ROOM_NODES))
        desbalanceo = float(np.sum((1.0 / num_rooms - probabilities) ** 2))
        max_desbalanceo = (num_rooms - 1.0) / num_rooms

        if max_desbalanceo <= 0.0:
            return 0.0

        normalized_reward = (max_desbalanceo - desbalanceo) / max_desbalanceo
        return float(10.0 * np.clip(normalized_reward, 0.0, 1.0))

    def _c_quadratic_penalty(self):
        """Penalización dispersa final cuadrática por uso de C."""
        all_visits = float(sum(self._visit_counts[node] for node in ALL_NODES))
        if all_visits <= 0.0:
            return 0.0

        c_frequency = float(self._visit_counts['C']) / all_visits
        return float(-C_PENALTY_COEFF * (c_frequency ** 2))