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

Acciones (Discrete(4)):
    0 -> goTo:Hab1
    1 -> goTo:Hab2
    2 -> goTo:Hab3
    3 -> goTo:C

Recompensa:
    - Visitar una habitación (Hab1/Hab2/Hab3) que esté por debajo de la media
      de visitas del episodio: +1.0
    - Batería agotada (terminated): -10.0
"""

import gymnasium as gym
import numpy as np
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# Mapeo acción (entero) -> nodo destino
ACTION_TO_NODE = {
    0: 'Hab1',
    1: 'Hab2',
    2: 'Hab3',
    3: 'C'
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

        return {
            'battery':  battery  if battery  is not None else 100,
            'node':     node     if node     else 'unknown',
            'status':   status   if status   else 'unknown',
            'depleted': depleted
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

        # Espacio de observación: [nodo(0-3), bateria(0-100), vis1, vis2, vis3, vis_c]
        self.observation_space = gym.spaces.Box(
            low  = np.array([0,    0.0, 0, 0, 0, 0], dtype=np.float32),
            high = np.array([3, 100.0, 999, 999, 999, 999], dtype=np.float32),
            dtype = np.float32
        )
        print("Observation space: {}".format(self.observation_space))

        # Espacio de acciones: 4 destinos posibles
        self.action_space = gym.spaces.Discrete(4)
        print("Action space: {}".format(self.action_space))

        # Parámetros del episodio
        self._max_steps = max_steps
        self._trace     = trace

        # Contadores internos (inicializados a valores válidos, no None)
        self._numep             = -1
        self._numstepsinepisode = 0
        self._accreward         = 0.0
        self._visit_counts      = {node: 0 for node in ALL_NODES}

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

        target_node = ACTION_TO_NODE[action]

        if self._trace:
            print(f"\tAction: {action} -> goTo:{target_node}")

        # Ejecutar acción en Coppelia y esperar resultado
        final_state = self._robot.go_to(target_node)

        # --- Calcular recompensa ---

        if final_state['depleted']:
            reward     = -10.0
            terminated = True
        elif target_node in ROOM_NODES + ['C']:
            self._visit_counts[target_node] += 1
            media = np.mean(list(self._visit_counts.values()))
            visitas_antes = self._visit_counts[target_node] - 1
            if visitas_antes < media - (1 / len(ALL_NODES)):
                reward = 1.0
        else:
            self._visit_counts[target_node] += 1

        self._accreward += reward

        if self._trace:
            print(f"\treward={reward:.2f}  accrew={self._accreward:.2f}  "
                  f"battery={final_state['battery']}  node={final_state['node']}")

        observation = self._get_obs()
        info        = self._get_info()

        self._numstepsinepisode += 1
        if self._numstepsinepisode >= self._max_steps:
            truncated = True

        return observation, reward, terminated, truncated, info

    # ----- métodos privados -----

    def _get_obs(self):
        state = self._robot.get_state()
        node_idx = NODE_TO_IDX.get(state['node'], 3)
        battery  = float(state['battery'])
        vis = [float(self._visit_counts[node]) for node in ALL_NODES]
        return np.array([node_idx, battery] + vis, dtype=np.float32)

    def _get_info(self):
        """Información auxiliar para logging."""
        state = self._robot.get_state()
        return {
            'node':       state['node'],
            'battery':    float(state['battery']),
            'acc_reward': float(self._accreward),
        }
