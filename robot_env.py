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
    - Visitar una habitación (Hab1/Hab2/Hab3) por debajo de la media de visitas: +1.0
    - Visitar una habitación por encima de la media: 0.0 (sin penalización explícita)
    - Batería agotada (terminated): -10.0

    C entra en el cálculo de la media igual que las habitaciones.
    Si el agente va demasiado a C, sube la media y le cuesta más obtener +1.0
    en las habitaciones. Penalización implícita, sin -0.5 explícito.
    Las acciones nulas (ir a un nodo donde ya estás) no cuentan como paso.
    Si el agente acumula demasiadas acciones nulas, el episodio termina (truncated).
"""

import gymnasium as gym
import numpy as np
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


ACTION_TO_NODE = {
    0: 'Hab1',
    1: 'Hab2',
    2: 'Hab3',
    3: 'C'
}

NODE_TO_IDX = {
    'Hab1': 0,
    'Hab2': 1,
    'Hab3': 2,
    'C':    3,
    'R':    3
}

# C incluida en visit_counts: entra en el cálculo de la media
ROOM_NODES = ['Hab1', 'Hab2', 'Hab3']


class RobotCoppeliaSim:
    def __init__(self):
        self._client = RemoteAPIClient()
        self._sim = self._client.require('sim')
        if self._sim.getSimulationState() == self._sim.simulation_stopped:
            self._sim.startSimulation()
            time.sleep(1)

    def get_state(self):
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
        if isinstance(target_statuses, str):
            target_statuses = [target_statuses]
        start = time.time()
        while time.time() - start < timeout:
            state = self.get_state()
            if state['status'] in target_statuses:
                return state
            time.sleep(0.05)
        return self.get_state()

    def reset(self):
        self.send_command('reset')
        self.wait_for_status(['ready', 'idle'], timeout=15)

    def go_to(self, node):
        self.send_command(f'goTo:{node}')
        return self.wait_for_status(['arrived', 'idle', 'error', 'depleted'], timeout=120)

    def stop_simulation(self):
        self._sim.stopSimulation()


class RobotEnv(gym.Env):
    """
    Entorno Gymnasium para el robot de vigilancia en CoppeliaSim.
    EXP1: C entra en el cálculo de la media de visitas.
    Las acciones nulas (ir a un nodo donde ya estás) no cuentan como paso.
    Si se acumulan demasiadas acciones nulas, el episodio termina (truncated).
    """

    metadata = {"render_modes": []}

    def __init__(self, max_steps=50, trace=False):
        super().__init__()

        # Espacio de observación: [nodo(0-3), bateria(0-100), vis1, vis2, vis3, visC]
        self.observation_space = gym.spaces.Box(
            low  = np.array([0,    0.0, 0, 0, 0, 0], dtype=np.float32),
            high = np.array([3, 100.0, 999, 999, 999, 999], dtype=np.float32),
            dtype = np.float32
        )
        print("Observation space: {}".format(self.observation_space))

        self.action_space = gym.spaces.Discrete(4)
        print("Action space: {}".format(self.action_space))

        self._max_steps = max_steps
        self._trace     = trace
        self._numep             = -1
        self._numstepsinepisode = 0
        self._accreward         = 0.0
        self._visit_counts      = {'Hab1': 0, 'Hab2': 0, 'Hab3': 0, 'C': 0}
        self._last_null_action  = False
        self._total_iterations  = 0  # contador total incluyendo acciones nulas
        self._robot = RobotCoppeliaSim()

    def close(self):
        self._robot.stop_simulation()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._numep += 1
        self._numstepsinepisode = 0
        self._accreward = 0.0
        self._visit_counts = {'Hab1': 0, 'Hab2': 0, 'Hab3': 0, 'C': 0}
        self._last_null_action = False
        self._total_iterations = 0
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

        target_node = ACTION_TO_NODE[int(action)]

        if self._trace:
            print(f"\tAction: {action} -> goTo:{target_node}")

        current_state = self._robot.get_state()
        already_there = (current_state['node'] == target_node)

        final_state = self._robot.go_to(target_node)

        if already_there:
            self._total_iterations += 1
            if self._trace:
                print(f"\t(acción nula: ya estaba en {target_node}, no cuenta paso)")
            info = self._get_info()
            info['null_action'] = True
            # Límite de seguridad: si acumula demasiadas nulas, terminar episodio
            if self._total_iterations >= self._max_steps * 20:
                return self._get_obs(), 0.0, False, True, info
            return self._get_obs(), 0.0, False, False, info

        self._total_iterations += 1
        self._last_null_action = False

        if final_state['depleted']:
            reward     = -10.0
            terminated = True
        else:
            visitas_antes = self._visit_counts[target_node]
            self._visit_counts[target_node] += 1
            media = np.mean(list(self._visit_counts.values()))

            if target_node in ROOM_NODES:
                if visitas_antes < media:
                    reward = 1.0

        self._accreward += reward

        if self._trace:
            print(f"\treward={reward:.2f}  accrew={self._accreward:.2f}  "
                  f"battery={final_state['battery']}  node={final_state['node']}")

        observation = self._get_obs()
        info        = self._get_info()
        info['null_action'] = False

        self._numstepsinepisode += 1
        if self._numstepsinepisode >= self._max_steps:
            truncated = True

        return observation, reward, terminated, truncated, info

    def _get_obs(self):
        state = self._robot.get_state()
        node_idx = NODE_TO_IDX.get(state['node'], 3)
        battery  = float(state['battery'])
        vis1     = float(self._visit_counts['Hab1'])
        vis2     = float(self._visit_counts['Hab2'])
        vis3     = float(self._visit_counts['Hab3'])
        visC     = float(self._visit_counts['C'])
        return np.array([node_idx, battery, vis1, vis2, vis3, visC], dtype=np.float32)

    def _get_info(self, null_action=False):
        state = self._robot.get_state()
        return {
            'node':        state['node'],
            'battery':     float(state['battery']),
            'acc_reward':  float(self._accreward),
            'null_action': null_action,
        }