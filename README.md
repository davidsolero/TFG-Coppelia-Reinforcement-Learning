# TFG - Aprendizaje por Refuerzo para Robot de Vigilancia en CoppeliaSim

Este repositorio contiene el desarrollo completo del TFG centrado en el entrenamiento de un agente de Aprendizaje por Refuerzo (RL) para control de alto nivel de un robot de vigilancia en CoppeliaSim.

El agente aprende a:
- Visitar de forma equilibrada las habitaciones Hab1, Hab2 y Hab3.
- Gestionar la energia y decidir cuando ir al nodo de carga C.
- Evitar terminar episodios por agotamiento de bateria.

Tecnologias principales:
- Python + Gymnasium para el entorno RL.
- Stable-Baselines3 (PPO) para el entrenamiento del agente.
- CoppeliaSim + script Lua para la simulacion del robot.
- TensorBoard, pandas y matplotlib para analisis y visualizacion.

## 1. Estructura del proyecto

```text
TFG-Coppelia-Reinforcement-Learning/
|- entornoRobotVigilancia.ttt           # Escena de CoppeliaSim
|- script pioneer.lua                   # Script Lua del robot en CoppeliaSim
|- robot_env.py                         # Entorno Gymnasium conectado a Coppelia
|- train.py                             # Entrenamiento PPO
|- evaluate.py                          # Evaluacion corta por episodios
|- evaluate_deep.py                     # Evaluacion profunda (500+ episodios)
|- experiments/                         # Resultados por experimento
|- scripts/                             # Automatizaciones .bat (Windows)
`- auxiliares/                          # Utilidades de apoyo
```

Subcarpetas relevantes:
- `experiments/<nombre_experimento>/model/`: modelos `.zip` entrenados.
- `experiments/<nombre_experimento>/train_logs/`: logs para TensorBoard.
- `experiments/<nombre_experimento>/deep_evaluation/`: CSV y analisis de evaluacion profunda.
- `auxiliares/requirements.txt`: dependencias Python usadas en el proyecto.

## 2. Arquitectura y flujo tecnico

El sistema se divide en dos capas:

1. Capa simulacion (Lua en CoppeliaSim)
- `script pioneer.lua` controla cinematica, bateria, grafo de rutas y estado del robot.
- Publica señales de estado: bateria, nodo actual, estado interno, etc.
- Recibe comandos desde Python: `goTo:<nodo>`, `reset`, `stop`, `callOperator`.

2. Capa RL (Python)
- `robot_env.py` implementa `gymnasium.Env`.
- `train.py` entrena PPO sobre ese entorno.
- `evaluate.py` y `evaluate_deep.py` ejecutan evaluaciones sin aprendizaje (`deterministic=True`).

Flujo de una accion RL:
- El agente elige una accion discreta (`Hab1`, `Hab2`, `Hab3`, `C`).
- Python envia `goTo:<destino>` por señal remota.
- Lua ejecuta el desplazamiento en la simulacion y actualiza estado.
- Python lee estado final y calcula recompensa en `robot_env.py`.

## 3. Entorno RL definido en `robot_env.py`

### 3.1 Espacio de acciones
- Discrete(4)
- `0 -> Hab1`
- `1 -> Hab2`
- `2 -> Hab3`
- `3 -> C`

### 3.2 Espacio de observaciones
Vector de 6 valores:
- nodo actual indexado (Hab1, Hab2, Hab3, C)
- bateria (%)
- contador de visitas Hab1
- contador de visitas Hab2
- contador de visitas Hab3
- contador de visitas C

### 3.3 Finalizacion de episodio
- `terminated=True` si la bateria llega a 0.
- `truncated=True` al superar `max_steps` (por defecto 50).

### 3.4 Recompensa principal (experimento actual)
En la configuracion activa (`exp_005`):
- `+1.0` por visitar Hab1/Hab2/Hab3 cuando esa habitacion esta por debajo de la media de visitas.
- Penalizacion por ir a C adaptativa segun bateria:
	- bateria > 70%: `-0.4`
	- 40% < bateria <= 70%: `-0.2`
	- bateria <= 40%: `-0.05`
- `-10.0` si la bateria se agota.

## 4. Requisitos

### 4.1 Software
- Windows 10/11.
- Python 3.10+ (recomendado 3.10 u 3.11).
- CoppeliaSimEdu instalado.
- Escena `entornoRobotVigilancia.ttt` preparada con el script Lua cargado.

### 4.2 Dependencias Python
El listado completo esta en:
- `auxiliares/requirements.txt`

Paquetes clave:
- `stable_baselines3`
- `gymnasium`
- `coppeliasim_zmqremoteapi_client`
- `tensorboard`
- `pandas`
- `matplotlib`
- `numpy`
- `torch`

## 5. Instalacion

Desde la raiz del repositorio:

```bash
python -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r auxiliares/requirements.txt
```

Si usas Conda:

```bash
conda create -n tfg_rl python=3.10 -y
conda activate tfg_rl
pip install -r auxiliares/requirements.txt
```

## 6. Preparacion de CoppeliaSim

1. Abrir CoppeliaSim.
2. Cargar `entornoRobotVigilancia.ttt`.
3. Verificar que el robot Pioneer tiene asociado `script pioneer.lua`.
4. Comprobar que existen los nodos y paths del grafo (`R`, `Hab1`, `Hab2`, `Hab3`, `C`).

Nota:
- Los scripts `.bat` de `scripts/` apuntan por defecto a:
	- `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe`
- Si tu instalacion esta en otra ruta, modifica esa variable en los `.bat`.

## 7. Entrenamiento

Script principal:

```bash
python train.py
```

Parametros importantes en `train.py`:
- `EXP_NAME`: nombre de carpeta de experimento en `experiments/`.
- `TOTAL_TIMESTEPS`: pasos totales de entrenamiento.
- `MAX_STEPS_EP`: maximo de pasos por episodio.
- `RESUME_TRAINING`: reanuda desde modelo previo si existe.

Salida esperada:
- Modelo en `experiments/<EXP_NAME>/model/ppo_robot.zip`
- Logs TensorBoard en `experiments/<EXP_NAME>/train_logs/`

## 8. Monitorizacion con TensorBoard

Puedes usar:

```bash
python -m tensorboard.main --logdir ./experiments/
```

O directamente el script:

```bat
scripts\abrir tensorflow.bat
```

## 9. Evaluacion del agente

### 9.1 Evaluacion rapida

```bash
python evaluate.py
```

Caracteristicas:
- Ejecuta 10 episodios por defecto.
- Politica determinista (`deterministic=True`).
- Muestra por episodio reward, pasos, habitaciones visitadas, cargas y agotamiento.

Tambien disponible automatizado en Windows:

```bat
scripts\evaluacion.bat
```

### 9.2 Evaluacion profunda

```bash
python evaluate_deep.py
```

Caracteristicas:
- 500 episodios por defecto (ajustable).
- Calcula metricas agregadas robustas.
- Exporta CSV a:
	- `experiments/<EXP_NAME>/deep_evaluation/deepevaluation_results.csv`
- Muestra histogramas y boxplot al finalizar.

Automatizado en headless:

```bat
scripts\deep evaluacion.bat
```

## 10. Experimentos realizados

La carpeta `experiments/` contiene la evolucion del diseno de recompensa:

1. `exp_001_MediaSoloHabs`
- Recompensa basada en media de Hab1/Hab2/Hab3.
- Sin penalizacion por ir a C.

2. `exp_001_MediaSoloHabsRemakeobs`
- Variante de `exp_001` con observacion reconstruida a 6 componentes.

3. `exp_002_MediaHabsCsinReward`
- La media incluye C, pero C no aporta +1.

4. `exp_003_MediaHabsCconReward`
- La media incluye C y C si aporta +1.

5. `exp_004_MediaHabSinCyConPenal`
- Media sobre habitaciones principales.
- Penalizacion fija por ir a C (`-0.1`).

6. `exp_005_MediaHabSinCyConPenalAdaptativa` (actual)
- Media sobre habitaciones principales.
- Penalizacion adaptativa por ir a C segun nivel de bateria.

## 11. Scripts auxiliares

En `auxiliares/`:
- `RobotController.py`: control manual y pruebas de comandos remotos.
- `test_episodes.py`: simulacion de episodios de prueba con resets.
- `test conexion pyhton.py`: verificacion basica de comunicacion Python-Coppelia.
- `plot_from_csv.py`: reconstruccion de graficas a partir del CSV de deep evaluation.

## 12. Ejecucion recomendada extremo a extremo

1. Configura `EXP_NAME` en `train.py`, `evaluate.py` y `evaluate_deep.py` al mismo valor.
2. Arranca CoppeliaSim con la escena cargada (o usa scripts `.bat`).
3. Ejecuta entrenamiento:

```bash
python train.py
```

4. Revisa curvas en TensorBoard.
5. Ejecuta evaluacion rapida:

```bash
python evaluate.py
```

6. Ejecuta evaluacion profunda:

```bash
python evaluate_deep.py
```

7. Analiza CSV y graficas para comparar experimentos.

## 13. Problemas frecuentes

1. `No se encontro el modelo ... ppo_robot.zip`
- El `EXP_NAME` no coincide entre entrenamiento y evaluacion.
- No se completo entrenamiento o se guardo en otra carpeta.

2. No conecta con CoppeliaSim
- Verifica que CoppeliaSim esta ejecutandose con la escena correcta.
- Comprueba que el script Lua del robot esta activo.
- Revisa que el plugin ZeroMQ Remote API esta disponible en tu instalacion.

3. Los `.bat` no arrancan CoppeliaSim
- Ajusta la ruta en `scripts/*.bat` a tu instalacion local.

4. Evaluacion profunda muy lenta
- Reduce `NUM_EPISODES` temporalmente para depuracion.
- Usa modo headless (`scripts/deep evaluacion.bat`) para ejecuciones largas.

## 14. Mejoras futuras propuestas

- Ajuste automatico de hiperparametros PPO.
- Registro unificado de metricas por experimento (JSON/CSV resumen).
- Benchmarks comparativos entre funciones de recompensa.
- Evaluaciones con seeds multiples para mayor robustez estadistica.

## 15. Autor y contexto academico

Trabajo desarrollado como TFG sobre control inteligente y aprendizaje por refuerzo aplicado a robotica de vigilancia en entorno simulado.

Si necesitas, se puede ampliar este README con una seccion especifica de metodologia experimental y tablas comparativas finales para inclusion directa en la memoria.