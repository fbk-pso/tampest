# TAMPEST

**TAMPEST** (Task and Motion Planning by Encoding into Satisfiability Testing) is a meta-framework for solving complex Task and Motion Planning (TAMP) problems, which combine discrete task planning with low-level continuous motion planning and are fundamental for robotic autonomy in dynamic and partially known environments.

TAMPEST provides a general open-source framework for modeling, solving, and benchmarking TAMP problems. It supports **classical, numeric, and temporal** task and motion planning, as well as **scheduling + motion planning**. It is built on top of the [Unified Planning](https://github.com/aiplan4eu/unified-planning) framework and includes a specialization for **incremental SMT-based planning** via [TemPEST](https://github.com/fbk-pso/tempest), a temporal planner that serves as one of the back-end engines.


## Installation

Install the system dependencies:
```bash
apt-get install python3-dev python3-pip
```

Install [Unified Planning](https://github.com/aiplan4eu/unified-planning), the framework TAMPEST builds on to model problems and register its engines:
```bash
pip3 install --pre unified-planning
```

Install TAMPEST itself, which pulls in the core dependencies (including [OMPL](https://ompl.kavrakilab.org/), [TemPEST](https://github.com/fbk-pso/tempest), and [PySMT](https://github.com/pysmt/pysmt)):
```bash
pip3 install "tampest @ git+https://github.com/fbk-pso/tampest.git"
```

Install the SMT solver required by TemPEST:
```bash
pysmt-install --z3
```

Optionally, install the visualization libraries (`matplotlib`, `scipy`, `pyvista`):
```bash
pip3 install "tampest[plot] @ git+https://github.com/fbk-pso/tampest.git"
```

### Optional: task planners for meta-engines

`TampMetaEngine` can wrap any UP-compatible task planner. Install only the ones you need:
```bash
# fast-downward, ENHSP, or Tamer (as UP extras)
pip3 install --pre unified-planning[fast-downward]
pip3 install --pre unified-planning[enhsp]
pip3 install --pre unified-planning[tamer]
```

ENHSP requires Java:
```bash
apt-get install openjdk-17-jdk
```

`SampMetaEngine` requires a scheduling engine. Install only the one you need:
```bash
# CPSE
pip3 install git+https://github.com/fbk-pso/cpse.git
# Aries
pip3 install https://github.com/plaans/aries/releases/download/latest/up_aries.tar.gz
```

## Usage

TAMPEST is fully integrated with the [Unified Planning](https://github.com/aiplan4eu/unified-planning) framework. You must register the planner engines with the Unified Planning environment:

```python
from unified_planning.shortcuts import *

# Register TAMPEST engines
env = get_environment()
env.factory.add_engine("tampest", "tampest.engine", "TampestEngine")
env.factory.add_meta_engine("tamp", "tampest.meta_engine", "TampMetaEngine")
env.factory.add_meta_engine("samp", "tampest.meta_engine_samp", "SampMetaEngine")

problem = ...  # your task and motion planning problem

# Solve with TAMPEST engine (it support classical, numeric and temporal planning)
with OneshotPlanner(name="tampest") as planner:
    result = planner.solve(problem)
    print(result.plan)

# Solve with TAMPEST meta engine (it support classical and numeric planning)
with OneshotPlanner(name="tamp[enhsp]") as planner:
    result = planner.solve(problem)
    print(result.plan)

# Solve a SchedulingMotionProblem with the SAMP meta engine
# (wraps any UP-compatible scheduling engine, e.g. CPSE or aries)
with OneshotPlanner(name="samp[cpse]") as planner:
    result = planner.solve(problem)
    print(result.plan)
```

## Reproducing Results

The paper experiments are driven by `run.py` and the problem definitions under `benchmarks/`, which live in the repository. Clone it first:

```bash
git clone https://github.com/fbk-pso/tampest.git
cd tampest
```

If you did not install TAMPEST with `pip` (see [Installation](#installation)), you can instead install the dependencies from the clone with `pip3 install -r requirements.txt`. For the meta-engine experiments, also install the task or scheduling planner required by the target paper (see [Optional: task planners for meta-engines](#optional-task-planners-for-meta-engines)).

### ECAI 2024

To reproduce the experimental results from the ECAI 2024 paper, use the following command-line interface:

```bash
python3 run.py --domain <domain-name> --tr <topological-refinement> --tp <task-planner-name> --mp <motion-planner> --dim <dim> --d <n> --c <m> [--capacity <robot_capacity>]
```

The possible domains are: `doors`, `maze`, `delivery`, `rover`.

The possible topological refinements are: `none`, `unreach`, `obs`, `all`.

The possible task planners are: `tampest`, `tamer`, `enhsp`, `fast-downward`.

The possible motion planners are: `LazyRRT`, `RRT`.

The possible dimensions are: `2D`, `3D`. `3D` setups are available only for `maze` and `rover`.

The possible set of benchmark options are:

* `doors`: `--d [1 2 4 6 8 10] --c [0 1 2 3]`, with *d* the number of doors to be open and ```c = {0 = [(0,0)], 1 = [(10,0)], 2 = [(0,10)], 3 = [(5,5)]}``` the number of extra configurations sampled within the initially reachable space, the unreachable space, or equally splitted between both;

* `maze`: `--d [1 2 3 4 5 6 7 8 9 10] --c [0 1 2 3 4 5 6 7 8 9 10]`, with *d* the number of closed doors and *c* the number of locations to be visited;

* `delivery`: `--d [1 2 4 6 8 10] --c [0 2 4 ... 48] --capacity [4 3 2 1]`, with *d* the number of closed doors, *capacity* the loading capacity of the robot, and *c* the configuration of the parcels. In detail, ```c = n = [nr, ng, dr, dg]```, where *nr* is the total number of red parcels, *ng* is the total number of green parcels, *dr* is the number of red parcels already at their delivery stations, and *dg* is the number of green parcels already at their delivery stations;

* `rover`: `--d [2 4 6 8 10] --c [0 1 2 3 4]`, with *d* the number of samples (equally splitted between soils and rocks, each one separated from the robot by a closed door) and *c* the number of objectives to be photographed around the sample.

*Refer to the paper for the detailed meaning of the domain parameters.*

See [here](https://github.com/fbk-pso/up-pddl-stream/) to reproduce the results related to `up-pddl-stream`.

### AAAI 2025

To reproduce the experimental results from the AAAI 2025 paper, use the following command-line interface:

```bash
python3 run.py --domain <domain-name> --tr <topological-refinement> --r <r> --d <n> [--c <m>] [--n_pallets <n_pallets>] [--kit_size <kit_size> --n_kit <n_kit>] [--n_drivers <n_drivers>] [--n_tiles <n_tiles> --n_colors <n_colors>]
```

The possible domains are: `tdoors`, `majsp`, `kitting`, `driverlog`, `floortile`.

The possible topological refinements are: `none`, `unreach`, `obs`, `all`.

The possible set of benchmark options are:

* `tdoors`: `--r [1 2 3] --d [1 2 4 6] --c [0 1 2 3]`, with *r* the number of robots, *d* the number of doors to be open and ```c = {0 = [(0,0)], 1 = [(10,0)], 2 = [(0,10)], 3 = [(5,5)]}``` the number of extra configurations sampled within the initially reachable space, the unreachable space, or equally splitted between both;

* `majsp`: `--r [1 2 3] --d [1 2 4 6] --n_pallets [1 2 3]`, with *r* the number of robots, *d* the number of closed doors and *n_pallets* the number of pallets to treat.

* `kitting`: `--r [1 2 3] --d [1 2 4 6] --kit_size [1 2 3] --n_kit [1 2 3]`, with *r* the number of robots, *d* the number of closed doors, *kit_size* the max size of the kits and *n_kit* the number of kits to collect.

* `driverlog`: `--r [1 2 3] --d [1 2 4 6] --n_drivers [1 2 3]`, with *r* the number of robots, *d* the number of closed doors and *n_drivers* the number of available drivers.

* `floortile`: `--r [1 2 3] --d [1 2 4 6] --n_tiles [1 2 4 6 8] --n_colors [1 2 3]`, with *r* the number of robots, *d* the number of closed doors, *n_tiles* the number of tiles to paint and *n_colors* the number of available colors.

*Refer to the paper for the detailed meaning of the domain parameters.*

### ICAPS 2026

The ICAPS 2026 paper adds a scheduling+motion-planning pipeline on top of TAMPEST, exposed via the `SampMetaEngine`. It wraps a UP-compatible scheduling engine and interleaves it with the existing motion-planning stack.

```bash
python3 run.py --tp <TP> [--use_fluents] [--opt] --domain <DOMAIN> \
    [--d <D>] [--r <R>] [--n_pallets <N_PALLETS>] \
    [--n_components <N_COMPONENTS>] [--use_external_locations]
```

The possible task planners are: `samp[aries]`, `samp[aries-opt]`, `samp[cpse]`.

The possible domains are:

* `jsp` (Job Shop Problem): `--r [1 2 3] --d [1 2 4 6] --n_pallets [1 2 3]`, with *r* the number of robots, *d* the number of machines closed by a door, and *n_pallets* the number of pallets to treat.

* `logistics`: `--r [1 2 3] --d [0 1] --n_shelves [2] --n_components [1 2 3 4 5 6 7 8]`, with *r* the number of robots, *d* a flag for the presence of doors (0 = no doors, 1 = doors present), *n_shelves* the number of shelves and *n_components* the number of components to pick up. Pass `--use_external_locations` to give each shelf both internal and external locations.

Additional options:

* `--use_fluents`: enable fluent-based modeling of configuration occupancy.
* `--opt`: enable makespan minimization (when supported by the selected scheduler).

*Refer to the paper for the detailed meaning of the domain parameters.*

## References

- E. Tosello, A. Valentini, A. Micheli. *A Meta-Engine Framework for Interleaved Task and Motion Planning using Topological Refinements.* **ECAI 2024**

- E. Tosello, A. Valentini, A. Micheli. *Temporal Task and Motion Planning with Metric Time for Multiple Object Navigation.* **AAAI 2025**

- E. Tosello, A. Bit-Monnot, D. Lusuardi, A. Valentini, A. Micheli. *Interleaving Scheduling and Motion Planning with Incremental Learning of Symbolic Space-Time Motion Abstractions.* **ICAPS 2026**

## License

TAMPEST is released under the GNU General Public License v3.0 (GPL-3.0).
See the `LICENSE` file for full details.

## Contact

For questions, bug reports, or contributions, please open an issue on GitHub or contact the maintainers.
