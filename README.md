# TAMPEST

**TAMPEST** (Task and Motion Planning by Encoding into Satisfiability Testing) is a meta-framework for solving complex Task and Motion Planning (TAMP) problems. TAMP problems combine discrete task planning with low-level continuous motion planning and are fundamental for robotic autonomy in dynamic and partially known environments.

TAMPEST introduces a general open-source framework for modeling, solving, and benchmarking TAMP problems. It includes a specialization for **incremental SMT-based planning** via [TemPEST](https://github.com/fbk-pso/tempest), a temporal planner that serves as one of the back-end engines.

## Requirements

The first requirement is Python3 since the framework is written in python:
```bash
apt-get install python3-dev python3-pip
```

For the motion planning part [OMPL](https://ompl.kavrakilab.org/) is needed:
```bash
wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh --python
```

Other Python requirements:
```bash
pip3 install -r requirements.txt
```

TAMPEST relies on [TemPEST](https://github.com/fbk-pso/tempest) for the SMT-based task planning capabilities.

```bash
pip3 install git+https://github.com/fbk-pso/tempest.git
```

Solver dependencies for TemPEST (see [PySMT](https://github.com/pysmt/pysmt)):
```bash
pysmt-install --z3
```

One of the task planners that can be used for numeric planning is ENHSP. It requires JAVA:
```bash
apt-get install openjdk-17-jdk
```

## Installation

Install TAMPEST with `pip`:

```bash
pip3 install git+https://github.com/fbk-pso/tampest.git
```

## Usage

TAMPEST is fully integrated with the [Unified Planning](https://github.com/aiplan4eu/unified-planning) framework. You must register the planner engines with the Unified Planning environment:

```python
from unified_planning.shortcuts import *

# Register TAMPEST engines
env = get_environment()
env.factory.add_engine("tampest", "tampest.engine", "TampestEngine")
env.factory.add_meta_engine("tamp", "tampest.meta_engine", "TampMetaEngine")

problem = ...  # your task and motion planning problem

# Solve with TAMPEST engine (it support classical, numeric and temporal planning)
with OneshotPlanner(name="tampest") as planner:
    result = planner.solve(problem)
    print(result.plan)

# Solve with TAMPEST meta engine (it support classical and numeric planning)
with OneshotPlanner(name="tamp[enhsp]") as planner:
    result = planner.solve(problem)
    print(result.plan)
```

## References

- E. Tosello, A. Valentini, A. Micheli. *A Meta-Engine Framework for Interleaved Task and Motion Planning using Topological Refinements.* **ECAI 2024**

- E. Tosello, A. Valentini, A. Micheli. *Temporal Task and Motion Planning with Metric Time for Multiple Object Navigation.* **AAAI 2025**

## Reproducing Results from ECAI 2024

To reproduce the experimental results from the ECAI 2024 paper, use the following command-line interface:

```bash
python3 test.py --domain <domain-name> --tr <topological-refinement> --tp <task-planner-name> --mp <motion-planner> --dim <dim> --d <n> --c <m> [--capacity <robot_capacity>]
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

## Reproducing Results from AAAI 2025

To reproduce the experimental results from the AAAI 2025 paper, use the following command-line interface:

```bash
python3 test.py --domain <domain-name> --tr <topological-refinement> --r <r> --d <n> [--c <m>] [--n_pallets <n_pallets>] [--kit_size <kit_size> --n_kit <n_kit>] [--n_drivers <n_drivers>] [--n_tiles <n_tiles> --n_colors <n_colors>]
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

## License

TAMPEST is released under the GNU Lesser General Public License v3.0 (LGPL-3.0).
See the `LICENSE` file for full details.

## Contact

For questions, bug reports, or contributions, please open an issue on GitHub or contact the maintainers.
