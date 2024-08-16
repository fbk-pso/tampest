Licence
-------

These files are released with the sole purpose of
reproducing/inspecting the experimental analysis of the paper
"A Meta-Engine Framework for Interleaved Task and Motion Planning using Topological Refinements".

Any other use is forbidden without prior written consent from the authors.

Info
-----

The code has been tested on Ubuntu 22.04

Prerequisites
-------------

The first requirement is Python3 since the framework is written in python:
```
apt-get install python3-dev python3-pip
```

For the motion planning part [OMPL](https://ompl.kavrakilab.org/) is needed:
```
wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh --python
```

Other Python requirements:
```
pip3 install -r requirements.txt
```

Requirements needed by the SMT-based task planner:
```
apt-get install libgmp3-dev
pysmt-install --msat --confirm-agreement
```

Requirements needed by the ENHSP task planner:
```
apt-get install openjdk-17-jdk
```

For the PDDLStream integration, [PDDLStream](https://github.com/caelan/pddlstream/tree/main) is needed:
```
cd up-pddl-stream
git clone --recursive --branch main https://github.com/caelan/pddlstream.git
cd pddlstream
git submodule update --init --recursive
./downward/build.py
```

Install
-------

Tempest
```
cd tempest
pip3 install .
```

Tampest
```
cd tampest
pip3 install .
```


Commands
--------

1) To run a Tampest test the command is:

	```
	python3 tampest/test.py --domain <domain-name> --dim <dim> --d <n> --c <m> [--capacity <robot_capacity>] --tp <task-planner-name> --mp <motion-planner> --tr <topological-refinement>
	```

	The possible domains are: `doors`, `maze`, `delivery`, `rover`.
	
	The possible dimensions are: `2D`, `3D`. `3D` setups are available only for `maze` and `rover`. 

	The possible set of benchmark options are:

	* `doors`: `--d [1 2 4 6 8 10] --c [0 1 2 3]`, with *d* the number of doors to be open and ```c = {0 = [(0,0)], 1 = [(10,0)], 2 = [(0,10)], 3 = [(5,5)]}``` the number of extra configurations sampled within the initially reachable space, the unreachable space, or equally splitted between both;

	* `maze`: `--d [1 2 3 4 5 6 7 8 9 10] --c [0 1 2 3 4 5 6 7 8 9 10]`, with *d* the number of closed doors and *c* the number of locations to be visited;

	* `deliery`: `--d [1 2 4 6 8 10] --c [0 2 4 ... 48] --capacity [4 3 2 1]`, with *d* the number of closed doors, *capacity* the loading capacity of the robot, and *c* the configuration of the parcels. In detail, ```c = n = [nr, ng, dr, dg]```, where *nr* is the total number of red parcels, *ng* is the total number of green parcels, *dr* is the number of red parcels already at their delivery stations, and *dg* is the number of green parcels already at their delivery stations;

	* `rover`: `--d [2 4 6 8 10] --c [0 1 2 3 4]`, with *d* the number of samples (equally splitted between soils and rocks, each one separated from the robot by a closed door) and *c* the number of objectives to be photographed around the sample.

	The possible task planners are: `tampest`, `tamer`, `enhsp`, `fast-downward`.

	The possible motion planners are: `LazyRRT`, `RRT`.

	The possible topological refinements are: `none`, `unreach`, `obs`, `all`.
	
2) To run a PDDLStream test the command is:

	```
	python3 up-pddl-stream/test.py --domain <domain-name> --dim <dim> --d <n> --c <m> --tp <task-planner-name> --mp <motion-planner>
	```

	The possible domains are: `doors`, `maze`, `rover`.
	
	The possible dimensions are: `2D`, `3D`. `3D` setups are available only for `maze` and `rover`. 

	The possible set of benchmark options are:

	* `doors`: `--d [1 2 4 6 8 10] --c [0 1 2 3]`

	* `maze`: `--d [1 2 3 4 5 6 7 8 9 10] --c [0 1 2 3 4 5 6 7 8 9 10]`

	* `rover`: `--d [2 4 6 8 10] --c [0 1 2 3 4]`
	
	The possible task planners are: `incremental`, `binding`, `focused`, `adaptive`.

	The possible motion planners are: `LazyRRT`, `RRT`.

