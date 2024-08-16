import os
import random
import yaml

FILE_PATH = os.path.dirname(os.path.abspath(__file__))

def main():

    targets_file = os.path.join(FILE_PATH, "configs/tests/targets.yaml")
    doors_file = os.path.join(FILE_PATH, "configs/tests/doors.yaml")

    configs = {}
    robot_configs = [f'c{item}' for item in range(1, 10)]
    for c in range(1, len(robot_configs)+1):
        configs[c] = random.sample(robot_configs, c)

    doors = {}
    door_configs = [f'd{item}' for item in range(0, 10)]
    for d in range(1, len(door_configs)+1):
        doors[d] = random.sample(door_configs, d)

    with open(targets_file, 'w') as file:
        yaml.dump(configs, file) 

    with open(doors_file, 'w') as file:
        yaml.dump(doors, file) 

if __name__ == '__main__':
    main()