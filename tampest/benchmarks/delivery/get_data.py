import os
import random
import yaml

FILE_PATH = os.path.dirname(os.path.abspath(__file__))

def main():

    parcels_file = os.path.join(FILE_PATH, "configs/tests/parcels.yaml")
    doors_file = os.path.join(FILE_PATH, "configs/tests/doors.yaml")
    all_file = os.path.join(FILE_PATH, "configs/tests/all.yaml")

    # (red, green) delivered al piu' uno in meno per colore
    parcels = [(1,0), (0,1), (1,1), 
               (2,0), (0,2), (2,2), 
               (3,0), (0,3), (3,3), 
               (4,0), (0,4), (4,4)]
    
    parcel_configs = [f't{item}' for item in range(1, 9)]
    unloading_red_configs = [f'u{item}' for item in range(5, 9)]
    unloading_green_configs = [f'u{item}' for item in range(9, 13)]

    parcels_dict = {}
    all = {}
    n = 0
    for (n_r, n_g) in parcels:

        n_red_delivered = list({0} | set(range(n_r)))
        n_green_delivered = list({0} | set(range(n_g)))

        delivered_red = []
        delivered_green = []
        for i in n_red_delivered:
            delivered_red = unloading_red_configs[:i] #random.sample(unloading_red_configs, i)
            for j in n_green_delivered:
                delivered_green = unloading_green_configs[:j] #random.sample(unloading_green_configs, j)

                reds = random.sample(parcel_configs, n_r-len(delivered_red)) + delivered_red
                greens = random.sample(list(set(parcel_configs) - (set(reds) -set(delivered_red))), n_g-len(delivered_green)) + delivered_green
        
                parcels_dict[str((n_r, n_g, i, j))] = {'red': reds, 'green': greens}
                all[n] = {"c0": n_r, "c1": n_g, "d0": i, "d1": j}
                n+=1

    doors = {}
    door_configs = [f'd{item}' for item in range(0, 10)]
    for d in range(1, len(door_configs)+1):
        doors[d] = random.sample(door_configs, d)

    with open(parcels_file, 'w') as file:
        yaml.dump(parcels_dict, file) 

    with open(doors_file, 'w') as file:
        yaml.dump(doors, file) 

    with open(all_file, 'w') as file:
        yaml.dump(all, file) 

if __name__ == '__main__':
    main()