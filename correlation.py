from sys import argv
from numpy import array
from numpy import zeros
from matplotlib import pyplot as plt
from tqdm import tqdm
from pandas import DataFrame
from itertools import product
from scipy.stats import pearsonr
from numpy import all
from numpy import sqrt
from statistics import mean

head_length =7
posistions =[]
file_val = True


def distance(vec1,vec2):
    x1,y1,z1 = vec1
    x2,y2,z2 = vec2
    return sqrt(min(x1-x2,1000-abs(x1-x2))**2+min(y1-y2,1000-abs(y1-y2))**2+min(z1-z2,1000-abs(z1-z2))**2)


if __name__ == "__main__":
    if(len(argv)>10):
        print(f"Usage {argv[0]} filename ")

    else:
        
        file = open("blank.txt") if file_val else open(argv[1])
        [next(file) for i in range(head_length)]
        for line in tqdm(file):
            pos_vector_string = line.split('$')[:-1]
            boid_posistions = []
           
            for pos_string in pos_vector_string:
                value_strings = pos_string.split(":")
               
                try:
                    values = [float(value) for value in value_strings]
                except:
                    print(value_strings)
                    exit()

                boid_posistions.append(array(values))
            posistions.append(boid_posistions)
    
    posistions = array(posistions)
    """ posistion_frames =  [DataFrame(posistions[:,i],columns = list("xyz")) for i in range(len(posistions))]
    num_boids= len(posistion_frames[0]['x'])
    correlations = zeros((num_boids,num_boids))

    for ind1,boid1 in tqdm(enumerate(posistion_frames)):
        for ind2,boid2 in enumerate(posistion_frames):
            pos1,pos2 in zip(zip(boid1['x'],boid1['y'],boid1['z']),zip(boid2['x'],boid1['x'],boid1['x'])) """

 

    boid = 0 
    distances = []
    for ind,boid_step in tqdm(enumerate(posistions)):
        chosen_boid = boid_step[boid]
        dist_arr = array([distance(chosen_boid,boid) for boid in boid_step if not all(boid==chosen_boid)])
        distances.append(dist_arr)
    distances = array(distances)
    paths = [distances[:,i] for i in range(len(distances[0]))]
    sorted_paths = sorted(paths, key = lambda x: mean(x)) 

    for path in sorted_paths[:4]:
        plt.plot(range(1,len(path)+1),path, color = 'b')
    
    for path in sorted_paths[-4:]:
        plt.plot(range(1,len(path)+1),path,color = 'r')
    
    plt.show()


       

   
   