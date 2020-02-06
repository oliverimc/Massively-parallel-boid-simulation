from sys import argv
from numpy import array
from numpy import zeros
from matplotlib import pyplot as plt
from tqdm import tqdm
from pandas import DataFrame
from itertools import product
from scipy.stats import pearsonr

head_length =7
posistions =[]
file_val = True

if __name__ == "__main__":
    if(len(argv)>10):
        print(f"Usage {argv[0]} filename ")

    else:
        file = open("test.txt") if file_val else open(argv[1])
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
    posistion_frames =  [DataFrame(posistions[:,i],columns = list("xyz")) for i in range(len(posistions))]
 

    num_boids= len(posistion_frames[0]['x'])
    correlations = zeros((num_boids,num_boids))

    for ind1,boid1 in tqdm(enumerate(posistion_frames)):
        for ind2,boid2 in enumerate(posistion_frames):
            xr,xp = pearsonr(boid1['x'],boid2['x'])
            yr,yp = pearsonr(boid1['y'],boid2['y'])
            zr,zp = pearsonr(boid1['z'],boid2['z'])
            correlations[ind1,ind2] = sum([xr,yr,zr])/3

       

    print(correlations[:,1])
    plt.scatter(range(1,len(correlations)+1), correlations[:,1])
    plt.show()

   