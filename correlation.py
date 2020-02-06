from sys import argv
from numpy import array
from matplotlib import pyplot as plt
from tqdm import tqdm

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
    boid1 = posistions[:,1]
    boid2 = posistions[:,2]
    print(boid1)
    print(boid2)


    fig, axs = plt.subplots(3)
    steps = range(1,1001)

    for i in range(3):
        axs[i].plot(steps,boid1[:,i])
        axs[i].plot(steps,boid2[:,i])
    
    plt.show()
        