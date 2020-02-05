from sys import argv
import glob

head_length =7

if(len(argv)==3):
    base_file_name = argv[1]
    filenames=list(glob.glob(base_file_name+'*'))
    print(f"Found files: {filenames}. Joining")
    files = [open(name) for name in filenames]
    output = open(argv[2]+".txt", 'w+')
    head_data =[]
    
    for ind,_file in enumerate(files):
        for i in range(head_length):
            if ind ==0:
                head_data.append(next(_file))
            else:
                next(_file)

    


            
    output = open(argv[2]+".txt", 'w+')
    output.write("".join(head_data))
    for streams in zip(*files):
        stripped_streams =[stream[:-1] for stream in streams]
        output.write("".join(stripped_streams))
        output.write('\n')
    output.close()
    print(f"Output to {argv[2]+'.txt'}")
        

else:
    print(f"Usage {argv[0]} FILEBASE OUTPUT_NAME")
