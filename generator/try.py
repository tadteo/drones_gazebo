import sys, getopt

def main(argv):

    n = 8
    algorithm="ORCA"
    print("Sei un coglione\n")
    try:
        opts, argv = getopt.getopt(argv, "hn:a:",["number=","algorithm="])
    except  getopt.GetoptError:
        print("try.py -n <number_of_drones>")
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print("try.py -n <number_of_drones>")
            sys.exit()
        elif opt in ('-n',"--number"):
            n=arg
        elif opt in ('-a',"--algorithm"):
            algorithm=arg
    print("Il numero di droni da creare e': "+str(n) + " e l'algoritmo da utilizzare e' " + algorithm)

if __name__=="__main__":
    main(sys.argv[1:])