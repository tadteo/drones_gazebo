import os
import errno
import sys,getopt
import fileinput
import shutil
import re
import math

#CREATION OF THE DRONE MODELS
def main(argv):
    for i in [1,2]: #Test cases
        for j in ["ORCA","BAPF","EAPF"]: #algorithms
            out_file = open("/home/matteo/test_paper/results_test_"+str(i)+"_algo_"+str(j)+".txt", "w+")
            traj_to_write ="["
            max_time_to_write ="["
            for k in [5,10,15,20,30,40,50]: #number of drones
                delta_traj = 0
                max_time =-1
                for n in range(1,k):
                    file = "/home/matteo/test_paper/world_"+str(k)+"_test_"+str(i)+"_algo_"+str(j)+"/log_drone_"+str(n)+".txt"
                    with open(file) as fp:
                        for l, line in enumerate(fp):
                            pass
                        if (l+1)!= 3 :
                            return -1
                        else:
                            with open(file) as fp:
                                for l, line in enumerate(fp):
                                    if l==0: #optimal trajectory
                                        delta_traj -= float(line)
                                    if l==1: #actual trajectory
                                        delta_traj += float(line)
                                    if l==2: #time of execution
                                        if(float(line) >= max_time ):
                                            max_time=float(line)
                traj_to_write += str(delta_traj/k)+", "
                max_time_to_write += str(max_time)+", "
            traj_to_write +="\b\b]\n"
            max_time_to_write +="\b\b]\n"
            out_file.write(traj_to_write)
            out_file.write(max_time_to_write)
            out_file.close()


if __name__ == "__main__":
    main(sys.argv[1:])


# For each simulation
# Time of execution is the maximum time of execution 
# 

# Optimal trajectory dell' ultimo caso di test