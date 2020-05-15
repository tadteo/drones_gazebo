import os
import errno
import sys,getopt
import fileinput
import shutil
import re
import math

def main(argv):
    #loop for all test cases, all algorithms and the number of drones to collect the data daved from the simulation
    
    for i in [1,2,3]: #Test cases
        for j in ["ORCA","BAPF","EAPF"]: #algorithms
            out_file = open("/home/matteo/test_paper/results_test_"+str(i)+"_algo_"+str(j)+".txt", "w+")
            number_of_drones="N of UAVs: ["
            total_traj_to_write ="total traj: ["
            traj_to_write ="trajectory: ["
            max_time_to_write ="time: ["
            collisions_to_write = "collisions: ["
            for k in [5,10,15,20,25,30,40,50]: #number of drones
                delta_traj = 0
                max_time =-1
                total_traj = 0
                for n in range(1,k+1):
                    file = "/home/matteo/test_paper/world_"+str(k)+"_test_"+str(i)+"_algo_"+str(j)+"/log_drone_"+str(n)+".txt"
                    if os.path.isfile(file):
                        with open(file) as fp:
                            for l, line in enumerate(fp):
                                pass
                            if (l+1)!= 3 :
                                print("Error in "+file+" pls rerun the test")
                            else:
                                with open(file) as fp:
                                    for l, line in enumerate(fp):
                                        if l==0: #optimal trajectory
                                            delta_traj -= float(line)
                                        if l==1: #actual trajectory
                                            delta_traj += float(line)
                                            total_traj += float(line)
                                        if l==2: #time of execution
                                            if(float(line) >= max_time ):
                                                max_time=float(line)
                    else:
                        print(file+" does not exists\n")

                number_of_drones+= str(k)+", "
                total_traj_to_write += str(total_traj/k)+", " 
                traj_to_write += str(delta_traj/k)+", "
                max_time_to_write += str(max_time)+", "
                # Calculations of collisions 
                collisions = 0
                file = "/home/matteo/test_paper/collisions/collisions_"+str(k)+"_"+str(j)+"_"+str(i)+".txt"
                with open(file) as fp:
                    for l, line in enumerate(fp):
                        if(line[0]=='m'):
                            collisions += 1
                collisions_to_write += str(collisions/2)+", "
            number_of_drones+="\b\b]\n"
            total_traj_to_write += "\b\b]\n"
            traj_to_write +="\b\b]\n"
            max_time_to_write +="\b\b]\n"
            collisions_to_write +="\b\b]\n"
            out_file.write(number_of_drones)
            out_file.write(total_traj_to_write)
            out_file.write(traj_to_write)
            out_file.write(max_time_to_write)
            out_file.write(collisions_to_write)
            out_file.close()



if __name__ == "__main__":
    main(sys.argv[1:])