import os
import errno
import sys,getopt
import fileinput
import shutil
import re
import math

#CREATION OF THE DRONE MODELS
def main(argv):
    
    numCopies = 5
    CAalgorithm = 'libCollisionAvoidance.so'

    templateDir = './../models/dronetemplate'
    copyDir = './../models/'
    worldsDir = './../worlds/'
    templateModelName = '{$modelName}'
    templatePose = '{$modelPose}'
    templateIP = '{$modelIP}'
    templateAlgorithm = '{$modelAlgorithm}'
    templateWorldModels = '{$models}'
    templateWorldName = '{$worldName}'
    modelName= 'drone'
    worldName= 'world'
    worldFileName= 'world_template.world'
    numTest =1
    radius = 5
    #Read the parameters
    try:
        opts, argv = getopt.getopt(argv, "hn:a:t:",["number=","algorithm=","test="])
    except  getopt.GetoptError:
        print("generate.py -n <number_of_drones> -a <collision_avoidance_algorithm> \n")
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print("generate.py -n <number_of_drones> -a <collision_avoidance_algorithm>\n")
            print("Default number of drones: 8\n")
            print("Default Collision Avoidance algorithm: ORCA\n")
            sys.exit()
        elif opt in ('-n',"--number"):
            numCopies=int(arg)
        elif opt in ("-a","--algorithm"):
            if (arg == 'ORCA'):
                CAalgorithm= 'libCollisionAvoidance.so'
            elif (arg == 'BAPF'):
                CAalgorithm= 'libBAPF.so'
            elif (arg == 'EAPF'):
                CAalgorithm = "libEAPF.so"
            elif (arg == 'boid'):
                CAalgorithm = "libboid.so"
            else:
                print("Invalid Argument. Using default algorithm (ORCA)")
        elif opt in ("-t","--test"):
            numTest = int(arg)
            
            if numTest != 1 and numTest != 2 :
                numTest = 1
    print("Il numero di droni da creare e': "+str(numCopies)+'\n')
    print("La libreria di collision avoidance utilizzata e': "+CAalgorithm+"\n")
    print("Il test case e' il test "+str(numTest)+'\n')
    
    #Remove old files
    files = os.listdir(copyDir)
    for x in files:
        if (re.match("drone_.+",x)):
            shutil.rmtree(os.path.join(copyDir,x))
            print("Deleted "+x)

    files = os.listdir(worldsDir)
    for x in files:
        if (re.match("drones_.+.world",x)):
            os.remove(os.path.join(worldsDir,x))
            print("Deleted "+x)

    confFileName= 'model.config'
    sdfFileName = 'model.sdf'

    confFile = open(os.path.join(templateDir,confFileName),'r')
    sdfFile = open(os.path.join(templateDir,sdfFileName),'r')
    alpha= 0
    #Create new drones
    for i in range(1,numCopies+1):
        newModelName = modelName+"_"+str(i)
        newDir = os.path.join(copyDir,newModelName)
        os.makedirs(newDir)

        newConfFile = open(os.path.join(newDir,confFileName),'w')
        for line in confFile:
            newConfFile.write(line.replace(templateModelName,newModelName))
        newConfFile.close()
        confFile.seek(0,0)

        newSdfFile = open(os.path.join(newDir,sdfFileName),'w')
        for line in sdfFile:
            if templatePose in line:
                if(numTest == 1):
                   num = (numCopies-i)*2
                   newSdfFile.write(line.replace(templatePose, (str(num) +' 0 10')))
                elif numTest == 2:
                   alpha = (((360/(numCopies-1))*(i-1))* math.pi/180)+math.pi
                   newSdfFile.write(line.replace(templatePose, (str(radius*math.cos(alpha)) +' '+str(radius*math.sin(alpha))+' 1')))
            elif templateIP in line:
                newSdfFile.write(line.replace(templateIP, '127.0.0.'+str(i)))
            elif templateAlgorithm in line:
                newSdfFile.write(line.replace(templateAlgorithm, CAalgorithm ))
            else:
                newSdfFile.write(line.replace(templateModelName,newModelName))
        newSdfFile.close()
        sdfFile.seek(0,0)

    confFile.close()
    sdfFile.close()

    #CREATION OF THE WORLD FILE

    worldFile = open(os.path.join(worldsDir,worldFileName),'r')
    newWorldName = "drones_"+str(numCopies)+".world"
    newWorldFile = open(os.path.join(worldsDir,newWorldName),'w')
    models=""
    for i in range(1,numCopies+1):
        if numTest==1 :
            models+="\r\t\t<include>\n\t\t\t<name>drone_"+str(i)+"</name>\n\t\t\t<uri>model://drone_"+str(i)+"</uri>\n\t\t\t<pose>"+str((i-1)*2)+" 0 1 0 0 0</pose>\n\t\t</include>\n"
        else:
            alpha = i*((2*math.pi)/(numCopies))            
            models+="\r\t\t<include>\n\t\t\t<name>drone_"+str(i)+"</name>\n\t\t\t<uri>model://drone_"+str(i)+"</uri>\n\t\t\t<pose>"+str(radius*math.cos(alpha))+" "+str(radius*math.sin(alpha))+" 1 0 0 0</pose>\n\t\t</include>\n"
        
    
    for line in worldFile:
        if templateWorldModels in line:
            newWorldFile.write(line.replace(templateWorldModels,models))
        elif templateWorldName in line:
            newWorldFile.write(line.replace(templateWorldName, worldName+'_'+str(numCopies)))
        else:
            newWorldFile.write(line)
    newWorldFile.close()
    worldFile.seek(0,0)

if __name__ == "__main__":
    main(sys.argv[1:])
