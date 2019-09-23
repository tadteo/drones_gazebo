import os
import errno
import sys
import fileinput
import shutil
import re

numCopies = 8

#CREATION OF THE DRONE MODELS
templateDir = './../models/dronetemplate'
copyDir = './../models/'
templateName = '{$modelName}'
templatePose = '{$modelPose}'
templateIP = '{$modelIP}'
modelName= 'drone'

confFileName= 'model.config'
sdfFileName = 'model.sdf'


confFile = open(os.path.join(templateDir,confFileName),'r')
sdfFile = open(os.path.join(templateDir,sdfFileName),'r')

files = os.listdir(copyDir)
for x in files:
    if (re.match("drone_.+",x)):
        shutil.rmtree(os.path.join(copyDir,x))
        print("Deleted "+x)

for i in range(1,numCopies+1):
    newModelName = modelName+"_"+str(i)
    newDir = os.path.join(copyDir,newModelName)
    os.makedirs(newDir)

    newConfFile = open(os.path.join(newDir,confFileName),'w')
    for line in confFile:
        newConfFile.write(line.replace(templateName,newModelName))
    newConfFile.close()
    confFile.seek(0,0)

    newSdfFile = open(os.path.join(newDir,sdfFileName),'w')
    for line in sdfFile:
        if templatePose in line:
            num = numCopies-i
            newSdfFile.write(line.replace(templatePose, (str(num) +' 0 10')))
        elif templateIP in line:
            newSdfFile.write(line.replace(templateIP, '127.0.0.'+str(i)))
        else:
            newSdfFile.write(line.replace(templateName,newModelName))
    newSdfFile.close()
    sdfFile.seek(0,0)

confFile.close()
sdfFile.close()

#CREATION OF THE WORLD FILE

worldsDir = './../worlds/'
template = '{$models}'

worldFileName= 'world_template.world'

files = os.listdir(worldsDir)
for x in files:
    if (re.match("world_.+_drones.world",x)):
        os.remove(os.path.join(worldsDir,x))
        print("Deleted "+x)

worldFile = open(os.path.join(worldsDir,worldFileName),'r')
newWorldName = "world_"+str(numCopies)+"_drones.world"
newWorldFile = open(os.path.join(worldsDir,newWorldName),'w')
models=""
for i in range(1,numCopies+1):
        models+="\r\t\t<include>\n\t\t\t<name>drone_"+str(i)+"</name>\n\t\t\t<uri>model://drone_"+str(i)+"</uri>\n\t\t\t<pose>"+str(i-1)+" 0 2 0 0 0</pose>\n\t\t</include>\n"
for line in worldFile:
    newWorldFile.write(line.replace(template,models))
newWorldFile.close()
worldFile.seek(0,0)