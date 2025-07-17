import argparse
import os,stat
import glob
import shutil

parser = argparse.ArgumentParser(description='parse input parameters')

parser.add_argument('--data_path', type=str, default='USE_TIME_TEST')


def process(data_path):
    #get pose file
    preprocessDir = data_path+"/preprocess/"
    reconstructionDir = data_path+"/reconstruction/"
    content = os.listdir(preprocessDir)
    dir = []
    for obj in content:
        if os.path.isdir(preprocessDir+obj):
            dir.append(obj) 
    if len(dir)==0:
        print("no directory exists at ", preprocessDir)      
        return
    files = glob.glob(preprocessDir+dir[0]+"/*.perception")
    if len(files)==0:
        print("no perception file exists at ", dir[0])
        return    
    perceptionFile = files[0]
    poseFile = reconstructionDir+"data_output/"+dir[0]+".gps"
    if not os.path.exists(poseFile):
        print("at roadMapping, could not find poseDir")
        return

    outputDir = data_path+"/roadMappingResult"
    print("poseFile: ", poseFile)
    print("perceptionFile: ", perceptionFile)
    print("outputDir: ", outputDir)
    
    if os.path.exists(outputDir):
        os.system("rm -rf "+outputDir)
    os.system("mkdir "+outputDir+" && chmod 777 "+outputDir)
    laneBoundaryDir = outputDir + "/laneBoundary"
    os.system("mkdir "+laneBoundaryDir+" && chmod 777 "+laneBoundaryDir)
    roadBoundaryDir = outputDir + "/roadBoundary"
    os.system("mkdir "+roadBoundaryDir+" && chmod 777 "+roadBoundaryDir)
    trafficArrowDir = outputDir + "/trafficArrow"
    os.system("mkdir "+trafficArrowDir+" && chmod 777 "+trafficArrowDir)
    roadMarkDir = outputDir + "/roadMark"
    os.system("mkdir "+roadMarkDir+" && chmod 777 "+roadMarkDir)

    command = os.getcwd()+"/mapbuild/roadMapping/build/roadModel "+poseFile+" "+perceptionFile+" "+reconstructionDir+" "+outputDir
    
    os.system(command)
    print("command: ", command)

    os.system("cp "+laneBoundaryDir+"/boundary.json "+outputDir+"/laneBoundary.json")
    os.system("cp "+roadBoundaryDir+"/boundary.json "+outputDir+"/roadBoundary.json")
    os.system("cp "+trafficArrowDir+"/trafficArrow.json "+outputDir+"/trafficArrow.json")
    os.system("cp "+roadMarkDir+"/roadMark.json "+outputDir+"/roadMark.json")

    #command = os.getcwd()+"/mapbuild/roadMapping/build/uploadLane "+outputDir+"/laneBoundary.json"
    #os.system(command)
    #print("command: ", command)


if __name__ == '__main__':
    args = parser.parse_args()
    data_path = str(args.data_path)
    process(data_path)
    

    
