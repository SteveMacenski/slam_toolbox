import pymongo
import gridfs
import time
from slam_toolbox_msgs.srv import DeserializePoseGraph
from slam_toolbox_msgs.srv import SerializePoseGraph
from slam_toolbox_msgs.srv import MergeMaps
from slam_toolbox_msgs.srv import AddSubmap
import rospy
from pprint import pprint
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import TransformStamped
import tf
import geometry_msgs.msg



    

def upload_to_robot(robot_name):
    db_robot = pymongo.MongoClient("mongodb://rentao:1234@cluster0-shard-00-00-itbnx.gcp.mongodb.net:27017,cluster0-shard-00-01-itbnx.gcp.mongodb.net:27017,cluster0-shard-00-02-itbnx.gcp.mongodb.net:27017/test?ssl=true&replicaSet=Cluster0-shard-0&authSource=admin&retryWrites=true&w=majority").robot_2
    fs_robot = gridfs.GridFS(db_robot)
    
    five_datas = db_robot['fs.files'].find().sort("uploadDate", -1)


    
    if five_datas.count() > 3:
        for i in range (3,five_datas.count()):
            five_datas = db_robot['fs.files'].find().sort("uploadDate", -1)
            file_data=five_datas[3]
            if (file_data):
                id = file_data["_id"]
                fs_robot.delete(id)    #     upload_to_robot("robot_1")


    cloud = open("/home/rentao/.ros/good.posegraph",'rb')
    fid = fs_robot.put(cloud, filename= robot_name)
    cloud.close()    
    #     a += 1
    #     print (a)




if __name__ == "__main__":

    a=0
    print (a)
    while a<5 :



        upload_to_robot("robot_1")
        
        a += 1
        print (a)


