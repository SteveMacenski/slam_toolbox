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


def get_from_cloud():
    db_cloud = pymongo.MongoClient("mongodb://rentao:1234@cluster0-shard-00-00-itbnx.gcp.mongodb.net:27017,cluster0-shard-00-01-itbnx.gcp.mongodb.net:27017,cluster0-shard-00-02-itbnx.gcp.mongodb.net:27017/test?ssl=true&replicaSet=Cluster0-shard-0&authSource=admin&retryWrites=true&w=majority").robot_2
    fs_cloud = gridfs.GridFS(db_cloud)

    five_datas = db_cloud['fs.files'].find().sort("uploadDate", -1)
    # print (five_datas.count())
    # for i in range(0,five_datas.count()):
    #     if (five_datas[i]):
    #         print("number", i ,"is" ,five_datas[i])


    file_data = five_datas[1]
    while (file_data["_id"] is None):
         time.sleep(1)
    
    id = file_data["_id"]
    print(id)
    data = fs_cloud.get(id).read()
    print("get data")
    robot=open("/home/rentao/.ros/cloud_map.posegraph",'wb')
    print ("open file")
    robot.write(data)
    robot.close()
    print("downlod from cloud finish")
 
    

def upload_to_robot(robot_name):
    db_robot = pymongo.MongoClient("mongodb://rentao:1234@cluster0-shard-00-00-itbnx.gcp.mongodb.net:27017,cluster0-shard-00-01-itbnx.gcp.mongodb.net:27017,cluster0-shard-00-02-itbnx.gcp.mongodb.net:27017/test?ssl=true&replicaSet=Cluster0-shard-0&authSource=admin&retryWrites=true&w=majority").robot_1
    fs_robot = gridfs.GridFS(db_robot)
    
    five_datas = db_robot['fs.files'].find().sort("uploadDate", -1)


    
    if five_datas.count() > 3:
        for i in range (3,five_datas.count()):
            five_datas = db_robot['fs.files'].find().sort("uploadDate", -1)
            file_data=five_datas[3]
            if (file_data):
                id = file_data["_id"]
                fs_robot.delete(id)    #     upload_to_robot("robot_1")


    cloud = open("/home/rentao/.ros/local.posegraph",'rb')
    fid = fs_robot.put(cloud, filename= robot_name)
    cloud.close()    
    #     a += 1
    #     print (a)



def deserialize(file_name,match_type,x,y,theta):
    rospy.wait_for_service('/slam_toolbox/deserialize_map')
    try:
        deserialize_call = rospy.ServiceProxy('/slam_toolbox/deserialize_map', DeserializePoseGraph)

        pose = Pose2D()
        pose.x=x
        pose.y=y
        pose.theta = theta

        deserialize_call(file_name,match_type,pose)

    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)


def serialize(file_name):
    rospy.wait_for_service('/slam_toolbox/serialize_map')
    try:
        serialize_call = rospy.ServiceProxy('/slam_toolbox/serialize_map', SerializePoseGraph)
        serialize_call(file_name)
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def addmap(submap_name):
    rospy.wait_for_service('/map_merging/add_submap')
    try:
        addmap_call = rospy.ServiceProxy('/map_merging/add_submap',AddSubmap)
        addmap_call(submap_name)
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)


def mergemap():
    rospy.wait_for_service('/map_merging/merge_submaps')
    try:
        mergemap_call = rospy.ServiceProxy('/map_merging/merge_submaps',MergeMaps)
        mergemap_call()
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

class myNode():
    def __init__(self):
        rospy.init_node("get_tf")
        self.listener = tf.TransformListener()
        rate = rospy.Rate(30.0)

    def give_tf(self):
        
        try:
            (trans,rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("no tf")
        
        euler = tf.transformations.euler_from_quaternion(rot)
        
        return (trans,euler)


if __name__ == "__main__":

    mynode = myNode()

    get_from_cloud()
    deserialize( "cloud_map" , 2, 0, 0, 0)
    print ("deserialize map")
    time.sleep(10)
    a=0
    print (a)
    while True :

        get_from_cloud()
        serialize("local")
        

        addmap("local")
        print ("add map")
        mergemap()
        time.sleep(1)
        print ("merge map")

        (trans,euler) = mynode.give_tf()   
        print ("stop for updatemap")
        time.sleep(1)
        deserialize("mergedmap" ,2, trans[0], trans[1], euler[2])   

        print ("deserialize map")

        upload_to_robot("robot_1")
        
        a += 1
        print (a)

        time.sleep(10)
