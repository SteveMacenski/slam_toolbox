import pymongo
import gridfs
import time
from slam_toolbox_msgs.srv import MergeMaps
from slam_toolbox_msgs.srv import AddSubmap
import rospy

def get_from_robot(robot_name):

    db_robot = pymongo.MongoClient("mongodb://rentao:1234@cluster0-shard-00-00-itbnx.mongodb.net:27017,cluster0-shard-00-02-itbnx.mongodb.net:27017/test?ssl=true&replicaSet=Cluster0-shard-0&authSource=admin&retryWrites=true&w=majority").robot
    fs_robot = gridfs.GridFS(db_robot)

    file_data = db_robot['fs.files'].find_one({"filename": robot_name},no_cursor_timeout=True)
    if (file_data):
        id = file_data["_id"]
        data = fs_robot.get(id).read()
        print("get from robot")
        robot=open("/home/rentao/cloud_server/" + robot_name + ".posegraph",'wb')
        robot.write(data)
        robot.close()
        return True
    else:
        return False


def upload_to_cloud():
    db_cloud = pymongo.MongoClient("mongodb://rentao:1234@cluster0-shard-00-00-itbnx.mongodb.net:27017,cluster0-shard-00-02-itbnx.mongodb.net:27017/test?ssl=true&replicaSet=Cluster0-shard-0&authSource=admin&retryWrites=true&w=majority").cloud
    fs_cloud = gridfs.GridFS(db_cloud)
    file_data = db_cloud['fs.files'].find_one({"filename": "mergedmap"},no_cursor_timeout=True)

    cloud = open("/home/rentao/.ros/mergedmap.posegraph",'rb')
    fid = fs_cloud.put(cloud, filename= "mergedmap")
    cloud.close()
    # if (file_data):
    #     id = file_data["_id"]
    #     print(id)
    #     fs_cloud.delete(id)


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

if __name__ == "__main__":
    while True :

        upload_to_cloud()

        # if_robot1 = get_from_robot("robot1")
        # # get_from_robot("robot2")
        # if (if_robot1):
        #     addmap("/home/rentao/cloud_server/robot1")
        #     print("add robot1")
        # addmap("mergedmap")

        # # addmap("/home/rentao/cloud_server/robot2")
        # mergemap()

        # time.sleep(10)
