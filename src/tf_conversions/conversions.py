from geometry_msgs.msg import Pose

# Converts a geometry_msgs/Pose into a list [x,y,z,q1,q2,q3,q4]
def pose_msg_to_pose_vector(pose):
    assert isinstance(pose, Pose), 'Input is not of type geometry_msgs/Pose'
    return [pose.position.x, pose.position.y, pose.position.z, \
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
           ]
