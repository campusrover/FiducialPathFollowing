

class CreateNodes:

     def __init__(self):
        # Subscribing to image?
        # or already completed image. Will 
        # work out control flow

        # Publishing list of nodes
        self.node_Pub = rospy.Publisher('/node_List', array, queue_size=1)