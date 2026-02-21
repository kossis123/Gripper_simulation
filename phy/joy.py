import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import pybullet as p
import pybullet_data
import math
class joy(Node):
    def __init__(self):
        super().__init__("node")
        self.sub=self.create_subscription(Joy,"/joy",self.callback,10)
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane=p.loadURDF("plane.urdf")
        self.robot=p.loadURDF('/home/badassatron/ros2_ws/src/phy/urdf/gripper.urdf',useFixedBase=True)
        self.robot2=p.loadURDF('/home/badassatron/ros2_ws/src/phy/urdf/box.urdf',basePosition=[0,0,0.15],useFixedBase=False)
        p.setGravity(0,0,-9.8)
        p.setTimeStep(1.0 / 240.0)
        p.setPhysicsEngineParameter(numSolverIterations=200)
        self.number=p.getNumJoints(self.robot)
        self.joints=[0.0]*self.number
        self.create_timer(1/240,self.time)
        self.scale=0.1
        self.movable_joints=[]
        self.uplimits=[0,0.384,0.9,-0.5,0,0.3822,0]
        self.lolimits=[0,-0.891,0.1,-1.57,0,-0.255,0]

        for i in range(self.number):
            joint_type=p.getJointInfo(self.robot,i)[2]
            if joint_type!=p.JOINT_FIXED:
                self.movable_joints.append(i)
        self.target=[0.0]*len(self.movable_joints)
    def callback(self,x:Joy):
         self.target[0]=self.target[0]+self.scale*x.axes[0]
         self.target[1]=self.target[1]+self.scale*x.axes[1]
         self.target[2]=self.target[2]+self.scale*x.buttons[0]-self.scale*x.buttons[2]
         self.target[3]=self.target[3]+self.scale*x.buttons[1]-self.scale*x.buttons[3]
         self.target[4]=self.target[4]+self.scale*x.buttons[4]-self.scale*x.buttons[5]
         self.target[5]=self.target[5]+self.scale*x.buttons[6]-self.scale*x.buttons[7]
         for i in range(len(self.target)):
            if i not in [0,4,6]:
                self.target[i]=max(min(self.uplimits[i],self.target[i]),self.lolimits[i])
         self.target[6]=-self.target[5]
         
    def time(self):
        for i,target in zip(self.movable_joints,self.target):
            if (i==6 ):
                p.setJointMotorControl2(
                self.robot,
                6,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target

            )
            elif(i==7):
                p.setJointMotorControl2(
                self.robot,
                7,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target

            )
        
            
            else:
                p.setJointMotorControl2(
                    self.robot,
                    i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target

                )
    

        p.stepSimulation()
         

        

def main(args=None):
    rclpy.init()
    node=joy()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()