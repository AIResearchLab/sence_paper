import pybullet as p
import pybullet_data

def load_server():
    p.connect(p.GUI_SERVER)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setRealTimeSimulation(1)
    p.setGravity(0, 0, -9.81)
    plane = p.loadSDF("stadium.sdf")
    return

def command_actuator(robot,M_ID,target_position,target_velocity):
    p.setJointMotorControl2(
        bodyIndex=robot,
        jointIndex=M_ID,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_position,
        maxVelocity=target_velocity) 
    return