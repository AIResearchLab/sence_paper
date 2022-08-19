import numpy as np
from numpy import eye,zeros
from roboticstoolbox import DHRobot, RevoluteDH, ikine_LM
from spatialmath import base,SE3
import spatialmath as sm
from math import pi

#robot = ERobot.URDF(sence_paper/sence_main_description/urdf/sence_comb.xacro)
class Sence(DHRobot):
    def __init__(self, symbolic=False):

        if symbolic:
            import spatialmath.base.symbolic as sym

            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi

            zero = 0.0
            
        deg = pi / 180
        basetr = SE3(-0.04375,-0.00555,0) * SE3.Ry(90, 'deg')
        print(basetr)
        L = [
            RevoluteDH(d = -0.033926, alpha = pi/2), 
            RevoluteDH(d = 0.01322, a = -0.065, alpha = -pi/2, offset = pi/2, qlim = [-1.919862, 1.919862]), 
            RevoluteDH( a = -0.15, qlim = [-2.268928, 2.356194])
        ]

        super().__init__(
            L,
            name="Sence",
            manufacturer="UC-HRI",
            symbolic=symbolic,
            base=basetr,
            gravity=[0,0,9.81],
        )

if __name__ == "__main__":  # pragma nocover

    sence = Sence(symbolic=False)
    print(sence)
    limit_of_j2 = 1.919862 #Affects the inverse kinematics calculation
    limit_of_j3 = 2.356194 #Affects the inverse kinematics calculation
    #use this component to convert a desired xyz coordinate of the foot
    example_pose1 = SE3(-0.1477,-0.1487,-0.1212);
    example_pose2 = SE3(-0.1477,-0.1222,-0.1212);

    #I think there is only one possible solution for this leg, I need to
    #manually draw it out to check
    pose1 = sence.ikine_LM(example_pose1,[0,0,0],'mask',[1,1,1,0,0,0],'tol',0.001,'ilimit',1500,'rlimit',1500)
    pose2 = sence.ikine_LM(example_pose2,[0,0,0],'mask',[1,1,1,0,0,0],'tol',0.001,'ilimit',1500,'rlimit',1500)
    
    q = [0,0,0] #q0 q1 q2, joint positions
    tjacob = sence.jacob0(q,'trans')
    Sence.plot(q)

    mass_0 = 0.09734685103972182
    mass_1 = 0.1208747488020619
    mass_2 = 0.052701725493830516

    sence.links(1).m=mass_0
    sence.links(2).m=mass_1
    sence.links(3).m=mass_2

    pybullet_COM_0 = SE3(-0.04078394902352935,-0.013231283994066126,0.00021684450055575039)
    pybullet_COM_1 = SE3(-0.06366942125676311,-0.021943300476205383,0.0024103683252244337)
    pybullet_COM_2 = SE3(-0.06827181090766318,0.01066089288586344,0.020893759814543007)

    sence.links(1).I=zeros(3,3)
    sence.links(2).I=zeros(3,3)
    sence.links(3).I=zeros(3,3)

    #Calculate the new COM position per PC's definition from URDF data
    basetr = SE3(-0.04375,-0.00555,0) * SE3.Ry(90, 'deg')
    row0 = basetr*SE3(0,0,-0.033926)*SE3.Rx(pi/2)
    row1 = row0*SE3.Rz(pi/2)*SE3(-0.065,0,0.01322)*SE3.Rx(-pi/2)
    row2 = row1*SE3(-0.15,0,0)

    pc_com_0_est = SE3(SE3.Ry(-pi/2)*SE3.Rz(pi/2)*SE3(0.033926,-0.00,0)*pybullet_COM_0)
    sence.links(1).r = pc_com_0_est
    #-0.0002   -0.0069    0.0132

    pc_com_1_est = SE3(SE3(0.065,0.02,0)*pybullet_COM_1)
    sence.links(2).r = pc_com_1_est
    #This is an approximation but broadly speaking correct, domain
    #randomization could assist in RLs 
    # 0.0013   -0.0019    0.0024

    #another approximation but should be fine
    pc_com_2_est = SE3(SE3(0.15,0.010,-0.02)*pybullet_COM_2)
    sence.links(3).r = pc_com_2_est
    #0.0817    0.0207    0.0009