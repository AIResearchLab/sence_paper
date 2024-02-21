from math import pi

poseSec = 0
poseNano = 500_000_000

jointNames = ['fl_joint1', 'fl_joint2', 'fl_joint3',
              'fr_joint1', 'fr_joint2', 'fr_joint3',
              'bl_joint1', 'bl_joint2', 'bl_joint3',
              'br_joint1', 'br_joint2', 'br_joint3']

flatPose = [0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0]

curlPose = [pi/2, pi/2, 2.0,
            -pi/2, -pi/2, 2.0,
            -pi/2, -pi/2, 2.0,
            pi/2, pi/2, pi/2]

crabStandPose = [-pi/2, pi/8, pi/6,
                 pi/2, -pi/8, pi/6,
                 pi/2, -pi/8, pi/6,
                 -pi/2, pi/8, pi/6]

dogReadyPose = [pi/2, 0.0, 0.0,
               -pi/2, 0.0, 0.0,
               -pi/2, 0.0, 0.0,
               pi/2, 0.0, 0.0]

dogFlatPose = [1.4, -pi/2, 0.2,
               -1.4, pi/2, 0.2,
               -1.8, -1.3, 0.2,
               1.8, 1.3, 0.2]

dogStandPose = [-0.9, -pi/2, 1.9, #square
               0.9, pi/2, 1.9,
               -4.0, -pi/2, 1.9,
               4.0, pi/2, 1.9]

dogWalkPoses = [
    #fl down fr up
    [0.0, -pi/2, 1.25,
    0.9, pi/2, 2.3,
    -4.0, -pi/2, 1.9,
    pi, pi/2, 1.25],

    #fr down bl up
    [-0.9, -pi/2, 1.9,
    0.45, pi/2, 1.8,
    -4.0, -pi/2, 2.3,
    3.8, pi/2, 1.7],

    #bl down br up
    [-0.9, -pi/2, 1.9,
    0.45, pi/2, 1.8,
    -pi, -pi/2, 1.25,
    4.0, pi/2, 2.3],

    #br down fl up
    [-0.9, -pi/2, 2.3,
    0.9, pi/2, 1.9,
    -4.0, -pi/2, 1.9,
    pi, pi/2, 1.25]

]

cw_angle1 = pi/8
cw_angle2 =  0.0
cw_angle3 = -pi/8
cw_angle4 = -pi/16
cw_angle5 = pi/16

UP = 0.0
DOWN = pi/3

poses = {
    "flat": [0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0],
    "crab_flat": [-pi/2, pi/8, 0.0,
                 pi/2, -pi/8, 0.0,
                 pi/2, -pi/8, 0.0,
                 -pi/2, pi/8, 0.0],
    "crab_stand_short": [-pi/2, pi/8, pi/8,
                 pi/2, -pi/8, pi/8,
                 pi/2, -pi/8, pi/8,
                 -pi/2, pi/8, pi/8],
    "crab_stand_tall": [-pi/2, pi/8, pi/4,
                 pi/2, -pi/8, pi/4,
                 pi/2, -pi/8, pi/4,
                 -pi/2, pi/8, pi/4],


    "cw1": [-pi/2, cw_angle5, DOWN,
                 pi/2, cw_angle1, DOWN,
                 pi/2, cw_angle4, DOWN,
                 -pi/2, cw_angle2, UP],
    "cw2": [-pi/2, cw_angle5, DOWN,
                 pi/2, cw_angle1, DOWN,
                 pi/2, cw_angle4, DOWN,
                 -pi/2, cw_angle3, DOWN],

    "cw3": [-pi/2, cw_angle4, DOWN,
                 pi/2, cw_angle2, UP,
                 pi/2, cw_angle3, DOWN,
                 -pi/2, cw_angle4, DOWN],
    "cw4": [-pi/2, cw_angle4, DOWN,
                 pi/2, cw_angle3, DOWN,
                 pi/2, cw_angle3, DOWN,
                 -pi/2, cw_angle4, DOWN],

    "cw5": [-pi/2, cw_angle3, DOWN,
                 pi/2, cw_angle4, DOWN,
                 pi/2, cw_angle2, UP,
                 -pi/2, cw_angle5, DOWN],
    "cw6": [-pi/2, cw_angle3, DOWN,
                 pi/2, cw_angle4, DOWN,
                 pi/2, cw_angle1, DOWN,
                 -pi/2, cw_angle5, DOWN],

    "cw7": [-pi/2, cw_angle2, UP,
                 pi/2, cw_angle5, DOWN,
                 pi/2, cw_angle5, DOWN,
                 -pi/2, cw_angle1, DOWN],
    "cw8": [-pi/2, cw_angle1, DOWN,
                 pi/2, cw_angle5, DOWN,
                 pi/2, cw_angle5, DOWN,
                 -pi/2, cw_angle1, DOWN],
}

sequences = {
    "flat": ["flat"],
    "crab_stand_up": ["crab_flat",
                      "crab_stand_short", 
                      "crab_stand_tall"],
    "crab_stand_to_flat": ["crab_flat",
                           "flat"],
    "crab_dance": ["crab_stand_short", 
                   "crab_stand_tall"],
    "crab_walk": ["cw1",
                  "cw2",
                  "cw3",
                  "cw4",
                  "cw5",
                  "cw6",
                  "cw7",
                  "cw8"]
}