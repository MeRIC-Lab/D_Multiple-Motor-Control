from motorLLC import *

DXL_MINIMUM_POSITION_VALUE  = 100               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

mc = motorLLC()
mc.open()
mc.torque_enable()

for i in range(1,6):

    mc.moveTo(dxl_goal_position[index])

    while 1:
        pos1, pos2 = mc.readPos()

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl_goal_position[index], pos1, DXL2_ID, dxl_goal_position[index], pos2))

        if ((abs(dxl_goal_position[index] - pos1) < DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl_goal_position[index] - pos2) < DXL_MOVING_STATUS_THRESHOLD)):
            break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

mc.close()
