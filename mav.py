import time
import keyboard
from pymavlink import mavutil


# 開始一個監聽UDP埠的連線
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# 等待第一個心跳訊息

# 這會設定鏈路的遠端系統及元件的ID
the_connection.wait_heartbeat()

# 向飛機發送解鎖指令
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

# 等待指令確認訊息
print("等待指令確認訊息")
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

print("從系統接收到的心跳訊息 (系統 %u 元件 %u)" %
      (the_connection.target_system, the_connection.target_component))

print("改變模式為 'GUIDED'")
the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,    # confirmation 
    1,    # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    4,    # GUIDED mode for Copter
    0, 0, 0, 0, 0  # unused parameters
)
# 接收COMMAND_ACK訊息並輸出
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)




while True:
    # 接收SERVO_OUTPUT_RAW訊息，並印出PWM輸出值
    #msg = the_connection.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
    #if msg:
        #print("PWM輸出：", msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw, msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw)

    # 接收GLOBAL_POSITION_INT訊息，並印出目前的高度
   # msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    #if msg:
        #print("目前的高度： %s" % msg.relative_alt)

    # 接收LOCAL_POSITION_NED訊息，並印出飛機的位置訊息
    #msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    #if msg:
        #print("飛機的位置訊息：", msg)

    # 接收RAW_IMU訊息，並印出IMU訊息
    #msg = the_connection.recv_match(type='RAW_IMU', blocking=True)
    #if msg:
        #print("IMU訊息：", msg)
    if keyboard.is_pressed("w"):
        print("向前")
        the_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            the_connection.target_system,  # target system
            the_connection.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            5, 0, 0,  # x, y, z positions
            0, 0, 0,  # x, y, z velocity in m/s (not used)
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0  # yaw, yaw_rate (not used)
        )        
    elif keyboard.is_pressed("s"):
        print("向後")    
        the_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            the_connection.target_system,  # target system
            the_connection.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            -5, 0, 0,  # x, y, z positions
            0, 0, 0,  # x, y, z velocity in m/s (not used)
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0  # yaw, yaw_rate (not used)
        )
    elif keyboard.is_pressed(" "):
        print("上升")
        the_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            the_connection.target_system,  # target system
            the_connection.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            0, 0, -1,  # x, y, z positions
            0, 0, 0,  # x, y, z velocity in m/s (not used)
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0  # yaw, yaw_rate (not used)
        )
    elif keyboard.is_pressed("c"):
        print("下降") 
        the_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            the_connection.target_system,  # target system
            the_connection.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            0, 0, 1,  # x, y, z positions
            0, 0, 0,  # x, y, z velocity in m/s (not used)
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0  # yaw, yaw_rate (not used)
        )        
    elif keyboard.is_pressed("a"):
        print("向左")       
        the_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            the_connection.target_system,  # target system
            the_connection.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            0, -5, 0,  # x, y, z positions
            0, 0, 0,  # x, y, z velocity in m/s (not used)
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0  # yaw, yaw_rate (not used)
        )        
    elif keyboard.is_pressed("d"):
        print("向右")
        the_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            the_connection.target_system,  # target system
            the_connection.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,  # frame
            0b0000111111111000,  # type_mask (僅啟用位置)
            0, 5, 0,  # x, y, z positions
            0, 0, 0,  # x, y, z velocity in m/s (not used)
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0  # yaw, yaw_rate (not used)
        )
    elif keyboard.is_pressed("t"):
         # 向飛機再次發送解鎖指令
        print("解鎖")
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

        time.sleep(1)
        # 向飛機發送起飛指令，目標高度為50
        print("起飛")
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 50)
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

        time.sleep(10)
        print("從系統接收到的心跳訊息 (系統 %u 元件 %u)" %
        (the_connection.target_system, the_connection.target_component)) 
    elif keyboard.is_pressed("l"):
        print("降落")
        # 設定為GUIDED模式
        the_connection.mav.command_long_send(
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            4,  # GUIDED mode
            0, 0, 0, 0, 0  # unused parameters
        )

        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 2)
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
    elif keyboard.is_pressed("l"):
        print("降落")
        # 設定為GUIDED模式
        the_connection.mav.command_long_send(
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            4,  # GUIDED mode
            0, 0, 0, 0, 0  # unused parameters
        )

        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
        the_connection.mav.command_long_send(
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            0,
            0, 0, 0, 0, 0, 0
        )
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    elif keyboard.is_pressed("X"):
        print("高度保持")
        # 設定為GUIDED模式
        the_connection.mav.command_long_send(
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            4,  # GUIDED mode
            0, 0, 0, 0, 0  # unused parameters
        )

        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                      mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)         
    elif keyboard.is_pressed("q"):
            break  # 按下 Q 键退出程序
    time.sleep(1)
