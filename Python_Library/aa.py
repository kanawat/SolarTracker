import sys
sys.path.append('G:\\Solar_PyLib')
import tracker
T=tracker.comm('COM9')
T.enable_comm(3)
T.move_actuator_west(3,0,500)
for i in range(20):
	T.get_actuator_len(3)
        T.enable_comm(3)
del T

