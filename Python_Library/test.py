#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      Kanawat
#
# Created:     21/12/2010
# Copyright:   (c) Kanawat 2010
# Licence:     <your licence>
#-------------------------------------------------------------------------------
#!/usr/bin/env python
import time,tracker,sys
import config

def retry_open_port():
    for i in range(config.OPEN_PORT_RETRY):
        try:
            T = tracker.comm(config.COMM_PORT)
        except:
            T = -1
        else:
            return T
        print("comm port open failed, retry#%d"%i)
        time.sleep(config.OPEN_PORT_RETRY_INTERVAL)
    return -1

def check_ready(units,T):
    non_ready_unit=[]
    for i in units:
        if (T.enable_comm(i)==0):
            non_ready_unit.append(i)
        T.disable_comm()
    return non_ready_unit


def main():
    import sys
    log=open(config.LOG,'a')
    args = sys.argv[1]
    sys.path.append(config.PATH)
    T = retry_open_port();
    T.debug=0
    if (T == -1):
        log.write('\r\n%s: opening port failed'%time.ctime());
        log.flush();
        log.close();
        ##return
    else:
        log.write('\r\n%s: opening port success'%time.ctime());
    if args == '-ut': # update time
        T.update_time()
        tracker_id = config.TRACKER_ID
        for i in range(config.COMMUNICATION_RETRY):
            retry_list = []
            if (len(tracker_id)>0):
                for unit in tracker_id:
                    T.enable_comm(unit);
                    my_time=T.get_datetime(unit);
                    T.disable_comm(unit);
                    if (my_time==-1):## fail communication
                        log.write('\r\n%s: fail communicating with unit id:%d, retry#%d'%(time.ctime(),unit,i))
                        retry_list.append(unit)
                    else:
                        log.write('\r\n%s: unit %d time is: %s'%(time.ctime(),unit,time.asctime(time.localtime(my_time))))
                tracker_id = retry_list;
                print retry_list
            else: ## all unit have success
                break
            time.sleep(config.COMMUNICATION_RETRY_INTERVAL)
    elif args == '-home': # go home
        for act in range(4):
            non_ready_unit = config.TRACKER_ID
            for i in range(config.COMMUNICATION_RETRY):
                non_ready_unit=check_ready(non_ready_unit,T)
                if (non_ready_unit ==[]) :
                    break
                time.sleep(config.COMMUNICATION_RETRY_INTERVAL)
            if (non_ready_unit != []):
                log.write('\r\n%s: Units %s not ready'%(time.ctime(),non_ready_unit))
            T.go_home(0xfffe,act) ## all control unit move
            log.write('\r\n%s: Broadcast gohome message on act#%d'%(time.ctime(),act))
    elif args == '-stat': ## check stat
        for i in config.TRACKER_ID:
            T.enable_comm(i)
            print('%s'%T.in_packet)
            log.write('\r\n%s: Checking unit stat:%s'%(time.ctime(),T.in_packet))
            T.disable_comm()
    del T
    log.flush()
    log.close()

if __name__ == '__main__':
    main()