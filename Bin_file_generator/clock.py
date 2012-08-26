import time
print('\r\n'*30)

epoch = 1262278800
# midnight of 1-jan-2010
while(1):
 current_time= (time.time()-epoch)
 print '\r\n'*30
 print '## SolTracker (TM) Clock Toolbox ##'
 print time.strftime("GMT:   %a, %d %b %Y %H:%M:%S ", time.gmtime())
 print time.strftime("LOCAL: %a, %d %b %Y %H:%M:%S ", time.localtime())
 print('#Day %d + %d sec'%(((current_time/86400)%1461+1), (current_time%86400)))
 print('Flash read command: >r%d\n'%((current_time/86400)%365+1+1500))
 time.sleep(1)
