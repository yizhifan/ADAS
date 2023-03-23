uint8 STOP = 0
uint8 SAFTY_RUNNING = 1
uint8 NORMAL_RUNNING = 2
uint8 TAKEOVER = 3
uint8 NONE = 0
uint8 SAFTY_RUNNING_WAIT_TAKEOVER = 1
std_msgs/Header header
uint64 timestamp_us
uint8 status
uint8 detail_status
string message
