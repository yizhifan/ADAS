uint8 NORM = 0
uint8 WARNING = 1
uint8 ERROR = 2
uint8 health_status
string name
uint64 start_timestamp_us
uint64 end_timestamp_us
float32 frame_rate
float32 standard_frame_rate
StatisticValue interval_ms
StatisticValue header_delay_ms
uint32 current_loss_frame_count
uint32 total_loss_frame_count
bool in_order
string reserved_info
string message
