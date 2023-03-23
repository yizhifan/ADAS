uint8 INITIALIZING = 0
uint8 READY = 1
uint8 ENGAGE_PENDING = 2
uint8 ENGAGED = 3
uint8 DISENGAGE_PENDING = 4
uint8 FAULT = 5
uint8 NAVIGATE = 0
uint8 CRUISE = 1
uint8 OFF_MAP = 2
uint8 AUTO_STATUS = 1
uint8 MAP_ENGINE_STATUS = 2
uint8 MAP_ROUTING_RESPONSE = 4
uint8 NOTIFY = 8
uint8 HDMAP_AVAILABLE = 16
std_msgs/Header header
uint8 available
uint8 auto_status
uint8 map_engine_status
RoutingResponse map_routing_response
ManagerNotify[] notify
bool hdmap_available
