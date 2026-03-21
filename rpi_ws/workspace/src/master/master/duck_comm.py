
from enum import Enum

class DuckType(Enum):
    CUBE = 0
    ARUCO_CUBE = 1
    CYLINDER = 2
    TUX = 3
    ZAYC = 4
    OCTO = 6
    
def enctypt_duck(duck_type: DuckType, duck_id, array):
        array.append(duck_type.value)
        array.append(duck_id)

int16_array_pub = self.create_publisher(Int16MultiArray, '/duck_type', 10)
list = [...]

msg.data = list
int16_array_pub.publish(msg)




self.sub = self.create_subscription(
Int16MultiArray,
'/duck_type',
self.listener_callback,
10
)
def listener_callback(self, msg):
    data = list(msg.data)
    queue.append(DuckType(data[0]), data[1])
