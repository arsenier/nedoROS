
from enum import Enum

class DuckType(Enum):
    OCTO = 1
    KROLIK = 2
    TUX = 3
    CYLYNDER = 4
    BLUE_CUBE = 5
    RED_CUBE = 6
    ARUCO_21_CUBE = 7
    ARUCO_20_CUBE = 8
    
sample_list = [4, 1, 3, 6, 5, 7, 8, 2]
sample_list = [6, 4, 5, 100000, 7, 1, 3, 2]

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
