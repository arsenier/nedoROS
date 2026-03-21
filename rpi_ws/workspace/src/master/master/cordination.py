from dataclasses import dataclass

@dataclass
class Pose:
    x: float
    y: float
    theta: float

cordination_ducks = [
    Pose(0.287213, 0.327222, 2.27), # 1
    Pose(0.336994, 0.363888, 0.61), # 2
    Pose(0.835612, 0.320223, 2.35), # 3
    Pose(0.890122, 0.443250, 0.61), # 4
    Pose(0.359710, 0.623648, 2.15), # 5
    Pose(0.354694, 0.621288, 1.03), # 6
    Pose(0.870093, 0.613370, 2.11), # 7
    Pose(0.891206, 0.633751, 0.97), # 8
    Pose(0.366035, 0.757009, 2.00), # 9
    Pose(0.338126, 0.750289, 1.15), # 10
    Pose(0.852274, 0.743161, 1.11), # 11
    Pose(0.891469, 0.749518, 2.03), # 12
    Pose(0.858826, 1.164620, 2.47), # 13
    Pose(0.853082, 1.129030, 0.67), # 14
    Pose(0.621408, 1.106540, 2.21), # 15
    Pose(0.344352, 1.145580, 2.23)  # 16
]

cordination_baze = []

baze1 = Pose(0.869143, 0.136321, 0.00)
baze2 = Pose(1.142820, 0.403740, -1.62)

for i in range(8):
    cordination_baze.append(Pose(baze1.x, 0.027 * (i + 1), baze1.theta))

for i in range(8):
    cordination_baze.append(Pose(1.25 - (0.027 * (i + 1)), baze2.y, baze2.theta))

def mirror_cordination(cordination: Pose, need: bool):
    if not need:
        return cordination
    else:
        return Pose(cordination.x, 1.75 - cordination.y, -cordination.theta)

for i in range(16):
    print(cordination_baze[i])

print(mirror_cordination(cordination_ducks[0], True))