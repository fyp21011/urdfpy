from urdfpy import URDF

def main():
    robot = URDF.load("tests/data/ur5/ur5.urdf")
    robot.animate()

if __name__ == '__main__':
    main()