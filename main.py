import argparse

from urdfpy import URDF

def main(path:str, animate:bool=True):
    robot = URDF.load(path)
    if animate:
        robot.animate()
    else:
        robot.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('path', type=str,
                        help='Path to URDF file that describes the robot')
    parser.add_argument('-a', action='store_true',
                        help='Visualize robot articulation')
    parser.add_argument('-c', action='store_true',
                        help='Use collision geometry')

    args = parser.parse_args()
    main(args.path, args.a)