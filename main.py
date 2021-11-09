import argparse

from urdfpy import URDF

def main(path:str, animate:bool=True, nogui:bool=False):
    robot = URDF.load(path)
    if not nogui:
        if animate:
            robot.animate()
        else:
            robot.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'path',
        type=str,
        help='Path to URDF file that describes the robot'
    )
    parser.add_argument(
        '-a',
        action='store_true',
        help='Visualize robot articulation'
    )
    parser.add_argument(
        '-c',
        action='store_true',
        help='Use collision geometry'
    )
    noGui = parser.add_argument(
        '--nogui',
        action='store_true',
        help='skip the GUI renderring, for machines with no display'
    )
    args = parser.parse_args()

    if args.nogui and args.a:
        raise argparse.ArgumentError(noGui, "no gui cannot be set when -a (animation) is given")
    main(args.path, args.a, args.nogui)