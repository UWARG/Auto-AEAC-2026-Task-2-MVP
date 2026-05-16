import argparse
import time

from airside import CAMERA_MODE, Camera, TARGET_CENTER_POSITION_PX, _target_is_locked


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print the closest detected target relative to the image center",
    )
    parser.add_argument(
        "--mode",
        default=CAMERA_MODE,
        choices=("oakd", "arducam"),
        help="Camera backend to use",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="Seconds between status prints",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Capture one frame and print a single result",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    camera = Camera(mode=args.mode)

    try:
        while True:
            frame = camera.capture_frame()
            if frame is None:
                time.sleep(0.02)
                continue

            target = camera.get_closest_target(frame)
            if target is None:
                print("No target detected")
            else:
                center_x = (frame.shape[1] / 2) + TARGET_CENTER_POSITION_PX[0]
                center_y = (frame.shape[0] / 2) + TARGET_CENTER_POSITION_PX[1]
                x, y = target
                dx = x - center_x
                dy = y - center_y
                locked = _target_is_locked(frame, x, y)
                print(
                    "closest_target "
                    f"x={x:.1f} y={y:.1f} "
                    f"dx={dx:.1f} dy={dy:.1f} "
                    f"locked={'yes' if locked else 'no'}"
                )

            if args.once:
                break

            time.sleep(max(args.interval, 0.0))
    finally:
        camera.close()


if __name__ == "__main__":
    main()