from pathlib import Path
import sys; sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import my_ros2_wavefrontier

if __name__ == "__main__":
    my_ros2_wavefrontier.wavefrontier.main()
