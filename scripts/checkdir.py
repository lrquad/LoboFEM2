import argparse
import os

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--filepath", help="")
    config = parser.parse_args()

    print(config.filepath)

