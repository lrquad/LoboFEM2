

import argparse
import os

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--videopath", help="")
    parser.add_argument("--gifpath", help="")
    parser.add_argument("--ss", help="")
    parser.add_argument("--t", help="")


    config = parser.parse_args()
    
    bashCommand = "ffmpeg -i " + config.videopath+" -vf fps=15,scale=640:-1:flags=lanczos,palettegen " + config.videopath+".png"
    os.system(bashCommand)
    bashCommand = "ffmpeg -t " + config.t +" -ss " +config.ss+ " -i " + config.videopath+ " -i " + config.videopath+".png" + " -filter_complex \"fps=15,scale=640:-1:flags=lanczos[x];[x][1:v]paletteuse\" " + config.gifpath
    os.system(bashCommand)

