import os
from PIL import Image

for file in os.listdir():
    filename, extension  = os.path.splitext(file)
    if extension == ".png":
        new_file = "{}.pgm".format(filename)
        with Image.open(file) as im:
            im.save(new_file)