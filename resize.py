from PIL import Image
import glob
import time
import shutil
import os

#path = glob.glob('./'+posinput+'/*.'+'jpg')
path = "cat.jpg"
width_base = 320

def main():
    im = Image.open(path)
    width = im.size[0]
    height = im.size[1]
    ratio = height / width
    size = (width_base, int(width_base * ratio))
    im=im.resize(size)
    im.save(path)
    print("success")
    print(im.size[0], im.size[1])
    time.sleep(1)
            
main()
    
