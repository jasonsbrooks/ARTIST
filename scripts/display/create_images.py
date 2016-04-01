from PIL import Image,ImageDraw,ImageFont
import sys,csv,os

IMAGE_SIZE = (1024,600)

white = (255,255,255)

fnt = ImageFont.truetype('YaleDisRom', 400)

# filepath prefix
prefix = sys.argv[2]

with open(sys.argv[1]) as f:
    reader = csv.reader(f)
    next(reader)

    for row in reader:
        number = row[0]
        name = row[1]

        img = Image.new("RGB",IMAGE_SIZE)
        draw = ImageDraw.Draw(img)

        w,h = draw.textsize(name,font=fnt)

        draw.text(((IMAGE_SIZE[0] - w) / 2, (IMAGE_SIZE[1] - h) / 2), name, font=fnt, fill=white)
        img.save(os.path.join(prefix,str(number) + ".png"))

