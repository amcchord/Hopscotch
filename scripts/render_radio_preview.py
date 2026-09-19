#!/usr/bin/env python3
"""Render production Lua draw traces with EdgeTX's monochrome font atlases.

Requires Pillow and --font-dir pointing to the three PNGs under EdgeTX v2.11.0
radio/src/fonts/std/. These GPLv2 upstream assets are not vendored here.
This checks layout, not EdgeTX runtime, RF transport, or radio Lua memory limits.
"""
import argparse
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

def render(trace, fonts):
    image = Image.new("1", (128, 64))
    draw = ImageDraw.Draw(image)
    for row in trace.read_text().splitlines():
        op, *a = row.split("\t")
        x, y = int(a[0]), int(a[1])
        if op == "drawText":
            value, flags = a[2], int(a[3])
            atlas, width, height = fonts[flags // 256]
            inverse = flags % 2 == 1
            start = x
            for char in value:
                index = ord(char) - 32
                ox, oy = (index % 16) * width, (index // 16) * height
                columns = [[atlas.getpixel((ox+i, oy+j)) < 128 for j in range(height)] for i in range(width)]
                columns = [c for c in columns if not all(c)]  # font padding sentinel
                for col in columns + [[False]*height]:
                    for j, on in enumerate(col):
                        if 0 <= x < 128 and 0 <= y+j < 64:
                            image.putpixel((x, y+j), on != inverse)
                    x += 1
            assert x <= 128 and y+height <= 65, f"{trace.name}: clipped text {value!r} at {start},{y}, ends {x}"
        elif op == "drawLine":
            x2,y2=int(a[2]),int(a[3]);draw.line((x,y,x2,y2),fill=1)
        else:
            w,h=int(a[2]),int(a[3])
            assert x >= 0 and y >= 0 and x+w <= 128 and y+h <= 64, trace
            draw.rectangle((x,y,x+w-1,y+h-1),outline=1,fill=1 if op=="drawFilledRectangle" else None)
    return image

def main():
    parser=argparse.ArgumentParser();parser.add_argument("--font-dir",type=Path,required=True)
    args=parser.parse_args()
    fonts={}
    for kind,size,w,h in [(0,"05x07",5,8),(2,"04x06",5,7),(3,"08x10",8,12)]:
        fonts[kind]=(Image.open(args.font_dir/f"font_{size}.png").convert("L"),w,h)
    for trace in sorted(Path("output").glob("radio-*.tsv")):
        im=render(trace,fonts);im.resize((768,384),Image.Resampling.NEAREST).save(trace.with_suffix(".png"))
    selected=[("balance","ROBOT OVERVIEW"),("health","POSE + POWER"),("motors","SIX MOTORS"),
              ("history","LAST RUN + EVENTS"),("drive","DRIVE ON / ARMS OFF"),("lost","STALE DATA")]
    sheet=Image.new("RGB",(1080,790),"#101820");pen=ImageDraw.Draw(sheet)
    font=ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf",19)
    for i,(name,title) in enumerate(selected):
        x=24+(i%2)*536;y=20+(i//2)*258
        pen.text((x,y),title,font=font,fill="#bcebe4")
        im=Image.open(f"output/radio-{name}.png").resize((512,256),Image.Resampling.NEAREST)
        # Render panel 3x native so labels and frames have space around them.
        im=im.resize((384,192),Image.Resampling.NEAREST)
        sheet.paste(im,(x+56,y+33));pen.rectangle((x+54,y+31,x+441,y+226),outline="#446367",width=2)
    sheet.save("output/radio-preview.png")
    print("All Lua draw traces fit; saved output/radio-preview.png (host rendering, not a radio screenshot).")

if __name__ == "__main__": main()
