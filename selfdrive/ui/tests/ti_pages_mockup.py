"""Off-device mock-up of the on-car pages: draws what dump_ti_pages_layout.cc prints (the rectangles, texts, sizes and colours of
selfdrive/ui/qt/ti_pages_logic.h) at the device's 2160 x 1080, one PNG per page and state, plus an index.html that shows them.
It is a drawing of the layout, not the device's renderer: the font here is Segoe UI / DejaVu, the device uses Inter.
  usage: ti_pages_mockup.py scenes.json out_dir"""
import html
import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

FONTS = {False: ["segoeui.ttf", "DejaVuSans.ttf"], True: ["segoeuib.ttf", "DejaVuSans-Bold.ttf"]}


def font(px, bold):
  for name in FONTS[bold]:
    try:
      return ImageFont.truetype(name, px)
    except OSError:
      continue
  return ImageFont.load_default()


def draw_scene(scene, size):
  im = Image.new("RGB", size, (0, 0, 0))
  d = ImageDraw.Draw(im)
  for it in scene["items"]:
    x, y, w, h = it["x"], it["y"], it["w"], it["h"]
    if it["fill"] is not None:
      d.rounded_rectangle([x, y, x + w - 1, y + h - 1], radius=24, fill=tuple(it["fill"]))
    ink = tuple(it["ink"])
    if it["what"] == "band" or not it["text"]:
      continue
    bold = it["what"] in ("control", "badge") or it["sub"] == "bold"
    f = font(it["px"], bold)
    if it["what"] == "control" and it["sub"]:
      sub_h = it["sub_px"] + 30
      d.multiline_text((x + w / 2, y + (h - sub_h) / 2), it["text"], font=f, fill=ink, anchor="mm", align="center")
      d.text((x + w / 2, y + h - sub_h / 2 - 20), it["sub"], font=font(it["sub_px"], True), fill=ink, anchor="mm")
    elif it["align"] == "center":
      d.multiline_text((x + w / 2, y + h / 2), it["text"], font=f, fill=ink, anchor="mm", align="center")
    else:
      d.multiline_text((x, y + h / 2), it["text"], font=f, fill=ink, anchor="lm")
  if scene["locked"]:
    d.rectangle([5, 5, size[0] - 6, size[1] - 6], outline=(218, 202, 37), width=10)
  return im


def main():
  data = json.load(open(sys.argv[1], encoding="utf-8"))
  out = Path(sys.argv[2])
  out.mkdir(parents=True, exist_ok=True)
  size = tuple(data["screen"])
  rows = []
  for scene in data["scenes"]:
    draw_scene(scene, size).save(out / f"{scene['name']}.png")
    rows.append(f'<h2>{html.escape(scene["title"])}</h2><img src="{scene["name"]}.png" width="1080" height="540">')
  (out / "index.html").write_text(
    "<!doctype html><meta charset=utf-8><title>On-car pages mock-up</title>"
    "<body style='background:#222;color:#eee;font-family:sans-serif'>"
    "<h1>On-car pages: mock-up at 2160 x 1080 (shown at half size)</h1>"
    "<p>Drawn from selfdrive/ui/qt/ti_pages_logic.h by dump_ti_pages_layout.cc. Not the device renderer.</p>" + "".join(rows),
    encoding="utf-8")
  print(f"{len(rows)} pages -> {out}")


if __name__ == "__main__":
  main()
