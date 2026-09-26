# -*- coding: utf-8 -*-
"""
在电脑上预览 0.96 寸 OLED 的 4 个页面（128x64，1:1 像素模拟）。

它直接读取 my_app/cn_font_data.h 里的中文点阵和 my_lib/oled_default_font.h
里的 8x8 英文点阵，按 main.c 里同样的坐标画一遍，所以不出意外就是接上屏幕
后看到的样子，还能顺便检查有没有"缺字"。

用法: python preview_ui.py [输出png路径]
"""

import math
import os
import re
import sys

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CN_H = os.path.join(HERE, "cn_font_data.h")
ASCII_H = os.path.join(ROOT, "my_lib", "oled_default_font.h")

W, H = 128, 64


# ---------------------------------------------------------------- 解析字库

def parse_cn_font(path):
    text = open(path, encoding="utf-8").read()
    data = {}
    for m in re.finditer(r"static const uint8_t (cn_[0-9A-F]{4})\[32\] = \{(.*?)\};", text, re.S):
        code = int(m.group(1)[3:], 16)
        body = re.sub(r"/\*.*?\*/", "", m.group(2), flags=re.S)
        data[code] = bytes(int(v, 16) for v in re.findall(r"0x([0-9A-Fa-f]{2})", body))
    return data


def parse_ascii_font(path):
    text = open(path, encoding="utf-8").read()
    bmps = {}
    for m in re.finditer(r"static const uint8_t (default_font_GlyphBitmap_[0-9A-F]{4})\[\] = \{(.*?)\};", text, re.S):
        bmps[m.group(1)] = bytes(int(v, 16) for v in re.findall(r"0x([0-9A-Fa-f]{2})", m.group(2)))

    glyphs = {}
    for name, body in re.findall(r"\{\s*\.Name = \"([0-9A-F]{4})\",(.*?)\}", text, re.S):
        def g(key, default=0):
            m = re.search(r"\.%s = (-?\d+)" % key, body)
            return int(m.group(1)) if m else default

        m = re.search(r"\.Bitmap = (\w+)", body)
        glyphs[int(name, 16)] = {
            "dwx0": g("Dwx0", 7),
            "bbw": g("BBw"),
            "bbh": g("BBh"),
            "bbx": g("BBxoff0x"),
            "bby": g("BByoff0y"),
            "bmp": bmps.get(m.group(1), b"") if m else b"",
        }
    return glyphs


# ---------------------------------------------------------------- 屏幕模拟

class Screen(object):
    def __init__(self):
        self.fb = [[0] * H for _ in range(W)]      # fb[x][y]，1 = 亮

    def rect_fill(self, x, y, w, h, color):
        for xx in range(max(0, x), min(W, x + w)):
            for yy in range(max(0, y), min(H, y + h)):
                self.fb[xx][yy] = color

    def rect_frame(self, x, y, w, h):
        for yy in range(max(0, y), min(H, y + h)):
            for xx in (x, x + w - 1):
                if 0 <= xx < W:
                    self.fb[xx][yy] = 1
        for xx in range(max(0, x), min(W, x + w)):
            for yy in (y, y + h - 1):
                if 0 <= yy < H:
                    self.fb[xx][yy] = 1

    def bitmap(self, x, y, w, h, data, pen=1, brush=None):
        nb = (w + 7) // 8
        for xx in range(w):
            for yy in range(h):
                px, py = x + xx, y + yy
                if not (0 <= px < W and 0 <= py < H):
                    continue
                if data[xx // 8 + yy * nb] & (0x80 >> (xx % 8)):
                    self.fb[px][py] = pen
                elif brush is not None:
                    self.fb[px][py] = brush

    def ascii_text(self, x, ybase, s, font):
        cx = x
        for ch in s:
            gid = font.get(ord(ch))
            if gid is None:
                cx += 7
                continue
            self.bitmap(cx + gid["bbx"], ybase - gid["bby"] - gid["bbh"],
                        gid["bbw"], gid["bbh"], gid["bmp"])
            cx += gid["dwx0"]

    def cn_text(self, x, ytop, s, cn, invert=False):
        for ch in s:
            data = cn.get(ord(ch))
            if data is not None:
                self.bitmap(x, ytop, 16, 16, data,
                            pen=0 if invert else 1,
                            brush=1 if invert else None)
            x += 16
        return x

    def cn_banner(self, ytop, s, cn):
        self.rect_fill(0, ytop, W, 16, 1)
        self.cn_text(34, ytop, s, cn, invert=True)

    def bar(self, x, y, w, h, value, maxv):
        self.rect_frame(x, y, w, h)
        fill = int(value * (w - 2) / maxv)
        self.rect_fill(x + 1, y + 1, max(0, min(fill, w - 2)), h - 2, 1)

    def curve(self, points, top=20, bottom=53, x0=0):
        prev = bottom
        for i, v in enumerate(points):
            xx = x0 + i
            if xx >= W:
                break
            y = bottom - int(v * (bottom - top) / 4095)
            for yy in range(min(prev, y), max(prev, y) + 1):
                self.fb[xx][yy] = 1
            prev = y

    def image(self, scale=4):
        img = Image.new("L", (W, H), 0)
        px = img.load()
        for x in range(W):
            for y in range(H):
                px[x, y] = 255 if self.fb[x][y] else 0
        return img.resize((W * scale, H * scale), Image.NEAREST)


# ---------------------------------------------------------------- 四个页面

def page_live(cn, asc, bright=2600, threshold=1800, pot=2048, alarm=False, alarm_en=True, mute=False):
    s = Screen()
    if alarm:
        s.cn_banner(0, "报警中", cn)
    else:
        s.cn_text(0, 0, "光照监测", cn)
        s.ascii_text(100, 15, "1/4", asc)
        if mute:
            s.cn_text(64, 0, "静音", cn)

    s.cn_text(0, 16, "光照", cn)
    s.bar(36, 20, 62, 8, bright, 4095)
    s.ascii_text(100, 31, "%4u" % bright, asc)

    s.cn_text(0, 32, "阈值", cn)
    s.bar(36, 36, 62, 8, threshold, 4095)
    s.ascii_text(100, 47, "%4u" % threshold, asc)

    s.cn_text(0, 48, "电位", cn)
    s.ascii_text(36, 63, "%4u" % pot, asc)
    s.cn_text(68, 48, "报警", cn)
    s.cn_text(104, 48, "开" if alarm_en else "关", cn)
    return s


def page_curve(cn, asc):
    s = Screen()
    s.cn_text(0, 0, "亮度曲线", cn)
    s.ascii_text(100, 15, "2/4", asc)
    s.rect_frame(0, 17, W, 39)
    s.curve([int(2048 + 1500 * math.sin(i / 9.0)) for i in range(W)])
    s.ascii_text(0, 64, "now:2600  len:128", asc)
    return s


def page_setting(cn, asc, threshold=1800, alarm_en=True, saved=False):
    s = Screen()
    s.cn_text(0, 0, "参数设置", cn)
    s.ascii_text(100, 15, "3/4", asc)

    s.cn_text(0, 16, "阈值", cn)
    s.bar(36, 20, 62, 8, threshold, 4095)
    s.ascii_text(100, 31, "%4u" % threshold, asc)

    s.cn_text(0, 32, "报警", cn)
    s.cn_text(36, 32, "开" if alarm_en else "关", cn)

    if saved:
        s.cn_text(0, 48, "已保存", cn)
    else:
        s.ascii_text(0, 64, "K2K3 +/-  K4 DEFLT", asc)
    return s


def page_boot(cn, asc):
    s = Screen()
    s.cn_text(0, 0, "光照监测", cn)
    s.ascii_text(100, 15, "v1.1", asc)
    s.ascii_text(0, 31, "STM32F103C8T6", asc)
    s.ascii_text(0, 47, "ADC PA0 PA1", asc)
    s.ascii_text(0, 63, "PWM PB6  LCD PB8/9", asc)
    return s


def page_history(cn, asc, count=1234, avg=2100, mx=3900, mn=180):
    s = Screen()
    s.cn_text(0, 0, "历史记录", cn)
    s.ascii_text(100, 15, "4/4", asc)

    s.cn_text(0, 16, "共", cn)
    s.ascii_text(20, 31, "%u" % count, asc)
    s.cn_text(48, 16, "条", cn)
    s.cn_text(64, 16, "平均", cn)
    s.ascii_text(100, 31, "%4u" % avg, asc)

    s.cn_text(0, 32, "最大", cn)
    s.ascii_text(34, 47, "%4u" % mx, asc)
    s.cn_text(66, 32, "最小", cn)
    s.ascii_text(100, 47, "%4u" % mn, asc)

    s.ascii_text(0, 64, "Hold K3 clear", asc)
    return s


USED_CHINESE = ["光照监测", "静音", "光照", "阈值", "报警中", "报警", "开", "关",
                "正常", "亮度曲线", "参数设置", "已保存", "历史记录", "共", "条",
                "平均", "最大", "最小", "已清除"]


def main():
    cn = parse_cn_font(CN_H)
    asc = parse_ascii_font(ASCII_H)
    print("中文字库 %d 个字；英文/数字字库 %d 个字形" % (len(cn), len(asc)))

    missing = set()
    for s in USED_CHINESE:
        for ch in s:
            if ord(ch) > 0x7F and ord(ch) not in cn:
                missing.add(ch)
    print("缺字: %s" % ("".join(sorted(missing)) if missing else "无，全部都有"))

    pages = [
        ("1/4 实时(正常)", page_live(cn, asc)),
        ("1/4 实时(报警)", page_live(cn, asc, bright=900, alarm=True)),
        ("2/4 曲线", page_curve(cn, asc)),
        ("3/4 设置", page_setting(cn, asc)),
        ("4/4 历史", page_history(cn, asc)),
        ("开机画面", page_boot(cn, asc)),
    ]

    scale, pad, label_h = 4, 12, 16
    cols = 2
    cell_w = W * scale + pad
    cell_h = H * scale + pad + label_h
    rows = (len(pages) + cols - 1) // cols

    sheet = Image.new("L", (cols * cell_w + pad, rows * cell_h + pad), 40)
    draw = ImageDraw.Draw(sheet)

    for i, (title, scr) in enumerate(pages):
        cx = pad + (i % cols) * cell_w
        cy = pad + (i // cols) * cell_h
        sheet.paste(scr.image(scale), (cx, cy))
        draw.text((cx + 2, cy + H * scale + 3), title, fill=220)

    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "ui_preview.png")
    sheet.save(out)
    print("已生成预览图:", out)


if __name__ == "__main__":
    main()
