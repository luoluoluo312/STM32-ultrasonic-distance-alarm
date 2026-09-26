# -*- coding: utf-8 -*-
"""
从 TTF 字体生成 16x16 中文点阵字库（C 代码），用于 main.c 里的 OLED 中文显示。

用法:
    python gen_cn_font.py            # 输出到屏幕
    python gen_cn_font.py out.txt    # 输出到文件

点阵格式（和 my_lib 的 OLED 驱动 DrawBitmapEx 一致）:
    行优先，每行 2 字节（16 像素），字节内高位在左(MSB first)，共 32 字节/字。

要加字：把新汉字填进 CHARS 字符串，重新运行，再把输出贴回 main.c 即可。
"""

import os
import sys

from PIL import Image, ImageDraw, ImageFont

# 用工程自带的黑体（也可以换成 my_lib/font/ttfttc 下其它字体）
HERE = os.path.dirname(os.path.abspath(__file__))
TTF = os.path.join(HERE, "..", "my_lib", "font", "ttfttc", "黑体.ttf")

FONT_SIZE = 16
PIXEL = 16

# 界面里用到的全部汉字（重复的会自动去重）
CHARS = (
    "光照监测"      # 第1页标题
    "光照电位阈值报警中正常"  # 第1页各标签与状态
    "开关"
    "亮度曲线"      # 第2页标题
    "参数设置"      # 第3页标题
    "已保存"
    "历史记录"      # 第4页标题
    "共条平均最大最小"  # 第4页统计
    "清除"
    "静音"          # 静音提示
)


def render_char(ch):
    """把一个汉字渲染成 16x16 的 0/1 矩阵"""
    font = ImageFont.truetype(TTF, FONT_SIZE)
    img = Image.new("1", (PIXEL, PIXEL), 0)
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), ch, font=font, fill=1, anchor="la")

    rows = []
    for y in range(PIXEL):
        row = []
        for x in range(PIXEL):
            row.append(1 if img.getpixel((x, y)) else 0)
        rows.append(row)
    return rows


def pack(rows):
    """按 行优先、每行 2 字节、MSB 在左 打包"""
    data = []
    for y in range(PIXEL):
        hi = 0
        lo = 0
        for x in range(8):
            if rows[y][x]:
                hi |= 0x80 >> x
        for x in range(8, 16):
            if rows[y][x]:
                lo |= 0x80 >> (x - 8)
        data.append(hi)
        data.append(lo)
    return data


def check_clip(ch, rows):
    """提示可能被裁掉的笔画（贴边）"""
    warn = []
    if any(rows[0]):
        warn.append("顶边")
    if any(rows[PIXEL - 1]):
        warn.append("底边")
    if any(r[0] for r in rows):
        warn.append("左边")
    if any(r[PIXEL - 1] for r in rows):
        warn.append("右边")
    if warn and ch not in "一":
        sys.stderr.write("提示: 字符 %s 贴到 %s，可能被裁\n" % (ch, "/".join(warn)))


def main():
    chars = []
    for ch in CHARS:
        if ch not in chars:
            chars.append(ch)

    out = []
    out.append("/* ==== 16x16 中文点阵字库（由 my_app/gen_cn_font.py 自动生成）==== */")
    out.append("/* 格式: 行优先, 每行 2 字节, 高位在左, 共 32 字节/字 */")
    out.append("")

    table = []
    for ch in chars:
        rows = render_char(ch)
        check_clip(ch, rows)
        data = pack(rows)
        name = "cn_%04X" % ord(ch)
        out.append("static const uint8_t %s[32] = { /* %s */" % (name, ch))
        for i in range(0, 32, 8):
            out.append("    " + ",".join("0x%02X" % b for b in data[i:i + 8]) + ",")
        out.append("};")
        table.append((ord(ch), name, ch))

    out.append("")
    out.append("typedef struct")
    out.append("{")
    out.append("    uint16_t       code;   /* Unicode 编码 */")
    out.append("    const uint8_t *bmp;    /* 32 字节点阵  */")
    out.append("} CnGlyph_TypeDef;")
    out.append("")
    out.append("static const CnGlyph_TypeDef g_cnGlyphs[] = {")
    for code, name, ch in table:
        out.append("    {0x%04X, %s},  /* %s */" % (code, name, ch))
    out.append("};")
    out.append("")
    out.append("#define CN_GLYPH_COUNT (sizeof(g_cnGlyphs) / sizeof(g_cnGlyphs[0]))")
    out.append("")

    text = "\n".join(out) + "\n"

    if len(sys.argv) > 1:
        with open(sys.argv[1], "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        sys.stderr.write("已生成 %d 个汉字 -> %s\n" % (len(chars), sys.argv[1]))
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
