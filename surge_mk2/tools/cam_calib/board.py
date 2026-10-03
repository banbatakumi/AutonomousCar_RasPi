"""印刷用チェッカーボードを作る。

    .venv/bin/python -m tools.cam_calib --make-board 9x6 --square 0.025 -o board.pdf

`board` は**内側の角点の数**（OpenCV の流儀）。9x6 なら白黒 10×7 マス。
PDF/PNG に DPI を埋め込むので、**「実際のサイズ」（100%）で印刷**すれば
1マスが `square` [m] になる。プリンタは黙って縮めることがあるので、
印刷後に必ず定規で測り、校正にはその実測値を入れる。

ボードは平らな板（段ボール・アクリル板）に貼る。たわんだ紙は平面という前提が崩れる。
"""

from __future__ import annotations

import numpy as np

__all__ = ["make_board"]

#: マスの外側の白い余白（マス何個ぶんか）。端の角点の検出には白い縁が要る
_MARGIN_SQUARES = 1.0


def make_board(board: tuple[int, int], square_m: float, dpi: int = 300) -> np.ndarray:
    """白黒のチェッカーボード画像（uint8 グレースケール）。

    左上のマスが黒。`dpi` で 1 マスの画素数が決まる（印刷時に同じ DPI で出す）。
    """
    cols, rows = board[0] + 1, board[1] + 1          # 角点数 → マス数
    px = int(round(square_m / 0.0254 * dpi))
    margin = int(round(_MARGIN_SQUARES * px))
    img = np.full((rows * px + 2 * margin, cols * px + 2 * margin), 255, np.uint8)
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                y, x = margin + r * px, margin + c * px
                img[y:y + px, x:x + px] = 0
    return img


def save_board(path: str, board: tuple[int, int], square_m: float, dpi: int = 300) -> None:
    """PNG/PDF に DPI 付きで保存する（Pillow）。"""
    from PIL import Image

    img = Image.fromarray(make_board(board, square_m, dpi))
    if str(path).lower().endswith(".pdf"):
        img.save(path, "PDF", resolution=float(dpi))
    else:
        img.save(path, dpi=(dpi, dpi))
