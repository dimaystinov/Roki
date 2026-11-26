# -*- coding: utf-8 -*-
r"""Заглушка вместо компилированного starkit.

В ELSIROS обратная кинематика ног вызывается как `starkit.alpha_calculation`,
и это .pyd, собранный под Python 3.9 - на нашем 3.12 он не грузится. Но рядом
в их же файле `compute_Alpha_v3.py` лежит тот же расчёт на чистом Python, и в
`class_Motion.py` он даже стоит закомментированной строкой рядом с вызовом
starkit. Здесь мы просто подставляем его.
"""
from Soccer.Motion.compute_Alpha_v3 import Alpha

_al = Alpha()


def alpha_calculation(xt, yt, zt, x, y, z, w, sizes, limAlpha):
    return _al.compute_Alpha_v3(xt, yt, zt, x, y, z, w, sizes, limAlpha)
