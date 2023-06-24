# !/usr/bin/env python
# coding=utf-8
import numpy as np

def TD_func_fast(v, x1, x2, r, h):
    """
    参数:
    v: 源信号
    x1: 跟踪信号
    x2: 跟踪微分信号
    r: 快速因子
    h: 步长

    返回:
    x1: 跟踪信号
    x2: 跟踪微分信号
    """
    # 快速因子r=1, 步长h, h0 = 1 * h, 自抗扰控制技术p73, 最速离散跟踪微分器 快速跟踪微分
    fh = fhan(x1 - v, x2, r, 1 * h)
    x1 = x1 + h * x2
    x2 = x2 + h * fh
    return x1, x2

def fhan(x1, x2, r, h):
    d = r * h ** 2
    a0 = h * x2
    y = x1 + a0
    a1 = (d * (d + 8 * abs(y))) ** 0.5
    a2 = a0 + np.sign(y) * (a1 - d) / 2.
    a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d))
    fh = -r * (a / d) * fsg(a, d) - r * np.sign(a) * (1 - fsg(a, d))
    return fh

def fsg(x, d):
    return (np.sign(x + d) - np.sign(x - d)) / 2.0

if __name__ == '__main__':
    # 测试跟踪微分器
    import matplotlib.pyplot as plt
    h = 0.2
    r = 1
    x0 = 0.5
    y0 = 0.5
    n = 200
    x = np.zeros([n, 2])
    v = np.zeros(n)
    v[0] = 1

    for k in range(1, n):
        v[k] = np.sin(k*h) + np.random.randn()*0.01 + 1
        x[0, :] = [v[1], 0]
        x[k, 0], x[k, 1] = TD_func_fast(v[k-1], x[k-1, 0], x[k-1, 1], r, h)

    fig = plt.figure()
    plt.plot(x[:, 0], color='red', label='track')
    plt.plot(v, color='blue', label='v')
    plt.plot(x[:, 1], color='green', label='vel')

    plt.legend()
    plt.show()


    # # 实际使用：Pose_X为MFC计算输出的位置
    # self.Pose_TD['pos_track'][0], self.Pose_TD['pos_diff'][0] \
    #     = ADRC_TD.TD_func_fast(self.Pose_X,
    #                            self.Pose_TD_pre['pos_track'][0],
    #                            self.Pose_TD_pre['pos_diff'][0],
    #                            self.r,
    #                            self.h)
