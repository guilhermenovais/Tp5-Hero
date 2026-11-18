#!/usr/bin/env python3
"""Diagrama dos nós e tópicos ROS usados no TP5 -> analise/figuras/diagrama_nos.png.

Uso:  python analise/diagrama_nos.py
"""
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

import common as c
from common import plt

AZUL = c.COR["A"]
LARANJA = c.COR["B"]


def caixa(ax, x0, y0, x1, y1, titulo, sub, cor=c.TEXTO_2):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle="round,pad=0,rounding_size=0.12",
                                facecolor="white", edgecolor=cor, linewidth=1.8, zorder=2))
    xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
    ax.text(xm, ym + 0.14, titulo, ha="center", va="center", fontsize=12, fontweight="bold", color=c.TEXTO, zorder=3)
    ax.text(xm, ym - 0.2, sub, ha="center", va="center", fontsize=8.5, color=c.TEXTO_2, zorder=3)


def seta(ax, xa, xb, y, topico, tipo, cor, tracejada=False):
    ax.add_patch(FancyArrowPatch((xa, y), (xb, y), arrowstyle="-|>", mutation_scale=13, color=cor, linewidth=1.6,
                                 linestyle=(0, (4, 3)) if tracejada else "-", shrinkA=0, shrinkB=0, zorder=1))
    xm = (xa + xb) / 2
    ax.text(xm, y + 0.1, topico, ha="center", va="bottom", fontsize=9, fontweight="bold", color=cor)
    ax.text(xm, y - 0.08, tipo, ha="center", va="top", fontsize=7.8, color=c.TEXTO_2)


def main():
    c.aplicar_estilo()
    fig, ax = plt.subplots(figsize=(10, 5.0))
    ax.set_xlim(0, 10)
    ax.set_ylim(0.1, 5.0)
    ax.axis("off")

    caixa(ax, 0.05, 2.1, 2.3, 3.1, "Simulador", "Genesis: física, lidar, juiz")
    caixa(ax, 3.25, 0.3, 5.05, 4.8, "hero_bridge", "ponte ROS")
    caixa(ax, 7.5, 3.0, 9.8, 4.8, "goto_0", "exercicios/goto/goto.py", cor=AZUL)
    caixa(ax, 7.5, 0.3, 9.8, 2.1, "avoid_0", "exercicios/avoid/avoid.py", cor=LARANJA)

    for xa, xb, y in ((2.3, 3.25, 2.7), (3.25, 2.3, 2.5)):
        ax.add_patch(FancyArrowPatch((xa, y), (xb, y), arrowstyle="-|>", mutation_scale=13, color=c.TEXTO_2,
                                     linewidth=1.6, shrinkA=0, shrinkB=0))
    ax.text(2.775, 2.88, "TCP 11312", ha="center", va="bottom", fontsize=8.5, color=c.TEXTO_2)
    ax.text(2.775, 2.32, "JSON", ha="center", va="top", fontsize=8.5, color=c.TEXTO_2)

    seta(ax, 5.05, 7.5, 4.35, "/hero_0/odom", "nav_msgs/Odometry (pose)", AZUL)
    seta(ax, 5.05, 7.5, 3.8, "/hero_0/goal", "geometry_msgs/PointStamped", AZUL)
    seta(ax, 7.5, 5.05, 3.25, "/hero_0/cmd_vel", "geometry_msgs/Twist", AZUL)
    seta(ax, 5.05, 7.5, 1.65, "/hero_0/laser", "sensor_msgs/LaserScan (8 feixes)", LARANJA)
    seta(ax, 5.05, 7.5, 1.1, "/hero_0/odom", "só para o registro, fora do controle", LARANJA, tracejada=True)
    seta(ax, 7.5, 5.05, 0.55, "/hero_0/cmd_vel", "geometry_msgs/Twist", LARANJA)

    c.salvar_figura(fig, "diagrama_nos")


if __name__ == "__main__":
    main()
