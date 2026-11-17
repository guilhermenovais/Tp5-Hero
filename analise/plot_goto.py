#!/usr/bin/env python3
"""Gráficos e métricas do ensaio de navegação até metas (goto).

Lê exercicios/goto/goto_log_A.csv e goto_log_B.csv (colunas: t, x, y, theta, xg, yg,
rho, alpha, v, w, goal_reached) e gera, em analise/figuras/ e analise/tabelas/:
  goto_trajetoria_<C>.png   trajetória com as metas atingidas marcadas e numeradas
  goto_rho_alpha_<C>.png    rho e alpha x tempo
  goto_comparacao.png       A e B lado a lado
  goto_metricas.csv / .md   tabela de métricas A vs B

Definição: a linha com goal_reached = 1 é a primeira com a NOVA meta; a meta atingida é a
que estava em (xg, yg) na linha anterior. O tempo de cada meta é medido entre atingimentos
sucessivos (a primeira, desde t = 0).

Uso:  python analise/plot_goto.py [--dir exercicios/goto]
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd

import common as c
from common import plt

RAIO_META = 0.06  # m, o simulador sorteia nova meta abaixo desta distância


def metas_atingidas(df):
    """Índices das linhas de troca de meta, tempos e posições das metas atingidas."""
    idx = np.flatnonzero(df["goal_reached"].to_numpy() == 1)
    idx = idx[idx > 0]
    tempos = df["t"].to_numpy()[idx]
    pos = df[["xg", "yg"]].to_numpy()[idx - 1]  # meta que valia na linha anterior
    return idx, tempos, pos


def metricas(df):
    idx, tempos, _ = metas_atingidas(df)
    t = df["t"].to_numpy()
    xy = df[["x", "y"]].to_numpy()
    dist = float(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])).sum())
    intervalos = np.diff(np.r_[t[0], tempos]) if len(tempos) else np.array([])
    andando = df["v"] > 0.05
    w = df["w"].to_numpy()
    perto = df["rho"].to_numpy() < 0.15
    trocas_w = int(((w[1:] * w[:-1]) < 0)[perto[1:]].sum())
    return {
        "Duração do ensaio (s)": round(float(t[-1] - t[0]), 1),
        "Metas atingidas": int(len(tempos)),
        "Tempo médio por meta (s)": round(float(intervalos.mean()), 1) if len(intervalos) else "n/d",
        "Tempo mediano por meta (s)": round(float(np.median(intervalos)), 1) if len(intervalos) else "n/d",
        "Menor / maior tempo por meta (s)": ("%.1f / %.1f" % (intervalos.min(), intervalos.max())) if len(intervalos) else "n/d",
        "Distância percorrida (m)": round(dist, 2),
        "Velocidade linear média (m/s)": round(float(df["v"].mean()), 3),
        "|alpha| médio com v > 0,05 m/s (graus)": round(float(np.degrees(df.loc[andando, "alpha"].abs().mean())), 1),
        "Tempo girando no lugar, v = 0 (%)": round(100.0 * float((df["v"] < 1e-9).mean()), 1),
        "Trocas de sinal de w com rho < 0,15 m": trocas_w,
    }


def desenhar_trajetoria(ax, df, cfg):
    _, _, pos = metas_atingidas(df)
    c.desenhar_arena(ax)
    ax.plot(df["x"], df["y"], color=c.COR[cfg], linewidth=1.1, alpha=0.85, zorder=3, label="Trajetória")
    ax.plot(df["x"].iloc[0], df["y"].iloc[0], "o", color=c.TEXTO, markersize=8, markeredgecolor=c.SUPERFICIE,
            markeredgewidth=2, zorder=5, label="Início")
    if len(pos):
        ax.scatter(pos[:, 0], pos[:, 1], s=130, facecolor=c.SUPERFICIE, edgecolor=c.COR[cfg], linewidth=2,
                   zorder=4, label="Meta atingida (%d)" % len(pos))
        for k, (gx, gy) in enumerate(pos, start=1):
            ax.annotate(str(k), (gx, gy), ha="center", va="center", fontsize=6.5, color=c.TEXTO, zorder=6)
    gx, gy = df["xg"].iloc[-1], df["yg"].iloc[-1]
    ax.scatter([gx], [gy], s=130, facecolor="none", edgecolor=c.MUDO, linewidth=1.6, linestyle="--", zorder=4,
               label="Meta em curso no fim")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=2)


def desenhar_rho(ax, df, cfg):
    ax.plot(df["t"], df["rho"], color=c.COR[cfg], linewidth=1.0, label="rho (distância à meta)")
    ax.axhline(RAIO_META, color=c.CRITICO, linestyle="--", linewidth=1.2, label="Raio da meta (0,06 m)")
    ax.set_ylabel("rho (m)")
    ax.set_xlim(df["t"].iloc[0], df["t"].iloc[-1])
    ax.set_ylim(0, 1.95)  # folga em cima para a legenda não cobrir os picos
    ax.legend(loc="upper right", ncol=2)


def desenhar_alpha(ax, df, cfg):
    ax.plot(df["t"], np.degrees(df["alpha"]), color=c.COR[cfg], linewidth=1.0, label="alpha (erro de orientação)")
    ax.axhline(0, color=c.EIXO, linewidth=1.0)
    ax.set_ylabel("alpha (graus)")
    ax.set_xlabel("Tempo (s)")
    ax.set_xlim(df["t"].iloc[0], df["t"].iloc[-1])
    ax.set_ylim(-190, 270)  # folga em cima para a legenda
    ax.set_yticks([-180, -90, 0, 90, 180])
    ax.legend(loc="upper right")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default=str(c.RAIZ / "exercicios" / "goto"), help="pasta com goto_log_<C>.csv")
    args = ap.parse_args()
    c.aplicar_estilo()

    logs = {cfg: c.ler_log(Path(args.dir) / ("goto_log_%s.csv" % cfg)) for cfg in c.CONFIGS}
    logs = {k: v for k, v in logs.items() if v is not None}
    if not logs:
        raise SystemExit("nenhum goto_log_<C>.csv encontrado em %s" % args.dir)

    tabela = {}
    for cfg, df in logs.items():
        tabela[cfg] = metricas(df)
        fig, ax = plt.subplots(figsize=(5.8, 6.2))
        desenhar_trajetoria(ax, df, cfg)
        ax.set_title("goto: trajetória e metas, %s" % c.NOME[cfg])
        c.salvar_figura(fig, "goto_trajetoria_" + cfg)

        fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        desenhar_rho(a1, df, cfg)
        desenhar_alpha(a2, df, cfg)
        a1.set_title("goto: rho e alpha, %s" % c.NOME[cfg])
        fig.tight_layout()
        c.salvar_figura(fig, "goto_rho_alpha_" + cfg)

    if len(logs) == 2:
        fig, eixos = plt.subplots(3, 2, figsize=(12, 13.5), gridspec_kw={"height_ratios": [2.2, 1, 1]})
        for j, cfg in enumerate(c.CONFIGS):
            desenhar_trajetoria(eixos[0][j], logs[cfg], cfg)
            eixos[0][j].set_title(c.NOME[cfg])
            desenhar_rho(eixos[1][j], logs[cfg], cfg)
            desenhar_alpha(eixos[2][j], logs[cfg], cfg)
        fig.suptitle("goto: comparação A vs B (mesma sequência de metas)", fontsize=14, fontweight="bold")
        fig.tight_layout()
        fig.subplots_adjust(hspace=0.55, top=0.94)  # espaço para a legenda sob a trajetória
        c.salvar_figura(fig, "goto_comparacao")

    md = c.salvar_tabela(tabela, "goto_metricas")
    print("\n" + md)
    for cfg, m in tabela.items():
        print("\n[%s] metas atingidas: %s | tempo médio por meta: %s s"
              % (cfg, m["Metas atingidas"], m["Tempo médio por meta (s)"]))


if __name__ == "__main__":
    main()
