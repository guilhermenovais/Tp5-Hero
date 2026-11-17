#!/usr/bin/env python3
"""Gráficos e métricas do ensaio de desvio de obstáculos (avoid).

Lê exercicios/avoid/avoid_log_A.csv e avoid_log_B.csv (colunas: t, x, y, theta,
min_range, v, w, mode) e gera, em analise/figuras/ e analise/tabelas/:
  avoid_trajetoria_<C>.png   trajetória na arena (paredes 2 x 2 m e cilindro)
  avoid_distancia_<C>.png    menor distância do lidar x tempo, com a linha de 0,045 m
  avoid_comparacao.png       A e B lado a lado
  avoid_metricas.csv / .md   tabela de métricas A vs B

Uso:  python analise/plot_avoid.py [--dir exercicios/avoid]
"""
import argparse
from pathlib import Path

import numpy as np

import common as c
from common import plt

LIMIAR_COLISAO = 0.045  # m, menor distância do lidar abaixo da qual há colisão
CELULA = 0.25  # m, lado das células usadas na cobertura da arena (grade 8 x 8)


def contar_colisoes(min_range):
    """Número de EVENTOS: cada vez que a menor distância cruza abaixo do limiar."""
    abaixo = (np.asarray(min_range) < LIMIAR_COLISAO)
    anterior = np.r_[False, abaixo[:-1]]
    return int((abaixo & ~anterior).sum())


def metricas(df):
    t = df["t"].to_numpy()
    xy = df[["x", "y"]].dropna()
    passos = np.hypot(np.diff(xy["x"].to_numpy()), np.diff(xy["y"].to_numpy()))
    celulas = {(int((x + c.ARENA) / CELULA), int((y + c.ARENA) / CELULA)) for x, y in xy.to_numpy()}
    modo = df["mode"].to_numpy()
    entradas_escape = int(((modo == "escape") & np.r_[True, modo[:-1] != "escape"]).sum())
    return {
        "Duração do ensaio (s)": round(float(t[-1] - t[0]), 1),
        "Distância percorrida (m)": round(float(passos.sum()), 2),
        "Colisões (eventos com min < 0,045 m)": contar_colisoes(df["min_range"]),
        "Amostras com min < 0,045 m": int((df["min_range"] < LIMIAR_COLISAO).sum()),
        "Menor distância do lidar (m)": round(float(df["min_range"].min()), 3),
        "Menor distância, percentil 5 (m)": round(float(df["min_range"].quantile(0.05)), 3),
        "Velocidade linear média (m/s)": round(float(df["v"].mean()), 3),
        "Tempo em modo desvio (%)": round(100.0 * float((modo == "avoid").mean()), 1),
        "Acionamentos do anti-travamento": entradas_escape,
        "Cobertura da arena (células de 0,25 m, de 64)": len(celulas),
    }


def desenhar_trajetoria(ax, df, cfg):
    xy = df.dropna(subset=["x", "y"])
    c.desenhar_arena(ax, cilindro=True)
    ax.plot(xy["x"], xy["y"], color=c.COR[cfg], linewidth=1.1, alpha=0.85, zorder=3, label="Trajetória")
    ax.plot(xy["x"].iloc[0], xy["y"].iloc[0], "o", color=c.TEXTO, markersize=8, markeredgecolor=c.SUPERFICIE,
            markeredgewidth=2, zorder=4, label="Início")
    ax.plot(xy["x"].iloc[-1], xy["y"].iloc[-1], "s", color=c.TEXTO, markersize=8, markeredgecolor=c.SUPERFICIE,
            markeredgewidth=2, zorder=4, label="Fim")
    bate = df[df["min_range"] < LIMIAR_COLISAO].dropna(subset=["x", "y"])
    if len(bate):
        ax.plot(bate["x"], bate["y"], "x", color=c.CRITICO, markersize=9, zorder=5, label="Colisão")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=4)


def desenhar_distancia(ax, df, cfg):
    ax.plot(df["t"], df["min_range"], color=c.COR[cfg], linewidth=1.0, label="Menor distância do lidar")
    ax.axhline(LIMIAR_COLISAO, color=c.CRITICO, linestyle="--", linewidth=1.3,
               label="Limiar de colisão (0,045 m)")
    ax.set_ylim(0, 0.52)
    ax.set_xlim(df["t"].iloc[0], df["t"].iloc[-1])
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Menor distância do lidar (m)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default=str(c.RAIZ / "exercicios" / "avoid"), help="pasta com avoid_log_<C>.csv")
    args = ap.parse_args()
    c.aplicar_estilo()

    logs = {cfg: c.ler_log(Path(args.dir) / ("avoid_log_%s.csv" % cfg)) for cfg in c.CONFIGS}
    logs = {k: v for k, v in logs.items() if v is not None}
    if not logs:
        raise SystemExit("nenhum avoid_log_<C>.csv encontrado em %s" % args.dir)

    tabela = {}
    for cfg, df in logs.items():
        tabela[cfg] = metricas(df)
        fig, ax = plt.subplots(figsize=(5.6, 6.2))
        desenhar_trajetoria(ax, df, cfg)
        ax.set_title("avoid: trajetória, %s" % c.NOME[cfg])
        c.salvar_figura(fig, "avoid_trajetoria_" + cfg)

        fig, ax = plt.subplots(figsize=(8.5, 4.4))
        desenhar_distancia(ax, df, cfg)
        ax.set_title("avoid: menor distância do lidar, %s" % c.NOME[cfg])
        c.salvar_figura(fig, "avoid_distancia_" + cfg)

    if len(logs) == 2:
        fig, eixos = plt.subplots(2, 2, figsize=(11, 11.4), gridspec_kw={"height_ratios": [1.5, 1]})
        for j, cfg in enumerate(c.CONFIGS):
            desenhar_trajetoria(eixos[0][j], logs[cfg], cfg)
            eixos[0][j].set_title(c.NOME[cfg])
            desenhar_distancia(eixos[1][j], logs[cfg], cfg)
        fig.suptitle("avoid: comparação A vs B", fontsize=14, fontweight="bold")
        fig.tight_layout()
        c.salvar_figura(fig, "avoid_comparacao")

    md = c.salvar_tabela(tabela, "avoid_metricas")
    print("\n" + md)
    for cfg, m in tabela.items():
        print("\n[%s] distância percorrida: %.2f m | colisões: %d"
              % (cfg, m["Distância percorrida (m)"], m["Colisões (eventos com min < 0,045 m)"]))


if __name__ == "__main__":
    main()
