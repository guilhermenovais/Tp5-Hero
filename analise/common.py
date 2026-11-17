"""Estilo, caminhos e utilidades compartilhados por plot_avoid.py e plot_goto.py."""
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # só grava arquivos: não precisa de janela
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parent
FIG_DIR = AQUI / "figuras"
TAB_DIR = AQUI / "tabelas"

CONFIGS = ("A", "B")
# Paleta categórica validada (slots 1 e 2): A = azul, B = laranja.
COR = {"A": "#2a78d6", "B": "#eb6834"}
NOME = {"A": "Config. A (conservadora)", "B": "Config. B (agressiva)"}

SUPERFICIE = "#fcfcfb"
TEXTO = "#0b0b0b"
TEXTO_2 = "#52514e"
MUDO = "#898781"
GRADE = "#e1e0d9"
EIXO = "#c3c2b7"
CRITICO = "#d03b3b"  # só para o limiar de colisão (rótulo sempre presente)
CILINDRO = "#d95933"  # cor do cilindro no simulador (0.85, 0.35, 0.2)

ARENA = 1.0  # a arena é o quadrado [-1, 1] m
RAIO_CILINDRO = 0.15


def aplicar_estilo():
    plt.rcParams.update({
        "figure.facecolor": SUPERFICIE,
        "axes.facecolor": SUPERFICIE,
        "savefig.facecolor": SUPERFICIE,
        "axes.edgecolor": EIXO,
        "axes.labelcolor": TEXTO_2,
        "axes.titlecolor": TEXTO,
        "axes.titlesize": 12,
        "axes.titleweight": "bold",
        "axes.labelsize": 10,
        "axes.grid": True,
        "grid.color": GRADE,
        "grid.linewidth": 0.8,
        "axes.axisbelow": True,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "xtick.color": MUDO,
        "ytick.color": MUDO,
        "text.color": TEXTO,
        "legend.frameon": False,
        "legend.fontsize": 9,
        "legend.labelcolor": TEXTO_2,
        "font.family": "sans-serif",
        "font.sans-serif": ["Segoe UI", "DejaVu Sans", "Arial"],
        "lines.linewidth": 1.4,
    })


def ler_log(caminho):
    """Lê um CSV de ensaio; devolve None se o arquivo não existe."""
    caminho = Path(caminho)
    if not caminho.exists():
        print("aviso: %s não encontrado, config ignorada" % caminho)
        return None
    return pd.read_csv(caminho)


def desenhar_arena(ax, cilindro=False):
    """Paredes 2 x 2 m (e o cilindro central, se pedido) em coordenadas do mundo."""
    ax.add_patch(plt.Rectangle((-ARENA, -ARENA), 2 * ARENA, 2 * ARENA, fill=False,
                               edgecolor=TEXTO_2, linewidth=2.2, zorder=2))
    if cilindro:
        ax.add_patch(plt.Circle((0, 0), RAIO_CILINDRO, facecolor=CILINDRO, edgecolor=CILINDRO,
                                alpha=0.85, zorder=2, label="Cilindro (r = 0,15 m)"))
    ax.set_xlim(-ARENA - 0.08, ARENA + 0.08)
    ax.set_ylim(-ARENA - 0.08, ARENA + 0.08)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def salvar_tabela(metricas, nome):
    """metricas: {config: {métrica: valor}}. Grava .csv e .md (inteiros continuam inteiros)."""
    TAB_DIR.mkdir(parents=True, exist_ok=True)
    configs = list(metricas)
    nomes = list(metricas[configs[0]])
    with open(TAB_DIR / (nome + ".csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Métrica"] + configs)
        for m in nomes:
            w.writerow([m] + [metricas[cfg][m] for cfg in configs])
    colunas = ["Métrica"] + configs
    linhas = ["| " + " | ".join(colunas) + " |", "|" + "|".join(["---"] * len(colunas)) + "|"]
    for m in nomes:
        linhas.append("| " + " | ".join([m] + [str(metricas[cfg][m]) for cfg in configs]) + " |")
    (TAB_DIR / (nome + ".md")).write_text("\n".join(linhas) + "\n", encoding="utf-8")
    return "\n".join(linhas)


def salvar_figura(fig, nome):
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    caminho = FIG_DIR / (nome + ".png")
    fig.savefig(caminho, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("figura:", caminho.relative_to(RAIZ))
