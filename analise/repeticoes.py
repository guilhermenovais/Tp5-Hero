#!/usr/bin/env python3
"""Consolida as repetições dos ensaios: valor por execução, média e desvio-padrão.

Procura, em exercicios/<goto|avoid>/, os CSVs de cada configuração:
  <ex>_log_<C>.csv        execução 1 (ensaio final, semente 1)
  <ex>_log_<C>_s<N>.csv   execução N (rodada por scripts/ensaio.sh <ex> <C> <N>)
e grava analise/tabelas/<ex>_repeticoes.csv / .md.

O desvio-padrão é amostral (n - 1). Com poucas execuções (n = 3) ele é só uma indicação de
variabilidade, não um intervalo de confiança.

ATENÇÃO: no goto a semente muda a sequência de metas. No avoid a semente do simulador NÃO muda
nada (arena fixa, sem sorteio): a variação vem do sorteio do wander e da escolha do lado em
empates, feitos pelo nó sem semente. Por isso as execuções do avoid são independentes, não
"sementes".

Uso:  python analise/repeticoes.py
"""
import csv
import re
import statistics
from pathlib import Path

import common as c
import plot_avoid
import plot_goto

EXERCICIOS = {
    "goto": (plot_goto.metricas, c.RAIZ / "exercicios" / "goto"),
    "avoid": (plot_avoid.metricas, c.RAIZ / "exercicios" / "avoid"),
}


def achar_execucoes(pasta, ex, cfg):
    """{numero da execução: caminho}. O arquivo sem sufixo é a execução 1."""
    achados = {}
    base = pasta / ("%s_log_%s.csv" % (ex, cfg))
    if base.exists():
        achados[1] = base
    for p in pasta.glob("%s_log_%s_s*.csv" % (ex, cfg)):
        m = re.search(r"_s(\d+)\.csv$", p.name)
        if m:
            achados[int(m.group(1))] = p
    return dict(sorted(achados.items()))


def fmt(v):
    return "%.4g" % v  # 4 algarismos significativos: 12, 9.9, 0.0917, 15.67


def consolidar(ex):
    metricas, pasta = EXERCICIOS[ex]
    por_cfg = {}
    for cfg in c.CONFIGS:
        execs = achar_execucoes(pasta, ex, cfg)
        if not execs:
            continue
        por_cfg[cfg] = {n: metricas(c.ler_log(p)) for n, p in execs.items()}
    if not por_cfg:
        print("sem logs de %s" % ex)
        return
    nomes = [m for m, v in next(iter(next(iter(por_cfg.values())).values())).items()
             if isinstance(v, (int, float))]
    cab = ["Métrica"]
    for cfg in por_cfg:
        cab += ["%s: média ± dp" % cfg, "%s: por execução (%s)" % (cfg, ", ".join("#%d" % n for n in por_cfg[cfg]))]
    linhas = []
    for m in nomes:
        linha = [m]
        for cfg, execs in por_cfg.items():
            vals = [execs[n][m] for n in execs]
            dp = statistics.stdev(vals) if len(vals) > 1 else float("nan")
            linha += ["%s ± %s" % (fmt(statistics.mean(vals)), fmt(dp)), " / ".join(fmt(v) for v in vals)]
        linhas.append(linha)

    c.TAB_DIR.mkdir(parents=True, exist_ok=True)
    with open(c.TAB_DIR / ("%s_repeticoes.csv" % ex), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cab)
        w.writerows(linhas)
    md = ["| " + " | ".join(cab) + " |", "|" + "|".join(["---"] * len(cab)) + "|"]
    md += ["| " + " | ".join(l) + " |" for l in linhas]
    (c.TAB_DIR / ("%s_repeticoes.md" % ex)).write_text("\n".join(md) + "\n", encoding="utf-8")
    print("\n== %s (execuções: %s)" % (ex, {cfg: list(e) for cfg, e in por_cfg.items()}))
    print("\n".join(md))


def main():
    for ex in EXERCICIOS:
        consolidar(ex)


if __name__ == "__main__":
    main()
