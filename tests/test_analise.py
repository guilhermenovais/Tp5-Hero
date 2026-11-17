"""Testes da lógica de métricas dos scripts de análise (precisa de pandas/matplotlib).

Rode da raiz do kit:  python tests/test_analise.py
"""
import os
import sys
import unittest

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "analise"))

import plot_avoid  # noqa: E402
import plot_goto  # noqa: E402


class ColisoesTests(unittest.TestCase):
    def test_conta_eventos_e_nao_amostras(self):
        # 3 amostras seguidas abaixo de 0,045 = 1 colisão; depois sai e entra de novo = 2
        m = [0.3, 0.04, 0.03, 0.04, 0.2, 0.3, 0.044, 0.5]
        self.assertEqual(plot_avoid.contar_colisoes(m), 2)

    def test_limiar_e_estrito(self):
        self.assertEqual(plot_avoid.contar_colisoes([0.3, 0.045, 0.3]), 0)  # igual ao limiar não conta
        self.assertEqual(plot_avoid.contar_colisoes([0.3, 0.0449, 0.3]), 1)

    def test_comeca_abaixo_do_limiar(self):
        self.assertEqual(plot_avoid.contar_colisoes([0.01, 0.01, 0.3]), 1)

    def test_sem_colisao(self):
        self.assertEqual(plot_avoid.contar_colisoes([0.5] * 10), 0)


class MetasTests(unittest.TestCase):
    def df(self):
        # meta 1 em (0.5, 0): atingida na linha 3 (troca para (0, 0.5)); meta 2 atingida na linha 6
        n = 8
        gx = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0, -0.5, -0.5]
        gy = [0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.0, 0.0]
        reached = [0, 0, 0, 1, 0, 0, 1, 0]
        return pd.DataFrame({
            "t": np.arange(n) * 1.0, "x": np.linspace(0, 1, n), "y": np.zeros(n), "theta": np.zeros(n),
            "xg": gx, "yg": gy, "rho": np.ones(n), "alpha": np.zeros(n), "v": np.full(n, 0.1),
            "w": np.zeros(n), "goal_reached": reached,
        })

    def test_posicao_da_meta_atingida_e_a_da_linha_anterior(self):
        idx, tempos, pos = plot_goto.metas_atingidas(self.df())
        self.assertEqual(list(idx), [3, 6])
        self.assertEqual(list(tempos), [3.0, 6.0])
        np.testing.assert_allclose(pos, [[0.5, 0.0], [0.0, 0.5]])

    def test_metricas(self):
        m = plot_goto.metricas(self.df())
        self.assertEqual(m["Metas atingidas"], 2)
        self.assertAlmostEqual(m["Tempo médio por meta (s)"], 3.0)  # intervalos 3 e 3, a primeira desde t = 0

    def test_sem_metas(self):
        d = self.df()
        d["goal_reached"] = 0
        m = plot_goto.metricas(d)
        self.assertEqual(m["Metas atingidas"], 0)
        self.assertEqual(m["Tempo médio por meta (s)"], "n/d")


if __name__ == "__main__":
    unittest.main(verbosity=2)
