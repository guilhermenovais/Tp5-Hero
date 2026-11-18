# TP5: ROS e navegação com o HeRo (Projeto de Sistemas Robóticos, DCC/UFMG)

Dois nós ROS para o robô HeRo no simulador do kit `psr-hero-kit`:

| Nó | Tarefa | Entrada | Saída |
|---|---|---|---|
| [`avoid`](exercicios/avoid/avoid.py) | andar pela arena 2 × 2 m sem bater no cilindro central nem nas paredes | **só** o lidar (`/hero_0/laser`, 8 feixes) | `/hero_0/cmd_vel` |
| [`goto`](exercicios/goto/goto.py) | ir de meta em meta (*goal to goal*) | `/hero_0/odom` e `/hero_0/goal` | `/hero_0/cmd_vel` |

O README original do kit (instalação, tópicos, feixes do lidar) está em [`README_KIT.md`](README_KIT.md).

## Estratégias

- **goto:** coordenadas polares da meta no referencial do robô. `rho = hypot(xg − x, yg − y)`,
  `alpha = wrap(atan2(yg − y, xg − x) − θ)`, `w = Kα·α` e `v = Kρ·ρ·max(cos α, 0)`, com `v = 0`
  quando `|α|` passa de um limiar (gira no lugar). Saturação em `v` e `w`.
- **avoid:** Braitenberg por feixe (obstáculo à direita gera giro à esquerda, e vice-versa), velocidade
  reduzida pela folga à frente, lado de giro travado quando há algo de frente, pequeno giro aleatório
  lento (*wander*) para não cair em órbita periódica, e anti-travamento (gira no lugar em sentido
  aleatório se a frente ficar bloqueada). A odometria é lida só para o log e **não entra no controle**.
- Cada nó tem duas configurações de parâmetros, `A` (conservadora) e `B` (agressiva), definidas por
  constantes nomeadas no topo do arquivo. Os valores e as equações completas estão no cabeçalho de
  cada `.py`.

## Estrutura

```
exercicios/avoid/avoid.py     nó de desvio (+ avoid_log_<A|B>.csv e as repetições *_s2.csv, *_s3.csv)
exercicios/goto/goto.py       nó de navegação (+ goto_log_<A|B>.csv e as repetições *_s2.csv, *_s3.csv)
exercicios/launch/*.launch    launch files (ganharam o argumento config:=A|B)
scripts/ensaio.sh             roda um ensaio final com semente fixa
analise/plot_avoid.py         gráficos e métricas do avoid (execução 1)
analise/plot_goto.py          gráficos e métricas do goto (execução 1)
analise/repeticoes.py         média e desvio-padrão entre as 3 execuções
RELATORIO.pdf                 relatório entregue (exportado de RELATORIO.docx)
RELATORIO.docx                relatório em Word, versão editável
analise/diagrama_nos.py       diagrama dos nós e tópicos usado no relatório
videos/                       vídeos dos dois exercícios no simulador
analise/figuras/              PNGs gerados
analise/tabelas/              tabelas de métricas A vs B (.csv e .md)
tests/                        testes offline (sem ROS) da lei de controle e da análise
README_KIT.md                 README original do kit
```

## Requisitos

- **Simulador e nós (Windows):** Windows 10/11 com **WSL2 + Ubuntu** e **Docker Desktop** com a integração
  WSL ativada para o Ubuntu (Settings → Resources → WSL integration). Rode tudo no terminal do Ubuntu,
  com o repositório dentro do sistema de arquivos do Linux (por exemplo `~/Tech-Tabajara`, não `/mnt/c/...`).
  A janela 3D aparece pelo WSLg. No Linux basta o Docker Engine; se a janela não abrir ("No display
  detected"), libere o X para o container com `xhost +local:` (pacote `xorg-xhost`/`x11-xserver-utils`).
- **Análise (fora do container):** Python 3.10 ou mais novo com `pandas`, `numpy` e `matplotlib`:
  `pip install pandas numpy matplotlib`.

## Como rodar

Instalação, uma vez (5 a 15 min):

```bash
git clone https://github.com/LeanHenrique/Tech-Tabajara.git ~/Tech-Tabajara
cd ~/Tech-Tabajara/"Trabalho Prático 5"
./run.sh build
```

### Ensaios finais, com semente fixa (recomendado)

O `./run.sh run` não aceita semente, então as metas do `goto` mudam a cada execução. Para comparar A
e B com a mesma sequência de metas, use o script abaixo. Ele reinicia o simulador (com janela) com
`--seed` (padrão 1) e roda o nó. O nó **para sozinho** (180 s no goto, 300 s no avoid) e fecha o CSV.

```bash
./scripts/ensaio.sh goto A       # 180 s
./scripts/ensaio.sh goto B       # 180 s
./scripts/ensaio.sh avoid A      # 300 s
./scripts/ensaio.sh avoid B      # 300 s
```

Um terceiro argumento muda a semente: `./scripts/ensaio.sh goto A 7`. Além de `goto_log_A.csv`, o script
grava uma cópia com a semente no nome (`goto_log_A_s7.csv`), para ensaios com sementes diferentes não se
sobrescreverem. No repositório, o arquivo **sem sufixo é a execução 1 (semente 1)** e os `_s2` e `_s3` são as
repetições.

**Atenção:** todo ensaio sobrescreve o arquivo sem sufixo (`goto_log_A.csv` etc.), que é o que os gráficos usam.
Para refazer um ensaio sem perder a execução 1 guardada, faça antes uma cópia ou restaure depois com
`git checkout -- exercicios/`. Rodar a semente 1 também cria um `*_s1.csv`, que não é usado pela análise.

**No avoid, a semente do simulador não muda nada** (arena fixa, sem sorteio). As repetições do avoid variam
apenas pelo sorteio do *wander* feito pelo nó, que não tem semente fixa; são execuções independentes.

### Execução comum (sem semente)

```bash
./run.sh run goto config:=A
./run.sh run goto config:=B
./run.sh run avoid config:=A
./run.sh run avoid config:=B
```

Sem `config:=`, vale a configuração `A`. Também vale a variável de ambiente `CONFIG` fora do launch.
Neste modo o `roslaunch` continua vivo depois que o nó termina (a ponte com o simulador não encerra):
quando o terminal mostrar `trial finished`, o CSV já está salvo e dá para parar com Ctrl-C.

## Onde ficam os logs

Na pasta do exercício, ao lado do script (a pasta `exercicios/` é montada dentro do container):

| Arquivo | Colunas |
|---|---|
| `exercicios/goto/goto_log_<A\|B>.csv` | `t, x, y, theta, xg, yg, rho, alpha, v, w, goal_reached` |
| `exercicios/avoid/avoid_log_<A\|B>.csv` | `t, x, y, theta, min_range, v, w, mode` |

`goal_reached = 1` marca a primeira linha com a nova meta, ou seja, a linha seguinte ao atingimento.
`mode` (`cruise`, `avoid`, `escape`) indica o estado do controlador do avoid. Quando o ensaio roda no
WSL, copie os CSVs de volta para o seu repositório se ele estiver em outro lugar.

## Gráficos e métricas

Da raiz do repositório, em qualquer sistema com os requisitos de análise:

```bash
python analise/plot_avoid.py
python analise/plot_goto.py
python analise/repeticoes.py     # média e desvio-padrão entre as 3 execuções de cada config
```

Geram os PNGs em `analise/figuras/` (trajetórias, menor distância × tempo, `rho` e `alpha` × tempo e
as comparações A vs B lado a lado) e as tabelas em `analise/tabelas/` (`.csv` e `.md`). Colisão é
contada por **evento**: cada vez que a menor distância do lidar cruza abaixo de 0,045 m.

## Testes

Rodam sem ROS nem Docker (`rospy` é simulado; o avoid usa o modelo de lidar do próprio simulador
numa malha fechada cinemática):

```bash
python tests/test_goto.py
python tests/test_avoid.py
python tests/test_analise.py     # precisa de pandas e matplotlib
```

## Resultados medidos

Ensaios no simulador (modo CPU, janela aberta), **3 execuções por configuração**. Média ± desvio-padrão
amostral; com n = 3 o desvio é só uma indicação de variabilidade. Valores por execução em
`analise/tabelas/*_repeticoes.md`.

| goto, 180 s (sementes 1, 2, 3) | A | B |
|---|---|---|
| Metas atingidas | 15,7 ± 1,2 | 32,3 ± 0,6 |
| Tempo médio por meta (s) | 11,1 ± 0,9 | 5,5 ± 0,1 |
| Distância percorrida (m) | 16,40 ± 0,12 | 33,07 ± 0,06 |

| avoid, 300 s (3 execuções independentes) | A | B |
|---|---|---|
| Colisões (menor distância < 0,045 m) | 0, 0, 0 | 0, 0, 0 |
| Menor distância lida (m) | 0,193 ± 0,008 | 0,124 ± 0,011 |
| Distância percorrida (m) | 26,98 ± 0,21 | 54,37 ± 0,17 |
| Cobertura da arena (células de 0,25 m, de 64) | 38,7 ± 2,1 | 60,3 ± 1,5 |

O anti-travamento do avoid **não foi acionado** em nenhum ensaio no simulador; ele só foi exercitado
nos testes offline. Tabelas completas em [`analise/tabelas/`](analise/tabelas/).

## Alterações no kit original

- `run.sh`: no WSL com WSLg, monta `/tmp/.X11-unix` para a janela do simulador (o caminho padrão do
  kit não funcionou nesta instalação do Docker Desktop).
- `exercicios/launch/goto.launch` e `avoid.launch`: novo argumento `config` repassado ao nó.

## Vídeos

Gravações de 100 s da janela do simulador, configuração B, semente 1:

- goto: [`videos/goto_B.mp4`](videos/goto_B.mp4) (placar de metas atingidas no topo)
- avoid: [`videos/avoid_B.mp4`](videos/avoid_B.mp4) (tempo de ensaio no topo)

## Créditos e licença

Kit de simulação e modelos 3D do HeRo conforme `LICENSE` e `NOTICE.md` (VeRLab/UFMG, hero_common).
