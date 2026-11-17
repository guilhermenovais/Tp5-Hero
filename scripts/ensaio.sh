#!/usr/bin/env bash
# Roda UM ensaio final do TP5 com o simulador reiniciado e semente fixa.
#
#   ./scripts/ensaio.sh <goto|avoid> <A|B> [semente]
#
# Por que existe: o ./run.sh run não aceita semente, então as metas do goto mudam a cada
# execução e A e B não seriam comparáveis. Aqui o simulador sobe com --seed (padrão 1), com a
# janela 3D, e o nó do aluno roda pelo próprio run.sh. O nó para sozinho (180 s no goto,
# 300 s no avoid) e grava goto_log_<C>.csv / avoid_log_<C>.csv na pasta do exercício.
#
# Rode no terminal do Ubuntu (WSL2) ou no Linux, com o kit fora de /mnt/c e a imagem já
# construída (./run.sh build).
set -u
CENARIO=${1:?uso: $0 <goto|avoid> <A|B> [semente]}
CONFIG=${2:?uso: $0 <goto|avoid> <A|B> [semente]}
SEMENTE=${3:-1}
case "$CENARIO" in goto|avoid) ;; *) echo "cenário inválido: $CENARIO" >&2; exit 1 ;; esac
case "$CONFIG" in A|B) ;; *) echo "config inválida: $CONFIG" >&2; exit 1 ;; esac

cd "$(dirname "$0")/.." || exit 1
docker image inspect psr-kit-sim:cpu >/dev/null 2>&1 || { echo "imagem do simulador não existe: rode ./run.sh build" >&2; exit 1; }
[ -n "${DISPLAY:-}" ] || { echo "DISPLAY não definido: rode no terminal do Ubuntu (WSL2) ou da sessão gráfica" >&2; exit 1; }

docker rm -f psrkit-sim psrkit-ros >/dev/null 2>&1
docker network inspect psrkit >/dev/null 2>&1 || docker network create psrkit >/dev/null

# Mesmo nome e rótulos que o run.sh espera, para ele reaproveitar este simulador.
# O socket X do WSLg é montado de /tmp/.X11-unix (o caminho padrão do run.sh não funcionou aqui).
ARGS=(-d --name psrkit-sim --network psrkit --label "psr.scenario=$CENARIO" --label psr.robots=1
      -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix -v psrkit-cache-cpu:/root/.cache)
[ -e /dev/dri ] && ARGS+=(--device /dev/dri)
docker run "${ARGS[@]}" psr-kit-sim:cpu \
  python -m psr_sim --viewer --scenario "$CENARIO" --robots 1 --backend cpu --seed "$SEMENTE" >/dev/null

for _ in $(seq 1 120); do
  docker logs psrkit-sim 2>&1 | grep -q "bridge on tcp" && break
  if [ "$(docker inspect -f '{{.State.Running}}' psrkit-sim 2>/dev/null)" != true ]; then
    echo "o simulador fechou:" >&2; docker logs psrkit-sim 2>&1 | tail -n 15 >&2; exit 1
  fi
  sleep 2
done

LOG=$(mktemp)
echo "== ensaio $CENARIO config $CONFIG, semente $SEMENTE, início $(date +%T)"
./run.sh run "$CENARIO" "config:=$CONFIG" > "$LOG" 2>&1 &
PID=$!
# O nó termina sozinho, mas o roslaunch continua vivo (por causa da ponte): remove-se o
# container assim que o nó avisa que terminou.
while kill -0 "$PID" 2>/dev/null; do
  grep -q "finished cleanly" "$LOG" && break
  sleep 3
done
sleep 2
docker rm -f psrkit-ros >/dev/null 2>&1
wait "$PID" 2>/dev/null
grep -E "trial finished|goals reached|saved|ERROR|Traceback" "$LOG" | cut -c1-200
rm -f "$LOG"
# Guarda também uma cópia com a semente no nome, para que ensaios com sementes diferentes não se sobrescrevam.
CSV="exercicios/$CENARIO/${CENARIO}_log_${CONFIG}.csv"
[ -f "$CSV" ] && cp "$CSV" "exercicios/$CENARIO/${CENARIO}_log_${CONFIG}_s${SEMENTE}.csv"
echo "== fim $(date +%T). Log em $CSV (cópia: ${CSV%.csv}_s${SEMENTE}.csv)"
