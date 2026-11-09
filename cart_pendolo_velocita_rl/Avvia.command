#!/bin/zsh
# Doppio clic: menu. Da terminale: ./Avvia.command <script.py> [opzioni], es.
#   ./Avvia.command addestra.py --resume --output modelli_riaddestramento
cd -- "${0:A:h}"
# Preferisci un ambiente locale; sul computer attuale riusa quello già installato.
PYTHON=".venv/bin/python"
[[ -x "$PYTHON" ]] || PYTHON="../cart_pendolo_stepper_rl/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  print "Creo .venv e installo requirements.txt (Python 3.12, serve Internet)..."
  python3.12 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt \
    || { read "?Installazione fallita. Premi Invio per chiudere."; exit 1; }
  PYTHON=".venv/bin/python"
fi
if (( $# == 0 )); then
  print "1) Rete consegnata in simulazione     5) Grafici di una traiettoria"
  print "2) Simulazione manuale (senza rete)   6) Esporta nel firmware STM32"
  print "3) Addestra con config.toml           7) Hardware-in-the-loop con la scheda"
  print "4) Valuta (20 episodi)"
  read "scelta?Scelta: "
  case $scelta in
    1) set -- simula.py ;;
    2) set -- simula.py --manual ;;
    3) set -- addestra.py ;;
    4) set -- simula.py valuta ;;
    5) set -- simula.py grafici ;;
    6) set -- esporta_firmware.py ;;
    7) set -- hil.py ;;
    *) print "Scelta non valida."; exit 1 ;;
  esac
fi
"$PYTHON" -u "$@"
result=$?
if [[ $result -ne 0 ]]; then
  read "?Errore: leggi il messaggio sopra e premi Invio per chiudere."
fi
exit $result
