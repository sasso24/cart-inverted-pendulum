#!/bin/zsh
cd -- "${0:A:h}"
# Hardware-in-the-loop con la STM32 (firmware con HIL_MODE 1). Vedi HIL.md.
PYTHON=".venv/bin/python"
[[ -x "$PYTHON" ]] || PYTHON="../cart_pendolo_stepper_rl/.venv/bin/python"
[[ -x "$PYTHON" ]] || PYTHON="../cart_pendolo_rl/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  print "Python non disponibile: esegui prima Setup.command."
  read "?Premi Invio per chiudere."
  exit 1
fi
# pyserial serve solo per la seriale della scheda: installarlo la prima volta.
if ! "$PYTHON" -c "import serial" 2>/dev/null; then
  print "Installo pyserial (solo la prima volta)..."
  "$PYTHON" -m pip install pyserial==3.5 || { read "?Installazione fallita. Premi Invio."; exit 1; }
fi
"$PYTHON" -u hil.py "$@"
result=$?
if [[ $result -ne 0 ]]; then
  read "?Errore: leggi il messaggio sopra e premi Invio per chiudere."
fi
exit $result
