#!/bin/zsh
cd -- "${0:A:h}"
# Scrive config e pesi della rete nel progetto STM32 (default ../Controllo_SwingUp_Asta20_Storia_BaseVecchia).
# Esempio: ./Esporta_Firmware.command --model modelli_asta20_basevecchia_storia1s_20s/migliore.zip
PYTHON=".venv/bin/python"
[[ -x "$PYTHON" ]] || PYTHON="../cart_pendolo_stepper_rl/.venv/bin/python"
[[ -x "$PYTHON" ]] || PYTHON="../cart_pendolo_rl/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  print "Python non disponibile: esegui prima Setup.command."
  read "?Premi Invio per chiudere."
  exit 1
fi
"$PYTHON" -u esporta_firmware.py "$@"
result=$?
if [[ $result -ne 0 ]]; then
  read "?Errore: leggi il messaggio sopra e premi Invio per chiudere."
fi
exit $result
