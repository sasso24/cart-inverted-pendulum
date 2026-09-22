#!/bin/zsh
cd -- "${0:A:h}"
# Preferisci un ambiente locale; sul computer attuale riusa quello già installato.
PYTHON=".venv/bin/python"
[[ -x "$PYTHON" ]] || PYTHON="../cart_pendolo_stepper_rl/.venv/bin/python"
[[ -x "$PYTHON" ]] || PYTHON="../cart_pendolo_rl/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  print "Python non disponibile: esegui prima Setup.command."
  read "?Premi Invio per chiudere."
  exit 1
fi
"$PYTHON" -u valuta.py "$@"
result=$?
if [[ $result -ne 0 ]]; then
  read "?Errore: leggi il messaggio sopra e premi Invio per chiudere."
fi
exit $result
