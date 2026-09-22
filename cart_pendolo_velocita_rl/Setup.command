#!/bin/zsh
set -e
cd -- "${0:A:h}"
# Ambiente indipendente opzionale. Richiede Python 3.12 e accesso a Internet.
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
print "Setup completato. Avvia Addestra.command."
