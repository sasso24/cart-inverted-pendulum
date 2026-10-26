"""Storia degli ingressi passati per la rete (config.toml [storia]).

La rete riceve i sei ingressi attuali e, per ogni ritardo k di [storia].ritardi,
i sei ingressi di k periodi fa con l'azione decisa allora:

    [obs(t), obs(t-k1), u(t-k1), obs(t-k2), u(t-k2), ...]   6 + 7*len(ritardi)

I sei ingressi sono quelli di sempre (stimati nell'ambiente disturbato);
u e l'azione in [-1, 1] dopo la saturazione, prima di ritardi e rampa.
All'inizio dell'episodio la storia e riempita con i primi ingressi e u = 0,
come nel firmware all'avvio con B1 (Storia_Reset in main.c).
Senza ritardi l'ambiente resta quello storico (sei ingressi).
"""
from collections import deque

from gymnasium import spaces
import numpy as np


class ConStoria:
    """Mixin da mettere prima dell'ambiente: CartPendoloEnv o CartPendoloDisturbatoEnv."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ritardi = self.cfg.storia.ritardi
        self._passato = deque(maxlen=max(self.ritardi))
        self.observation_space = spaces.Box(-np.inf, np.inf, (self.cfg.storia.ingressi,),
                                            dtype=np.float32)

    def osservazione_attuale(self):
        """I sei ingressi attuali, senza storia."""
        return super().observation()

    def observation(self):
        attuale = self.osservazione_attuale()
        if not self._passato:
            return attuale
        # _passato[0] = periodo precedente (ritardo 1).
        return np.concatenate([attuale] + [self._passato[k - 1] for k in self.ritardi])

    def _riempi(self):
        primo = np.append(self.osservazione_attuale(), np.float32(0.)).astype(np.float32)
        self._passato.clear()
        self._passato.extend(primo.copy() for _ in range(self._passato.maxlen))

    def reset(self, *, seed=None, options=None):
        _, info = super().reset(seed=seed, options=options)
        self._riempi()
        return self.observation(), info

    def step(self, action, *, push=0.):
        # Prima del passo: gli ingressi e l'azione di adesso diventano il ritardo 1
        # dell'osservazione restituita da step.
        u = float(np.clip(np.asarray(action, dtype=np.float32).reshape(-1)[0], -1., 1.))
        self._passato.appendleft(np.append(self.osservazione_attuale(), np.float32(u)).astype(np.float32))
        return super().step(action, push=push)


_CLASSI = {}


def con_storia(classe):
    """Sottoclasse di `classe` con la storia (una per classe, riusata)."""
    if classe not in _CLASSI:
        _CLASSI[classe] = type(classe.__name__ + 'ConStoria', (ConStoria, classe), {})
    return _CLASSI[classe]
