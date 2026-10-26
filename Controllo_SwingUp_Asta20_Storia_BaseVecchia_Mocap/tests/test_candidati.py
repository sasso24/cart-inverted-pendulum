"""Le reti in Core/Src/reti_candidate.c sono identiche, bit per bit, ai checkpoint indicati.

python3 Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap/tests/test_candidati.py   (ambiente Python del progetto RL)
La rete 0 e quella di X-CUBE-AI/App/network_data_params.c; le altre hanno un array
con lo stesso formato (float32 W0, b0, W1, b1, W2, b2 + riempimento).
"""
from pathlib import Path
import re
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PY = ROOT.parent / 'cart_pendolo_velocita_rl'
sys.path.insert(0, str(PY))
from aggiorna_pesi_xcubeai import leggi_array, pesi_rete  # noqa: E402


def main():
    sorgente = (ROOT / 'Core/Src/reti_candidate.c').read_text()
    nomi = re.findall(r'^ \*   (\d+): (.+)$', sorgente, re.M)
    n = int(re.search(r'#define RETI_CANDIDATE_N (\d+)U',
                      (ROOT / 'Core/Inc/reti_candidate.h').read_text())[1])
    assert len(nomi) == n, (nomi, n)
    tabella = re.search(r'reti_candidate_pesi\[RETI_CANDIDATE_N\] = \{(.*?)\};', sorgente, re.S)[1]
    array = re.findall(r'AI_HANDLE_PTR\((\w+)\)', tabella)
    assert len(array) == n and array[0] == 's_network_weights_array_u64'
    import stima
    from configurazione import configurazione_modello
    guadagni = re.findall(r'\{ ([-0-9.e()f]+), ([-0-9.e()f]+), ([-0-9.e()f]+), ([-0-9.e()f]+) \}',
                          sorgente[sorgente.index('reti_candidate_stima'):])
    assert len(guadagni) == n, guadagni
    for (k, nome), simbolo, g in zip(nomi, array, guadagni):
        cfg = configurazione_modello((PY / nome.replace(' + stimatore offset', '')).parent)
        st = cfg.stimatore
        if nome.endswith(' + stimatore offset'):
            st = stima.guadagni_con_offset(st, cfg.episodio.periodo_controllo_s, -1.,
                                           (st.l_theta, st.l_omega, cfg.episodio.periodo_controllo_s))
            st = {**st, 'l_x': cfg.stimatore.l_x}
        else:
            st = {n_: getattr(st, n_) for n_ in ('l_theta', 'l_omega', 'l_b', 'l_x')}
        valori = [float(v.strip('()').rstrip('f')) for v in g]
        attesi = [st['l_theta'], st['l_omega'], st['l_b'], st['l_x']]
        assert np.allclose(valori, attesi, rtol=1e-6, atol=1e-9), (nome, valori, attesi)
        modello = PY / nome.replace(' + stimatore offset', '')
        if simbolo == 's_network_weights_array_u64':
            testo = (ROOT / 'X-CUBE-AI/App/network_data_params.c').read_text()
        else:
            testo = sorgente.replace(f'static const ai_u64 {simbolo}', 'const ai_u64 s_network_weights_array_u64')
            testo = testo[testo.index('const ai_u64 s_network_weights_array_u64'):]
        _, firmware = leggi_array(testo)
        attesi = pesi_rete(modello)
        assert np.array_equal(firmware[:len(attesi)], attesi) and firmware[-1] == 0, nome
        print(f'  rete {k}: {nome} OK ({len(attesi)} pesi)')
    print(f'OK: {n} reti nel firmware identiche ai checkpoint')


if __name__ == '__main__':
    main()
