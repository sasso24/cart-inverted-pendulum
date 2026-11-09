"""Ambiente di simulazione del pendolo su carrello (MuJoCo + Gymnasium).

Contiene tutto cio che definisce l'ambiente, nello stesso ordine del flusso:
  - configurazione: lettura e validazione di config.toml (Configurazione);
  - motore: conversione uscita rete [-1, 1] -> frequenza STEP firmata e rampa,
    come il firmware (ParametriStepper, ControlloStepper);
  - stimatore dello stato dalle misure rumorose, come stima.c (StimatoreStato);
  - ambiente nominale (CartPendoloEnv) e disturbato (CartPendoloDisturbatoEnv),
    storia degli ingressi passati (ConStoria) e crea_ambiente per sceglierli.

Ogni addestramento salva una copia completa della configurazione in
<cartella>/config.json: simulazione, valutazione ed esportazione usano quella
copia (configurazione_modello), non il config.toml corrente.
"""
from collections import deque
from dataclasses import asdict, dataclass, field, fields, replace
import json
import math
from pathlib import Path
import tomllib

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
CONFIG_DEFAULT = ROOT / 'config.toml'
XML_DEFAULT = 'cart_pendolo_asta20.xml'


def _numeri(obj, nonnegativi=()):
    for f in fields(obj):
        v = getattr(obj, f.name)
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            continue
        if not math.isfinite(v):
            raise ValueError(f'{type(obj).__name__}.{f.name}: serve un numero finito.')
        if f.name in nonnegativi and v < 0:
            raise ValueError(f'{type(obj).__name__}.{f.name}: non puo essere negativo.')


# =============================================================================
#  Motore stepper: uscita della rete -> frequenza STEP firmata, rampa
# =============================================================================
# Non simula bobine, coppia, passi persi o un generatore di impulsi.

@dataclass(frozen=True)
class ParametriStepper:
    frequenza_min_hz: float = 3200.0
    frequenza_max_hz: float = 32000.0
    passi_per_giro: int = 200
    angolo_passo_gradi: float = 1.8
    microstepping: int = 32
    passo_cinghia_m: float = 0.002
    denti_puleggia: int = 48
    rapporto_trasmissione: float = 1.0
    zona_morta: float = 0.0
    rampa_hz_s: float = 1600000.0
    # Inversione di verso: passi della rampa (2 ms) in cui la frequenza resta a
    # zero quando cambierebbe segno, come il firmware (arresto al fronte STEP,
    # cambio DIR, ripartenza da zero). 0 = la rampa attraversa lo zero (storico).
    tick_fermo_inversione: int = 0

    def __post_init__(self):
        for name, value in vars(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name}: serve un numero finito.')
        if not 0 <= self.frequenza_min_hz < self.frequenza_max_hz:
            raise ValueError('Frequenze: serve 0 <= min < max.')
        if not 0 <= self.zona_morta < 1:
            raise ValueError('zona_morta deve essere in [0, 1).')
        if self.rampa_hz_s <= 0:
            raise ValueError('rampa_hz_s deve essere positiva.')
        if type(self.tick_fermo_inversione) is not int or self.tick_fermo_inversione < 0:
            raise ValueError('tick_fermo_inversione: serve un intero >= 0.')
        for name in ('passi_per_giro', 'microstepping', 'denti_puleggia'):
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError(f'{name}: serve un intero positivo.')
        for name in ('angolo_passo_gradi', 'passo_cinghia_m', 'rapporto_trasmissione'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name}: deve essere positivo.')
        if not math.isclose(self.passi_per_giro * self.angolo_passo_gradi, 360., rel_tol=1e-6):
            raise ValueError('passi_per_giro e angolo_passo_gradi sono incoerenti.')

    @property
    def impulsi_per_giro(self):
        return self.passi_per_giro * self.microstepping

    @property
    def metri_per_impulso(self):
        return (self.passo_cinghia_m * self.denti_puleggia
                / (self.impulsi_per_giro * self.rapporto_trasmissione))

    @property
    def velocita_max_m_s(self):
        return self.frequenza_max_hz * self.metri_per_impulso


@dataclass(frozen=True)
class ComandoMotore:
    frequenza_hz: float # firmata: positiva verso +X, negativa verso -X
    velocita_carrello_m_s: float

    @property
    def direzione(self):
        return (self.frequenza_hz > 0) - (self.frequenza_hz < 0)

    @property
    def frequenza_step_hz(self):
        """Modulo non negativo da passare al generatore STEP del firmware."""
        return abs(self.frequenza_hz)


class ControlloStepper:
    def __init__(self, parametri=None):
        self.p = parametri if parametri is not None else ParametriStepper()
        self.reset()

    def reset(self):
        """Azzeramento immediato per reset, finecorsa e fine episodio."""
        self.frequenza_hz = 0.
        self._fermo = 0
        return self.comando(0.)

    def imposta(self, frequenza_hz):
        """Stato della rampa a una frequenza data (partenza con carrello in moto)."""
        limite = self.p.frequenza_max_hz
        self.frequenza_hz = max(-limite, min(limite, float(frequenza_hz)))
        self._fermo = 0
        return ComandoMotore(self.frequenza_hz, self.frequenza_hz * self.p.metri_per_impulso)

    def azione_a_hz(self, azione):
        u = float(azione)
        if not math.isfinite(u):
            raise ValueError('L’azione deve essere finita.')
        u = max(-1., min(1., u))
        if abs(u) <= self.p.zona_morta:
            return 0.
        frazione = (abs(u) - self.p.zona_morta) / (1. - self.p.zona_morta)
        hz = self.p.frequenza_min_hz + frazione * (self.p.frequenza_max_hz - self.p.frequenza_min_hz)
        return math.copysign(hz, u)

    def comando(self, azione):
        """Richiesta istantanea, prima della rampa; non modifica lo stato."""
        hz = self.azione_a_hz(azione)
        return ComandoMotore(hz, hz * self.p.metri_per_impulso)

    def avanza(self, azione, dt):
        """Limita la variazione in Hz/s, anche verso zero e nelle inversioni."""
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt deve essere finito e positivo.')
        richiesta = self.azione_a_hz(azione)
        delta_max = self.p.rampa_hz_s * dt
        delta = richiesta - self.frequenza_hz
        if abs(delta) <= delta_max:
            nuova = richiesta
        else:
            nuova = self.frequenza_hz + math.copysign(delta_max, delta)
        if self.p.tick_fermo_inversione:
            # Stessa logica di Controllo_MotorTick (firmware): un cambio di segno
            # ferma la rampa a zero per tick_fermo_inversione passi.
            if nuova*self.frequenza_hz < 0:
                nuova, self._fermo = 0., self.p.tick_fermo_inversione - 1
            elif self._fermo > 0:
                nuova, self._fermo = 0., self._fermo - 1
        self.frequenza_hz = nuova
        return ComandoMotore(self.frequenza_hz, self.frequenza_hz * self.p.metri_per_impulso)


# =============================================================================
#  Stimatore dello stato: lo stesso algoritmo e sul firmware (stima.c)
# =============================================================================
# Perche serve: derivando misure rumorose a ogni periodo (20 o 10 ms) il rumore esplode.
# Con 7,5 mm di rumore di posizione la velocita derivata ha circa 0,53 m/s
# di rumore (il carrello arriva a 0,48 m/s); con 0,75 gradi sull'angolo la
# velocita angolare derivata ha circa 0,9 rad/s di rumore. La rete vedeva
# quasi solo rumore e non riusciva piu a stabilizzare il pendolo.
#
# Idea: prevedere lo stato con la fisica nota e correggerlo poco con le misure.
#
# - Carrello: la posizione avanza con la velocita applicata dalla rampa
#   (sul banco: il conteggio degli impulsi STEP emessi) e viene corretta
#   lentamente dalla misura di posizione. La velocita e quella applicata.
# - Pendolo, a ogni passo della rampa (2 ms):
#       alfa  = K_G*sin(theta) - K_A*cos(theta)*a_carrello - K_D*omega
#       omega += dt*alfa ;  theta += dt*omega
#   e a ogni periodo della rete, con la lettura dell'encoder:
#       r = wrap(theta_misurato - theta) ; theta += L_theta*r ; omega += L_omega*r
#
# Sul firmware: `predici` nell'interrupt della rampa a 500 Hz, `correggi`
# nel ciclo della rete (50 o 100 Hz), prima di preparare i sei ingressi.
# I guadagni L sono per lettura: cambiando il periodo vanno ricalcolati
# (guadagni_equivalenti) per avere la stessa dinamica dell'errore.
# Il modello e non lineare in theta: vale anche lontano dalla verticale.

@dataclass(frozen=True)
class ParametriStimatore:
    k_g: float = 17.1317      # 1/s^2, m*l*g/J del pendolo
    k_a: float = 1.74635      # rad/m, accoppiamento con l'accelerazione del carrello
    k_d: float = 0.35034      # 1/s, smorzamento del giunto / J
    l_theta: float = 0.2      # correzione dell'angolo per lettura encoder
    l_omega: float = 1.0      # 1/s, correzione della velocita angolare
    l_x: float = 0.05         # correzione della posizione per lettura
    # Offset dell'encoder stimato come terzo stato (0 = stimatore storico a due
    # stati). L'offset e osservabile perche la gravita lega l'accelerazione
    # angolare all'angolo vero: un angolo letto spostato di b produce una
    # caduta prevista che le letture successive non confermano. l_b < 0
    # nell'assegnamento dei poli (guadagni_con_offset); |b| <= b_max_rad.
    # L'offset si aggiorna con peso max(cos(theta), 0): solo nella meta alta.
    l_b: float = 0.0
    b_max_rad: float = 0.2617993877991494   # 15 gradi

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value) or (value < 0 and name != 'l_b'):
                raise ValueError(f'{name}: serve un numero finito non negativo.')
        if not -1 < self.l_b <= 0:
            raise ValueError('Serve -1 < l_b <= 0.')
        if not 0 < self.l_theta < 1 or not 0 <= self.l_x <= 1:
            raise ValueError('Serve 0 < l_theta < 1 e 0 <= l_x <= 1.')


def coefficienti_da_modello(model):
    """K_G, K_A, K_D del modello MuJoCo, per controllare [stimatore] dopo modifiche all'XML."""
    data = mujoco.MjData(model)
    # L'attrito coulombiano (frictionloss) entrerebbe in mj_inverse e falserebbe k_g.
    attrito = model.dof_frictionloss.copy()
    model.dof_frictionloss[:] = 0.

    def forza(theta, qacc):
        mujoco.mj_resetData(model, data)
        data.qpos[:] = [0., theta]
        data.qacc[:] = qacc
        mujoco.mj_inverse(model, data)
        return float(data.qfrc_inverse[1])

    zero = forza(0., [0., 0.])
    inerzia = forza(0., [0., 1.]) - zero
    accoppiamento = forza(0., [1., 0.]) - zero
    theta = 0.3
    gravita = -forza(theta, [0., 0.]) / (inerzia * math.sin(theta))
    model.dof_frictionloss[:] = attrito
    return {'k_g': gravita, 'k_a': accoppiamento / inerzia,
            'k_d': float(model.dof_damping[1]) / inerzia}


class StimatoreStato:
    def __init__(self, parametri=None):
        self.p = parametri if parametri is not None else ParametriStimatore()
        self.reset(0., 0.)

    def reset(self, x, theta, velocita=0., omega=0.):
        """Prima lettura: posizione e angolo misurati, pendolo fermo (omega=0 come
        nel firmware; un valore diverso serve solo alle partenze casuali in addestramento)."""
        self.x, self.theta, self.omega = float(x), float(theta), float(omega)
        self.offset = 0.
        self.velocita = self._velocita_precedente = float(velocita)

    def predici(self, velocita_applicata, dt):
        """Un passo della rampa: velocita del carrello in m/s dopo la rampa."""
        p = self.p
        accelerazione = (velocita_applicata - self._velocita_precedente) / dt
        self._velocita_precedente = velocita_applicata
        alfa = (p.k_g*math.sin(self.theta) - p.k_a*math.cos(self.theta)*accelerazione
                - p.k_d*self.omega)
        self.omega += dt*alfa
        self.theta += dt*self.omega
        self.x += dt*velocita_applicata
        self.velocita = velocita_applicata

    def correggi(self, x_misurata, theta_misurato):
        """Una lettura dei sensori a ogni periodo della rete."""
        p = self.p
        r = math.remainder(theta_misurato - self.theta - self.offset, 2*math.pi)
        if p.l_b:
            # Solo nella meta alta: in basso la gravita cambia segno e con lo
            # stesso guadagno l'offset divergerebbe (in basso il firmware ricalibra).
            peso = max(math.cos(self.theta), 0.)
            self.offset = max(-p.b_max_rad, min(p.b_max_rad, self.offset + p.l_b*peso*r))
        self.theta = math.remainder(self.theta + p.l_theta*r, 2*math.pi)
        self.omega += p.l_omega*r
        self.x += p.l_x*(x_misurata - self.x)


def guadagni_equivalenti(p, periodo_da, periodo_a, dt=0.002):
    """l_theta, l_omega, l_x per il periodo `periodo_a` con la stessa dinamica
    dell'errore (poli in s, linearizzata in alto) dei guadagni p a `periodo_da`."""
    from scipy.optimize import least_squares

    def poli(lt, lo, n):
        a = np.eye(2)
        passo = np.array([[1., dt], [0., 1.]]) @ np.array([[1., 0.], [dt*p.k_g, 1. - dt*p.k_d]])
        for _ in range(n):
            a = passo @ a
        lam = np.linalg.eigvals(np.array([[1. - lt, 0.], [-lo, 1.]]) @ a).astype(complex)
        return np.sort_complex(np.log(lam) / (n*dt))

    n_da, n_a = round(periodo_da/dt), round(periodo_a/dt)
    obiettivo = poli(p.l_theta, p.l_omega, n_da)

    def errore(v):
        d = poli(v[0], v[1], n_a) - obiettivo
        return np.r_[d.real, d.imag]
    scala = periodo_a / periodo_da
    lt, lo = least_squares(errore, [p.l_theta*scala, p.l_omega*scala]).x
    l_x = 1. - (1. - p.l_x)**scala
    return {'l_theta': float(lt), 'l_omega': float(lo), 'l_x': float(l_x)}


def guadagni_con_offset(p, periodo, polo_offset_s=-1.0, riferimento=(0.2, 1.0, 0.02), dt=0.002):
    """l_theta, l_omega, l_b dello stimatore a tre stati (theta, omega, offset).

    Assegnamento dei poli sul modello linearizzato in alto: i due poli
    dell'errore restano quelli dei guadagni `riferimento` (l_theta, l_omega,
    periodo) dello stimatore storico, il terzo (offset) e `polo_offset_s` in 1/s.
    """
    from scipy.signal import place_poles

    def phi(n):
        a = np.eye(2)
        passo = np.array([[1., dt], [0., 1.]]) @ np.array([[1., 0.], [dt*p.k_g, 1. - dt*p.k_d]])
        for _ in range(n):
            a = passo @ a
        return a
    lt, lo, periodo_rif = riferimento
    n_rif = round(periodo_rif/dt)
    lam = np.linalg.eigvals(np.array([[1. - lt, 0.], [-lo, 1.]]) @ phi(n_rif)).astype(complex)
    poli_s = np.log(lam) / (n_rif*dt)
    n = round(periodo/dt)
    f = np.eye(3)
    f[:2, :2] = phi(n)
    h = np.array([[1., 0., 1.]])
    z = np.exp(np.r_[poli_s, polo_offset_s] * n*dt)
    l = place_poles(f.T, (h @ f).T, z).gain_matrix.ravel()
    return {'l_theta': float(l[0]), 'l_omega': float(l[1]), 'l_b': float(l[2])}


# =============================================================================
#  Configurazione (config.toml)
# =============================================================================
# I valori di default delle classi riproducono il comportamento storico
# (solo equilibrio dall'alto, ricompensa del 1 ottobre); config.toml li sovrascrive.

@dataclass(frozen=True)
class ParametriModello:
    file_xml: str = XML_DEFAULT


@dataclass(frozen=True)
class ParametriEpisodio:
    # Swing-up: il pendolo parte appeso (angolo_iniziale_gradi, -180 = in basso).
    # Senza swing-up parte dall'alto (0 gradi) come prima.
    swing_up: bool = False
    angolo_iniziale_gradi: float = -180.0
    dispersione_x_m: float = 0.1
    dispersione_angolo_rad: float = 0.05
    dispersione_omega_rad_s: float = 0.02
    # Solo in addestramento e solo con swing-up: frazione di episodi che
    # partono vicino all'alto, per allenare presto anche l'equilibrio.
    frazione_partenza_alto: float = 0.0
    durata_s: float = 20.0
    # 0 = finecorsa dell'XML (0,462 m). Un valore minore lascia un margine
    # di frenata prima dei finecorsa reali; il firmware usa lo stesso limite.
    limite_corsa_m: float = 0.0
    penalita_finecorsa: float = 5.0
    # 0 = nessun limite al numero di giri completi del pendolo.
    giri_max: float = 0.0
    # Definizione di "in equilibrio" per metriche e successo.
    stabile_angolo_gradi: float = 12.0
    stabile_omega_rad_s: float = 1.0
    stabile_velocita_m_s: float = 0.75
    # Successo: in equilibrio senza interruzioni dal piu tardi a questo istante
    # fino alla fine (0 = per tutto l'episodio, come prima).
    tempo_max_salita_s: float = 0.0
    # Normalizzazione della velocita angolare nel 5o ingresso (uguale nel firmware).
    scala_omega_rad_s: float = 10.0
    # Periodo della rete (una decisione ogni periodo). Deve essere un multiplo del
    # passo fisico dell'XML (2 ms); 0.02 = comportamento storico (50 Hz).
    periodo_controllo_s: float = 0.02
    # Solo addestramento: quota di episodi che parte da uno stato casuale su tutto
    # il giro (angolo uniforme, omega, x e velocita del carrello entro questi limiti),
    # per allenare l'aggancio in alto da arrivi diversi da quelli del proprio swing-up.
    # Si somma a frazione_partenza_alto (se la somma supera 1 resta la quota rimanente).
    frazione_partenza_casuale: float = 0.0
    casuale_x_m: float = 0.25
    casuale_velocita_m_s: float = 0.3
    casuale_omega_rad_s: float = 4.0

    def __post_init__(self):
        _numeri(self, {f.name for f in fields(self)} - {'angolo_iniziale_gradi'})
        if not 0 <= self.frazione_partenza_alto <= 1:
            raise ValueError('frazione_partenza_alto deve essere in [0, 1].')
        if not 0 <= self.frazione_partenza_casuale <= 1:
            raise ValueError('frazione_partenza_casuale deve essere in [0, 1].')
        if self.periodo_controllo_s <= 0:
            raise ValueError('periodo_controllo_s deve essere positivo.')
        if self.durata_s <= 0 or self.scala_omega_rad_s <= 0:
            raise ValueError('durata_s e scala_omega_rad_s devono essere positive.')
        if self.tempo_max_salita_s >= self.durata_s:
            raise ValueError('tempo_max_salita_s deve essere minore di durata_s.')


@dataclass(frozen=True)
class ParametriRicompensa:
    """r = alto^esponente_alto * F_x * F_omega * F_angolo * F_v
           + peso_energia * exp(-((E-1)/scala_energia)^2)
           - G * (peso_azione*u^2 + peso_variazione*(u-u_prec)^2
                  + peso_inversione*[cambio di verso])
           - peso_bordo*(ingresso nella fascia)^2

    G = ((1+cos(angolo))/2)^esponente_penalita_alto: con 0 le penalita sul comando
    valgono sempre; con 2 valgono solo vicino all'alto e lo swing-up resta libero.

    alto = (1+cos(angolo))/2 ; ogni F = minimo + (1-minimo)*exp(-(valore/scala)^2):
    minimo = 1 spegne il termine, minimo vicino a 0 lo rende decisivo.
    E = cos(angolo) + omega^2/(2*k_g) vale 1 col pendolo fermo in alto.

    Con periodo_controllo_s diverso da 0.02 la ricompensa di ogni passo (non la
    penalita di finecorsa, che e una tantum) e moltiplicata per periodo/0.02:
    ricompensa al secondo e scala dei valori restano quelle dei 50 Hz.
    """
    esponente_alto: float = 1.0
    minimo_x: float = 0.6
    scala_x_m: float = 0.15
    minimo_omega: float = 0.5
    scala_omega_rad_s: float = math.sqrt(10.)
    minimo_angolo: float = 1.0
    scala_angolo_gradi: float = 10.0
    minimo_velocita: float = 1.0
    scala_velocita_m_s: float = 0.2
    peso_energia: float = 0.0
    scala_energia: float = 0.3
    peso_azione: float = 0.005
    peso_variazione: float = 0.02
    peso_inversione: float = 0.0
    esponente_penalita_alto: float = 0.0
    peso_bordo: float = 0.0
    inizio_bordo_m: float = 0.30

    def __post_init__(self):
        _numeri(self, {f.name for f in fields(self)})
        for nome in ('minimo_x', 'minimo_omega', 'minimo_angolo', 'minimo_velocita'):
            if not 0 <= getattr(self, nome) <= 1:
                raise ValueError(f'{nome} deve essere in [0, 1].')
        for nome in ('scala_x_m', 'scala_omega_rad_s', 'scala_angolo_gradi',
                     'scala_velocita_m_s', 'scala_energia'):
            if getattr(self, nome) <= 0:
                raise ValueError(f'{nome} deve essere positiva.')


@dataclass(frozen=True)
class ParametriDisturbi:
    """Stress riproducibile sul modello nominale, senza cambiarne XML o firmware.

    I modi sono oscillatori equivalenti di forza/coppia, non una simulazione FEM
    dell'asta o un modello elettromeccanico dello stepper.
    """
    intensita: float = 0.35
    massa_rel: float = 0.10
    smorzamento_rel: float = 0.25
    guadagno_motore_rel: float = 0.15
    ritardo_max_s: float = 0.006
    frequenze_min_hz: tuple = (8., 20., 3.)
    frequenze_max_hz: tuple = (18., 40., 10.)
    zeta_min: float = 0.08
    zeta_max: float = 0.25
    forza_cinghia_n: float = 0.20
    forza_motore_n: float = 0.10
    coppia_asta_nm: float = 0.003
    accelerazione_scala_m_s2: float = 10.
    eccitazione_casuale: float = 0.25
    correlazione_s: float = 0.04
    # Deviazione standard del rumore, estratta per episodio in [min, max].
    # NON scalata da intensita: sono i valori richiesti per i sensori.
    rumore_posizione_m: tuple = (0.005, 0.010)
    rumore_angolo_gradi: tuple = (0.5, 1.0)
    offset_angolo_rad: float = math.radians(0.5)
    encoder_conteggi_giro: int = 2400
    # Perno del pendolo (24/10): valori ASSOLUTI per episodio, NON scalati da intensita.
    # Con smorzamento_perno_max_nms > 0 lo smorzamento viscoso del perno e
    # log-uniforme in [min, max] N*m*s/rad al posto del damping dell'XML (x smorzamento_rel);
    # attrito coulombiano (frictionloss) uniforme in [0, attrito_perno_max_nm];
    # resistenza dell'aria -c*omega*|omega| con c uniforme in [0, resistenza_aria_max_nms2].
    # Tutto a 0 = comportamento e sequenza casuale di prima.
    smorzamento_perno_min_nms: float = 0.
    smorzamento_perno_max_nms: float = 0.
    attrito_perno_max_nm: float = 0.
    resistenza_aria_max_nms2: float = 0.

    def __post_init__(self):
        for name, value in vars(self).items():
            if name.startswith('frequenze_') or name.startswith('rumore_'):
                n = 3 if name.startswith('frequenze_') else 2
                if not isinstance(value, (list, tuple)) or len(value) != n:
                    raise ValueError(f'{name}: servono {n} valori.')
                object.__setattr__(self, name, tuple(value))
                values = value
            else:
                values = (value,)
            if any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) or v < 0 for v in values):
                raise ValueError(f'{name}: servono numeri finiti non negativi.')
        if not 0 <= self.intensita <= 1:
            raise ValueError('intensita deve essere in [0, 1].')
        if any(getattr(self, n) >= 1 for n in
               ('massa_rel', 'smorzamento_rel', 'guadagno_motore_rel')):
            raise ValueError('Le variazioni relative devono essere inferiori a 1.')
        if not 0 < self.zeta_min <= self.zeta_max < 1:
            raise ValueError('Serve 0 < zeta_min <= zeta_max < 1.')
        if any(not 0 < lo <= hi for lo, hi in
               zip(self.frequenze_min_hz, self.frequenze_max_hz)):
            raise ValueError('Intervalli di frequenza non validi.')
        if any(lo > hi for lo, hi in (self.rumore_posizione_m, self.rumore_angolo_gradi)):
            raise ValueError('Rumore: serve [minimo, massimo] con minimo <= massimo.')
        if self.correlazione_s <= 0 or self.accelerazione_scala_m_s2 <= 0:
            raise ValueError('Correlazione e scala accelerazione devono essere positive.')
        if type(self.encoder_conteggi_giro) is not int or self.encoder_conteggi_giro < 1:
            raise ValueError('encoder_conteggi_giro deve essere un intero positivo.')
        if self.ritardo_max_s > 0.1:
            raise ValueError('ritardo_max_s deve essere <= 0.1 s.')
        if self.smorzamento_perno_max_nms and \
                not 0 < self.smorzamento_perno_min_nms <= self.smorzamento_perno_max_nms:
            raise ValueError('Smorzamento del perno: serve 0 < minimo <= massimo.')

    @classmethod
    def carica(cls, path):
        """Sezione [disturbi] di un TOML (anche un config.toml completo)."""
        with Path(path).open('rb') as stream:
            config = tomllib.load(stream)
        if 'disturbi' not in config:
            raise ValueError(f'{path}: manca la sezione [disturbi].')
        valori = dict(config['disturbi'])
        valori.pop('attivi', None)
        return cls(**valori)


@dataclass(frozen=True)
class ParametriSensori:
    """Guasti dei sensori, solo nell'ambiente disturbato (stima + rumore).

    Encoder: offset iniziale, conteggi persi durante il moto (l'errore si
    accumula come una deriva), impulsi spuri/salti da disturbi elettrici.
    Carrello: passi persi (la posizione contata dagli STEP deriva da quella vera).
    Nessuno di questi valori e scalato da [disturbi].intensita.
    """
    offset_max_gradi: float = 0.0
    prob_conteggio_perso_max: float = 0.0
    salti_al_s: float = 0.0
    salto_max_conteggi: int = 1
    passi_persi_max: float = 0.0

    def __post_init__(self):
        _numeri(self, {f.name for f in fields(self)})
        if not 0 <= self.prob_conteggio_perso_max < 1 or not 0 <= self.passi_persi_max < 1:
            raise ValueError('Le probabilita/frazioni devono essere in [0, 1).')
        if type(self.salto_max_conteggi) is not int or self.salto_max_conteggi < 1:
            raise ValueError('salto_max_conteggi deve essere un intero >= 1.')

    @property
    def attivi(self):
        return any((self.offset_max_gradi, self.prob_conteggio_perso_max,
                    self.salti_al_s, self.passi_persi_max))


@dataclass(frozen=True)
class ParametriAddestramento:
    cartella: str = 'modelli'
    passi: int = 450000
    seed: int = 42
    valuta_ogni: int = 10000
    episodi_selezione: int = 5
    usa_disturbi: bool = True
    # 6 ingressi -> strati nascosti ReLU -> 1 uscita tanh. Con [128, 128] i pesi
    # si copiano nel progetto X-CUBE-AI esistente; altre taglie richiedono di
    # reimportare l'ONNX in STM32CubeMX.
    strati: tuple = (128, 128)
    learning_rate: float = 3e-4
    buffer: int = 300000
    learning_starts: int = 3000
    batch: int = 128
    gamma: float = 0.99
    tau: float = 0.005
    train_freq: int = 4
    gradient_steps: int = 1
    ent_coef: str = 'auto_0.1'

    def __post_init__(self):
        object.__setattr__(self, 'strati', tuple(self.strati))
        if not self.strati or any(type(n) is not int or n <= 0 for n in self.strati):
            raise ValueError('strati: servono interi positivi.')
        for nome in ('passi', 'valuta_ogni', 'episodi_selezione', 'buffer', 'batch',
                     'train_freq', 'gradient_steps'):
            if type(getattr(self, nome)) is not int or getattr(self, nome) <= 0:
                raise ValueError(f'{nome}: serve un intero positivo.')
        if not 0 < self.gamma < 1:
            raise ValueError('gamma deve essere in (0, 1).')


@dataclass(frozen=True)
class ParametriFirmware:
    # Solo per esporta_firmware.py: vanno nell'header del progetto STM32.
    nome_progetto: str = 'Controllo_SwingUp'
    # B1 con il pendolo fermo e l'angolo letto entro angolo_ricalibrazione_gradi
    # da -180: l'angolo torna esattamente a -180 gradi, poi swing-up.
    ricalibra_theta_all_avvio: bool = True
    angolo_ricalibrazione_gradi: float = 20.0
    tempo_quiete_ms: int = 1000
    # B1 con |angolo| entro questa soglia da 0: partenza dall'alto (solo equilibrio),
    # senza ricalibrazione. Vale anche con swing-up.
    angolo_max_avvio_gradi: float = 10.0
    # Senza swing-up: arresto se durante il controllo |angolo| la supera.
    angolo_max_controllo_gradi: float = 45.0
    # Lettura encoder scartata come guasto se in un periodo implica piu di questo.
    omega_max_plausibile_rad_s: float = 25.0


@dataclass(frozen=True)
class ParametriStoria:
    """Storia degli ingressi passati (ConStoria), anche nel firmware.

    ritardi: periodi di controllo nel passato (1 = il periodo precedente).
    Per ogni ritardo k la rete riceve i sei ingressi di k periodi fa e l'azione
    decisa allora, dopo i sei ingressi attuali: 6 + 7*len(ritardi) ingressi.
    Vuoto = solo i sei ingressi attuali (reti fino al 21/10).
    """
    ritardi: tuple = ()

    def __post_init__(self):
        object.__setattr__(self, 'ritardi', tuple(self.ritardi))
        if any(type(k) is not int or k < 1 for k in self.ritardi):
            raise ValueError('[storia] ritardi: servono interi >= 1.')
        if list(self.ritardi) != sorted(set(self.ritardi)):
            raise ValueError('[storia] ritardi: crescenti e senza ripetizioni.')

    @property
    def ingressi(self):
        return 6 + 7*len(self.ritardi)


SEZIONI = {'modello': ParametriModello, 'episodio': ParametriEpisodio,
           'ricompensa': ParametriRicompensa, 'sensori': ParametriSensori,
           'addestramento': ParametriAddestramento, 'firmware': ParametriFirmware,
           'stepper': ParametriStepper, 'stimatore': ParametriStimatore,
           'storia': ParametriStoria}


@dataclass(frozen=True)
class Configurazione:
    modello: ParametriModello = field(default_factory=ParametriModello)
    episodio: ParametriEpisodio = field(default_factory=ParametriEpisodio)
    ricompensa: ParametriRicompensa = field(default_factory=ParametriRicompensa)
    sensori: ParametriSensori = field(default_factory=ParametriSensori)
    addestramento: ParametriAddestramento = field(default_factory=ParametriAddestramento)
    firmware: ParametriFirmware = field(default_factory=ParametriFirmware)
    stepper: ParametriStepper = field(default_factory=ParametriStepper)
    stimatore: ParametriStimatore = field(default_factory=ParametriStimatore)
    storia: ParametriStoria = field(default_factory=ParametriStoria)
    disturbi: object = None   # ParametriDisturbi oppure None (ambiente nominale)

    @classmethod
    def da_dict(cls, dati):
        dati = dict(dati)
        sconosciute = set(dati) - set(SEZIONI) - {'disturbi'}
        if sconosciute:
            raise ValueError(f'Sezioni sconosciute in configurazione: {sorted(sconosciute)}')
        argomenti = {}
        for nome, classe in SEZIONI.items():
            if nome in dati:
                valori = dict(dati[nome])
                validi = {f.name for f in fields(classe)}
                extra = set(valori) - validi
                if extra:
                    raise ValueError(f'[{nome}] chiavi sconosciute: {sorted(extra)}')
                argomenti[nome] = classe(**valori)
        disturbi = dati.get('disturbi')
        if disturbi is not None:
            disturbi = dict(disturbi)
            attivi = disturbi.pop('attivi', True)
            disturbi = ParametriDisturbi(**disturbi) if attivi else None
        return cls(**argomenti, disturbi=disturbi)

    @classmethod
    def carica(cls, percorso=CONFIG_DEFAULT):
        with Path(percorso).open('rb') as stream:
            return cls.da_dict(tomllib.load(stream))

    def a_dict(self):
        """Dizionario JSON completo (salvato in config.json di ogni modello)."""
        risultato = {nome: asdict(getattr(self, nome)) for nome in SEZIONI}
        risultato['disturbi'] = None if self.disturbi is None else asdict(self.disturbi)
        return json.loads(json.dumps(risultato))

    @property
    def disturbi_addestramento(self):
        return self.disturbi if self.addestramento.usa_disturbi else None


def configurazione_modello(cartella):
    """Configurazione salvata con un modello (<cartella>/config.json)."""
    config = json.loads((Path(cartella) / 'config.json').read_text())
    if 'configurazione' not in config:
        raise ValueError(f'{cartella}: checkpoint senza configurazione completa '
                         '(reti precedenti al 10/10: ramo archivio-completo).')
    return Configurazione.da_dict(config['configurazione'])


def con_offset(cfg, polo):
    """Stimatore a tre stati (offset dell'encoder, polo in 1/s) per una rete
    addestrata senza; invariata se polo e None o se la rete lo ha gia."""
    if polo is None or cfg.stimatore.l_b:
        return cfg
    st, periodo = cfg.stimatore, cfg.episodio.periodo_controllo_s
    g = guadagni_con_offset(st, periodo, polo, (st.l_theta, st.l_omega, periodo))
    return replace(cfg, stimatore=replace(st, **g))


# =============================================================================
#  Ambiente nominale: azione di velocita, comando motore in Hz firmati
# =============================================================================

def _fattore(minimo, valore, scala):
    if minimo == 1.:
        return 1.
    return minimo + (1. - minimo)*math.exp(-(valore/scala)**2)


def ricompensa(p, x, angle, omega, vx, u, u_precedente, *, k_g=17.1317, limite=0.462):
    """Ricompensa di config.toml [ricompensa]; formula in ParametriRicompensa."""
    angolo = math.remainder(angle, 2*math.pi)
    alto = ((math.cos(angle) + 1.) / 2.)**p.esponente_alto
    r = (alto * _fattore(p.minimo_x, x, p.scala_x_m)
         * _fattore(p.minimo_omega, omega, p.scala_omega_rad_s)
         * _fattore(p.minimo_angolo, math.degrees(angolo), p.scala_angolo_gradi)
         * _fattore(p.minimo_velocita, vx, p.scala_velocita_m_s))
    if p.peso_energia:
        energia = math.cos(angle) + omega*omega/(2.*k_g)
        r += p.peso_energia*math.exp(-((energia - 1.)/p.scala_energia)**2)
    g = ((math.cos(angle) + 1.) / 2.)**p.esponente_penalita_alto if p.esponente_penalita_alto else 1.
    r -= g*(p.peso_azione*u*u + p.peso_variazione*(u - u_precedente)**2)
    if p.peso_inversione and u*u_precedente < 0:
        r -= g*p.peso_inversione
    if p.peso_bordo and abs(x) > p.inizio_bordo_m:
        fascia = max(limite - p.inizio_bordo_m, 1e-6)
        r -= p.peso_bordo*((abs(x) - p.inizio_bordo_m)/fascia)**2
    return r


class CartPendoloEnv(gym.Env):
    metadata = {'render_modes': []}
    frame_skip = 10
    max_steps = 1000

    def __init__(self, configurazione=None, *, addestramento=False):
        super().__init__()
        self.cfg = configurazione if configurazione is not None else Configurazione()
        self.ep = self.cfg.episodio
        self.rp = self.cfg.ricompensa
        self.k_g = self.cfg.stimatore.k_g
        # In addestramento (solo swing-up) una parte degli episodi parte in alto.
        self.addestramento = addestramento
        self.controllo = ControlloStepper(self.cfg.stepper)
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / self.cfg.modello.file_xml))
        self.data = mujoco.MjData(self.model)
        # limite: scala della posizione nella rete (finecorsa XML), invariata.
        # limite_corsa: arresto dell'episodio, uguale o piu interno.
        self.limite = float(self.model.joint('scorrimento').range[1])
        self.limite_corsa = self.ep.limite_corsa_m or self.limite
        if self.limite_corsa > self.limite:
            raise ValueError('limite_corsa_m oltre il finecorsa dell XML.')
        # Periodo della rete: multiplo intero del passo fisico (storico 10 x 2 ms).
        passi = self.ep.periodo_controllo_s / self.model.opt.timestep
        if abs(passi - round(passi)) > 1e-6 or round(passi) < 1:
            raise ValueError('periodo_controllo_s deve essere un multiplo del passo dell XML.')
        if round(passi) != type(self).frame_skip:
            self.frame_skip = int(round(passi))
        # Ricompensa per passo riscalata sul periodo storico di 20 ms (1 se invariato).
        self.scala_ricompensa = self.dt / 0.02 if self.frame_skip != 10 else 1.
        if self.ep.durata_s != 20.0 or self.frame_skip != 10:
            self.max_steps = int(round(self.ep.durata_s / self.dt))
        self.action_space = spaces.Box(-1., 1., (1,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (6,), dtype=np.float32)
        self.steps = self.stable_steps = self.best_stable_steps = 0
        self.u_precedente = 0.
        self.motivo = ''
        self.comando_motore = self.controllo.comando(0.)
        self.frequenza_richiesta_hz = 0.
        self.angolo_iniziale = 0.
        self.partenza_casuale = False
        self._episode_done = True

    @property
    def dt(self):
        return self.model.opt.timestep * self.frame_skip

    def observation(self):
        # Sensori ideali dello stato fisico; nessun conteggio STEP fittizio.
        x, angle = self.data.qpos
        vx, omega = self.data.qvel
        return np.array([x / self.limite, math.sin(angle), math.cos(angle),
                         vx / self.controllo.p.velocita_max_m_s, omega / self.ep.scala_omega_rad_s,
                         self.comando_motore.frequenza_hz / self.controllo.p.frequenza_max_hz],
                        dtype=np.float32)

    def info(self):
        angle = math.atan2(math.sin(self.data.qpos[1]), math.cos(self.data.qpos[1]))
        c = self.comando_motore
        return {'motor_frequency_hz': c.frequenza_hz,
                'step_frequency_hz': c.frequenza_step_hz,
                'direction': c.direzione,
                'requested_frequency_hz': self.frequenza_richiesta_hz,
                'command_velocity_m_s': c.velocita_carrello_m_s,
                'cart_velocity_m_s': float(self.data.qvel[0]),
                'cart_x': float(self.data.qpos[0]), 'angle_deg': math.degrees(angle),
                'termination_reason': self.motivo,
                'left_limit': bool(self.data.qpos[0] <= -self.limite_corsa),
                'right_limit': bool(self.data.qpos[0] >= self.limite_corsa),
                'stable_seconds': self.best_stable_steps * self.dt,
                'final_stable_seconds': self.stable_steps * self.dt,
                'is_success': bool(self.best_stable_steps * self.dt >= 3.)}

    def successo(self, terminated):
        """Equilibrio ininterrotto da tempo_max_salita_s (0 = sempre) fino alla fine."""
        richiesti = self.steps - int(round(self.ep.tempo_max_salita_s / self.dt))
        return bool(not terminated and self.steps >= self.max_steps
                    and self.stable_steps >= max(richiesti, 1))

    def _stop(self):
        self.comando_motore = self.controllo.reset()
        self.data.ctrl[0] = 0.

    def _stato_iniziale(self, exact):
        """(x, angolo, omega, velocita del carrello) all'inizio dell'episodio."""
        ep, rng = self.ep, self.np_random
        centro = 0.
        self.partenza_casuale = False
        if ep.swing_up:
            centro = math.radians(ep.angolo_iniziale_gradi)
            if self.addestramento and ep.frazione_partenza_alto > 0 \
                    and rng.random() < ep.frazione_partenza_alto:
                centro = 0.
            # Estrazione separata: con frazione_partenza_casuale = 0 la sequenza
            # casuale resta quella storica.
            elif (self.addestramento and not exact and ep.frazione_partenza_casuale > 0
                  and rng.random() < ep.frazione_partenza_casuale / max(1. - ep.frazione_partenza_alto, 1e-9)):
                self.partenza_casuale = True
                x = rng.uniform(-ep.casuale_x_m, ep.casuale_x_m)
                angolo = rng.uniform(-math.pi, math.pi)
                omega = rng.uniform(-ep.casuale_omega_rad_s, ep.casuale_omega_rad_s)
                v = rng.uniform(-ep.casuale_velocita_m_s, ep.casuale_velocita_m_s)
                return x, angolo, omega, v
        x = 0. if exact else rng.uniform(-ep.dispersione_x_m, ep.dispersione_x_m)
        angolo = centro + (0. if exact else rng.uniform(-ep.dispersione_angolo_rad,
                                                         ep.dispersione_angolo_rad))
        omega = 0. if exact else rng.uniform(-ep.dispersione_omega_rad_s, ep.dispersione_omega_rad_s)
        return x, angolo, omega, 0.

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        exact = (options or {}).get('exact', False)
        x, angolo, omega, v = self._stato_iniziale(exact)
        self.data.qpos[:] = [x, angolo]
        self.data.qvel[1] = omega
        self.angolo_iniziale = angolo
        self.steps = self.stable_steps = self.best_stable_steps = 0
        self.u_precedente = 0.
        self.motivo = ''
        self.frequenza_richiesta_hz = 0.
        self._stop()
        if v:
            # Carrello gia in moto: rampa e servo di velocita partono da v.
            self.comando_motore = self.controllo.imposta(v / self.controllo.p.metri_per_impulso)
            self.data.qvel[0] = self.data.ctrl[0] = self.comando_motore.velocita_carrello_m_s
        self._episode_done = False
        mujoco.mj_forward(self.model, self.data)
        return self.observation(), self.info()

    def _fuori_corsa(self):
        return abs(self.data.qpos[0]) >= self.limite_corsa

    def _ricompensa(self, u):
        x, angle = self.data.qpos
        vx, omega = self.data.qvel
        return ricompensa(self.rp, x, angle, omega, vx, u, self.u_precedente,
                          k_g=self.k_g, limite=self.limite_corsa)

    def _fine_passo(self, u):
        """Parte comune a nominale e disturbato dopo l'integrazione."""
        self.steps += 1
        if (not self.motivo and self.ep.giri_max
                and abs(self.data.qpos[1] - self.angolo_iniziale) > 2*math.pi*self.ep.giri_max):
            self.motivo = 'troppi giri'
        terminated = bool(self.motivo)
        truncated = self.steps >= self.max_steps
        self._episode_done = terminated or truncated
        if self._episode_done:
            self._stop()
        x, angle = self.data.qpos
        vx, omega = self.data.qvel
        reward = self._ricompensa(u) * self.scala_ricompensa
        self.u_precedente = u
        ep = self.ep
        stable = (math.cos(angle) > math.cos(math.radians(ep.stabile_angolo_gradi))
                  and abs(x) < self.limite_corsa and abs(omega) < ep.stabile_omega_rad_s
                  and abs(vx) < ep.stabile_velocita_m_s and not terminated)
        self.stable_steps = self.stable_steps + 1 if stable else 0
        self.best_stable_steps = max(self.best_stable_steps, self.stable_steps)
        return float(reward - ep.penalita_finecorsa*terminated), terminated, truncated

    def step(self, action, *, push=0.):
        if self._episode_done:
            raise RuntimeError('Chiamare reset() prima di iniziare un nuovo episodio.')
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (1,) or not np.isfinite(action).all() or not math.isfinite(push):
            raise ValueError('Serve un’azione finita di forma (1,) e una spinta finita.')
        u = float(np.clip(action[0], -1., 1.))
        self.frequenza_richiesta_hz = self.controllo.azione_a_hz(u)
        self.data.qfrc_applied[0] = push
        for _ in range(self.frame_skip):
            if self._fuori_corsa():
                self.motivo = 'finecorsa'
                break
            self.comando_motore = self.controllo.avanza(u, self.model.opt.timestep)
            self.data.ctrl[0] = self.comando_motore.velocita_carrello_m_s
            mujoco.mj_step(self.model, self.data)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                self._stop()
                self._episode_done = True
                raise FloatingPointError('Stato fisico non finito.')
            if self._fuori_corsa():
                self.motivo = 'finecorsa'
                break
        reward, terminated, truncated = self._fine_passo(u)
        return self.observation(), reward, terminated, truncated, self.info()


# =============================================================================
#  Ambiente disturbato: vibrazioni, parametri variabili, rumore, guasti, stimatore
# =============================================================================

class ModiSmorzati:
    """y'' + 2*zeta*w*y' + w²*y = w²*ingresso, soluzione esatta per ZOH.

    y e adimensionale; la forza/coppia viene scalata e limitata separatamente.
    La discretizzazione esatta evita instabilita numerica dei modi veloci.
    """
    def __init__(self, frequenze, zeta, dt):
        w = 2 * np.pi * np.asarray(frequenze)
        a = np.asarray(zeta) * w
        wd = np.sqrt(w*w - a*a)
        decay, c, s = np.exp(-a*dt), np.cos(wd*dt), np.sin(wd*dt)/wd
        self.e00, self.e01 = decay*(c+a*s), decay*s
        self.e10, self.e11 = -decay*w*w*s, decay*(c-a*s)
        self.y = np.zeros_like(w)
        self.v = np.zeros_like(w)

    def step(self, ingresso):
        delta = self.y - ingresso
        y = ingresso + self.e00*delta + self.e01*self.v
        self.v = self.e10*delta + self.e11*self.v
        self.y = y
        return y


class CartPendoloDisturbatoEnv(CartPendoloEnv):
    def __init__(self, disturbi, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.disturbi = disturbi
        self.sensori = self.cfg.sensori
        if max(disturbi.frequenze_max_hz) * self.model.opt.timestep > 0.1:
            raise ValueError('Servono almeno 10 passi fisici per periodo dei modi.')
        self._nominale = {n: getattr(self.model, n).copy() for n in
                          ('body_mass', 'body_inertia', 'dof_damping', 'dof_frictionloss',
                           'actuator_gainprm', 'actuator_biasprm')}
        self._obs = None
        self.stimatore = StimatoreStato(self.cfg.stimatore)
        self.parametri_episodio = {}
        self.forza_disturbo_n = self.coppia_disturbo_nm = 0.
        self.resistenza_aria = 0.

    def observation(self):
        return super().observation() if self._obs is None else self._obs.copy()

    def info(self):
        return {**super().info(), 'disturbance_force_n': self.forza_disturbo_n,
                'disturbance_torque_nm': self.coppia_disturbo_nm,
                'disturbance_parameters': self.parametri_episodio.copy()}

    def _stop(self):
        super()._stop()
        self.data.qfrc_applied[:] = 0.
        if hasattr(self, '_azioni'):
            self._azioni.clear()

    def reset(self, *, seed=None, options=None):
        # Il reset nominale estrae prima lo stato iniziale: a parita di seed
        # il confronto nominale/disturbato parte dalle stesse qpos e qvel.
        for name, values in self._nominale.items():
            getattr(self.model, name)[:] = values
        mujoco.mj_setConst(self.model, self.data)
        self._obs = None
        super().reset(seed=seed, options=options)
        p, rng, s = self.disturbi, self.np_random, self.disturbi.intensita
        scales = rng.uniform(1-s*p.massa_rel, 1+s*p.massa_rel, 2)
        for name, scale in zip(('carrello', 'pendolo'), scales):
            bid = self.model.body(name).id
            self.model.body_mass[bid] *= scale
            self.model.body_inertia[bid] *= scale
        damping = rng.uniform(1-s*p.smorzamento_rel, 1+s*p.smorzamento_rel, 2)
        self.model.dof_damping[:] *= damping
        perno = self._perno()
        gain =rng.uniform(1-s*p.guadagno_motore_rel, 1+s*p.guadagno_motore_rel)
        self.model.actuator_gainprm[0, 0] *= gain
        self.model.actuator_biasprm[0, 2] *= gain
        # mj_setConst puo usare qpos0: preservare lo stato estratto dal reset.
        qpos, qvel = self.data.qpos.copy(), self.data.qvel.copy()
        mujoco.mj_setConst(self.model, self.data)
        self.data.qpos[:], self.data.qvel[:] = qpos, qvel
        mujoco.mj_forward(self.model, self.data)
        freq = rng.uniform(p.frequenze_min_hz, p.frequenze_max_hz)
        zeta = rng.uniform(p.zeta_min, p.zeta_max, 3)
        self.modi = ModiSmorzati(freq, zeta, self.model.opt.timestep)
        self._ampiezze = s*rng.uniform(0.5, 1., 3)*np.array(
            [p.forza_cinghia_n, p.forza_motore_n, p.coppia_asta_nm])
        delay_max = int(math.floor(s*p.ritardo_max_s / self.model.opt.timestep + 1e-9))
        delay = int(rng.integers(delay_max+1))
        self._azioni = deque([0.]*delay)
        self._offset = rng.uniform(-s*p.offset_angolo_rad, s*p.offset_angolo_rad)
        self._sigma_x = rng.uniform(*p.rumore_posizione_m)
        self._sigma_theta = math.radians(rng.uniform(*p.rumore_angolo_gradi))
        self._colore = np.zeros(3)
        self._rho = math.exp(-self.model.opt.timestep / p.correlazione_s)
        # Partenza casuale col carrello in moto: nessun gradino fittizio di accelerazione.
        self._ultima_velocita = float(self.data.ctrl[0])
        self.forza_disturbo_n = self.coppia_disturbo_nm = 0.
        self.parametri_episodio = {
            'intensita': s, 'mass_scales': scales.tolist(), 'damping_scales': damping.tolist(),
            'motor_gain_scale': gain, 'frequencies_hz': freq.tolist(), 'zeta': zeta.tolist(),
            'amplitude_limits': self._ampiezze.tolist(),
            'delay_s': delay*self.model.opt.timestep, 'angle_offset_rad': self._offset,
            'position_noise_m': self._sigma_x, 'angle_noise_deg': math.degrees(self._sigma_theta),
            **perno}
        self._guasti_iniziali()
        self._misura(iniziale=True)
        return self.observation(), self.info()

    def _perno(self):
        """Attriti del perno del pendolo (dof 1), estratti solo se configurati:
        con i tre massimi a zero nessuna estrazione, sequenza casuale invariata."""
        p, rng = self.disturbi, self.np_random
        self.resistenza_aria = 0.
        if not (p.smorzamento_perno_max_nms or p.attrito_perno_max_nm or p.resistenza_aria_max_nms2):
            return {}
        if p.smorzamento_perno_max_nms:
            self.model.dof_damping[1] = math.exp(rng.uniform(
                math.log(p.smorzamento_perno_min_nms), math.log(p.smorzamento_perno_max_nms)))
        if p.attrito_perno_max_nm:
            self.model.dof_frictionloss[1] = rng.uniform(0., p.attrito_perno_max_nm)
        if p.resistenza_aria_max_nms2:
            self.resistenza_aria = rng.uniform(0., p.resistenza_aria_max_nms2)
        return {'pivot_damping_nms': float(self.model.dof_damping[1]),
                'pivot_friction_nm': float(self.model.dof_frictionloss[1]),
                'air_drag_nms2': self.resistenza_aria}

    def _guasti_iniziali(self):
        """Guasti di config.toml [sensori], estratti per episodio dopo i disturbi
        storici: con [sensori] tutto a zero la sequenza casuale non cambia."""
        g, rng = self.sensori, self.np_random
        self._guasti = g is not None and g.attivi
        self.errore_encoder_rad = self.errore_x_m = 0.
        if not self._guasti:
            return
        tick = 2*math.pi/self.disturbi.encoder_conteggi_giro
        massimo = math.radians(g.offset_max_gradi)
        self.errore_encoder_rad = rng.uniform(-massimo, massimo) if massimo else 0.
        self._p_perso = rng.uniform(0., g.prob_conteggio_perso_max) if g.prob_conteggio_perso_max else 0.
        self._frazione_passi = rng.uniform(0., g.passi_persi_max) if g.passi_persi_max else 0.
        self._theta_prec, self._x_prec = float(self.data.qpos[1]), float(self.data.qpos[0])
        self._tick = tick
        self.parametri_episodio.update({
            'encoder_offset_deg': math.degrees(self.errore_encoder_rad),
            'encoder_lost_count_probability': self._p_perso,
            'lost_step_fraction': self._frazione_passi})

    def _aggiorna_guasti(self):
        """Errori che si accumulano col moto, una volta per lettura (ogni periodo)."""
        g, rng = self.sensori, self.np_random
        theta, x = float(self.data.qpos[1]), float(self.data.qpos[0])
        d_theta, d_x = theta - self._theta_prec, x - self._x_prec
        self._theta_prec, self._x_prec = theta, x
        if self._p_perso:
            conteggi = int(abs(d_theta)/self._tick)
            persi = rng.binomial(conteggi, self._p_perso) if conteggi else 0
            # Conteggi persi: l'angolo misurato si muove meno di quello vero.
            self.errore_encoder_rad -= math.copysign(persi*self._tick, d_theta)
        if g.salti_al_s:
            for _ in range(rng.poisson(g.salti_al_s*self.dt)):
                ampiezza = int(rng.integers(1, g.salto_max_conteggi + 1))
                self.errore_encoder_rad += (1 if rng.random() < 0.5 else -1)*ampiezza*self._tick
        if self._frazione_passi:
            # Passi persi: il conteggio degli STEP supera lo spostamento vero.
            self.errore_x_m += self._frazione_passi*d_x/(1. - self._frazione_passi)

    def _misura(self, iniziale=False):
        """Lettura rumorosa a ogni periodo, poi correzione dello stimatore.

        La rete non vede le misure grezze: derivarle amplificherebbe il rumore.
        """
        p = self.disturbi
        x, theta = self.data.qpos
        if self._guasti:
            if not iniziale:
                self._aggiorna_guasti()
            x += self.errore_x_m
            theta += self.errore_encoder_rad
        x += self.np_random.normal(0., self._sigma_x)
        theta += self._offset + self.np_random.normal(0., self._sigma_theta)
        tick = 2*math.pi/p.encoder_conteggi_giro
        theta = round(theta/tick)*tick
        self.misura_grezza = (x, theta)
        if iniziale:
            # Partenza casuale (solo addestramento): lo stimatore parte dalla omega
            # vera, come se il pendolo fosse seguito da prima; altrimenti fermo.
            omega = float(self.data.qvel[1]) if self.partenza_casuale else 0.
            self.stimatore.reset(x, theta, self.comando_motore.velocita_carrello_m_s, omega)
        else:
            self.stimatore.correggi(x, theta)
        st = self.stimatore
        self._obs = np.array([st.x/self.limite, math.sin(st.theta), math.cos(st.theta),
                              st.velocita/self.controllo.p.velocita_max_m_s,
                              st.omega/self.ep.scala_omega_rad_s,
                              self.comando_motore.frequenza_hz/self.controllo.p.frequenza_max_hz],
                             dtype=np.float32)

    def _forze(self, velocita, push):
        p = self.disturbi
        accel = (velocita-self._ultima_velocita)/self.model.opt.timestep
        self._ultima_velocita = velocita
        self._colore = (self._rho*self._colore + math.sqrt(1-self._rho**2)
                        * self.np_random.normal(size=3))
        drive = np.clip(accel/p.accelerazione_scala_m_s2, -1., 1.)
        response = self.modi.step(drive + p.eccitazione_casuale*self._colore)
        loads = self._ampiezze*np.tanh(response)
        self.forza_disturbo_n = float(loads[0]+loads[1])
        self.coppia_disturbo_nm = float(loads[2])
        coppia = self.coppia_disturbo_nm
        if self.resistenza_aria:
            omega = float(self.data.qvel[1])
            coppia -= self.resistenza_aria*omega*abs(omega)
        self.data.qfrc_applied[:] = [push+self.forza_disturbo_n, coppia]

    def step(self, action, *, push=0.):
        # Stesso ciclo/ricompensa del nominale, con ritardo e forze a 500 Hz.
        if self._episode_done:
            raise RuntimeError('Chiamare reset() prima di iniziare un nuovo episodio.')
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (1,) or not np.isfinite(action).all() or not math.isfinite(push):
            raise ValueError('Serve un’azione finita di forma (1,) e una spinta finita.')
        u = float(np.clip(action[0], -1., 1.))
        self.frequenza_richiesta_hz = self.controllo.azione_a_hz(u)
        start = self.data.time
        for _ in range(self.frame_skip):
            if self._fuori_corsa():
                self.motivo = 'finecorsa'
                break
            self._azioni.append(u)
            delayed = self._azioni.popleft()
            self.comando_motore = self.controllo.avanza(delayed, self.model.opt.timestep)
            self.data.ctrl[0] = self.comando_motore.velocita_carrello_m_s
            # Sul firmware: stessa previsione nell'interrupt della rampa a 500 Hz.
            self.stimatore.predici(self.comando_motore.velocita_carrello_m_s, self.model.opt.timestep)
            self._forze(self.data.ctrl[0], push)
            mujoco.mj_step(self.model, self.data)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                self._stop()
                self._episode_done = True
                raise FloatingPointError('Stato fisico non finito.')
            if self._fuori_corsa():
                self.motivo = 'finecorsa'
                break
        reward, terminated, truncated = self._fine_passo(u)
        if self.data.time > start:
            self._misura()
        else:
            self._misura(iniziale=True)
        return self.observation(), reward, terminated, truncated, self.info()


# =============================================================================
#  Storia degli ingressi passati ([storia])
# =============================================================================
# La rete riceve i sei ingressi attuali e, per ogni ritardo k di [storia].ritardi,
# i sei ingressi di k periodi fa con l'azione decisa allora:
#
#     [obs(t), obs(t-k1), u(t-k1), obs(t-k2), u(t-k2), ...]   6 + 7*len(ritardi)
#
# I sei ingressi sono quelli di sempre (stimati nell'ambiente disturbato);
# u e l'azione in [-1, 1] dopo la saturazione, prima di ritardi e rampa.
# All'inizio dell'episodio la storia e riempita con i primi ingressi e u = 0,
# come nel firmware all'avvio con B1 (Storia_Reset in main.c).
# Senza ritardi l'ambiente resta quello storico (sei ingressi).

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


def crea_ambiente(disturbi=None, *, configurazione=None, **kwargs):
    """Ambiente nominale o disturbato.

    Con configurazione=..., episodio, ricompensa, sensori, stepper e stimatore
    vengono da li; `disturbi` (anche None) sceglie il profilo di disturbi.
    """
    kwargs['configurazione'] = configurazione
    nominale = disturbi is None or disturbi.intensita == 0
    classe = CartPendoloEnv if nominale else CartPendoloDisturbatoEnv
    if configurazione is not None and configurazione.storia.ritardi:
        classe = con_storia(classe)
    return classe(**kwargs) if nominale else classe(disturbi, **kwargs)
