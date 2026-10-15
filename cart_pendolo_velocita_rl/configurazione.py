"""Configurazione unica dell'addestramento: legge config.toml.

Un solo file decide episodio (swing-up o solo equilibrio), ricompensa,
disturbi, guasti dei sensori, motore, stimatore, rete/SAC e parametri del
firmware. Ogni addestramento salva una copia completa della configurazione
in <cartella>/config.json: simulazione, valutazione ed esportazione usano
quella copia, non il config.toml corrente.

I valori di default delle classi riproducono il comportamento precedente
(solo equilibrio dall'alto, ricompensa del 1 ottobre): servono ai vecchi
checkpoint, che non hanno una configurazione salvata.
"""
from dataclasses import asdict, dataclass, field, fields
import math
from pathlib import Path
import tomllib

from stepper import ParametriStepper
from stima import ParametriStimatore

ROOT = Path(__file__).resolve().parent
CONFIG_DEFAULT = ROOT / 'config.toml'


def _numeri(obj, nonnegativi=()):
    for f in fields(obj):
        v = getattr(obj, f.name)
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            continue
        if not math.isfinite(v):
            raise ValueError(f'{type(obj).__name__}.{f.name}: serve un numero finito.')
        if f.name in nonnegativi and v < 0:
            raise ValueError(f'{type(obj).__name__}.{f.name}: non puo essere negativo.')


@dataclass(frozen=True)
class ParametriModello:
    file_xml: str = 'cart_pendolo.xml'


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

    def __post_init__(self):
        _numeri(self, {f.name for f in fields(self)} - {'angolo_iniziale_gradi'})
        if not 0 <= self.frazione_partenza_alto <= 1:
            raise ValueError('frazione_partenza_alto deve essere in [0, 1].')
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
    # Lettura encoder scartata come guasto se in 20 ms implica piu di questo.
    omega_max_plausibile_rad_s: float = 25.0


SEZIONI = {'modello': ParametriModello, 'episodio': ParametriEpisodio,
           'ricompensa': ParametriRicompensa, 'sensori': ParametriSensori,
           'addestramento': ParametriAddestramento, 'firmware': ParametriFirmware,
           'stepper': ParametriStepper, 'stimatore': ParametriStimatore}


@dataclass(frozen=True)
class Configurazione:
    modello: ParametriModello = field(default_factory=ParametriModello)
    episodio: ParametriEpisodio = field(default_factory=ParametriEpisodio)
    ricompensa: ParametriRicompensa = field(default_factory=ParametriRicompensa)
    sensori: ParametriSensori = field(default_factory=ParametriSensori)
    addestramento: ParametriAddestramento = field(default_factory=ParametriAddestramento)
    firmware: ParametriFirmware = field(default_factory=ParametriFirmware)
    stepper: ParametriStepper = field(default_factory=ParametriStepper.carica)
    stimatore: ParametriStimatore = field(default_factory=ParametriStimatore.carica)
    disturbi: object = None   # ParametriDisturbi oppure None (ambiente nominale)

    @classmethod
    def da_dict(cls, dati):
        from disturbi import ParametriDisturbi
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
        import json
        risultato = {nome: asdict(getattr(self, nome)) for nome in SEZIONI}
        risultato['disturbi'] = None if self.disturbi is None else asdict(self.disturbi)
        return json.loads(json.dumps(risultato))

    @property
    def disturbi_addestramento(self):
        return self.disturbi if self.addestramento.usa_disturbi else None


def configurazione_modello(cartella):
    """Configurazione salvata con un modello; per i vecchi checkpoint quella storica."""
    import json
    from disturbi import ParametriDisturbi
    config = json.loads((Path(cartella) / 'config.json').read_text())
    if 'configurazione' in config:
        return Configurazione.da_dict(config['configurazione'])
    # Checkpoint precedenti: episodio, ricompensa e sensori storici (default),
    # stepper.toml e stimatore.toml, profilo disturbi salvato nel contratto.
    valori = config.get('environment', {}).get('disturbi')
    return Configurazione(disturbi=None if valori is None else ParametriDisturbi(**valori))


def configurazione_ambiente(cartella):
    """Configurazione da passare all'ambiente: None per i vecchi checkpoint
    (ambiente storico identico), altrimenti quella salvata col modello."""
    import json
    config = json.loads((Path(cartella) / 'config.json').read_text())
    return configurazione_modello(cartella) if 'configurazione' in config else None
