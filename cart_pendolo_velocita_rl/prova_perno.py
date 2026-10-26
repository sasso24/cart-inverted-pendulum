"""Pendolo libero col carrello fermo: parte da un angolo (default 90 gradi) e cade.

Serve a confrontare con il banco attrito e smorzamento del perno: si cambiano
direttamente dalla finestra e si guarda quanto oscilla (tempo per scendere sotto
45, 20, 10, 5, 2, 1 gradi e per fermarsi). Non modifica XML, configurazioni o
reti: i valori cambiano solo in memoria.

    python prova_perno.py
    python prova_perno.py --smorzamento 0.004 --attrito 0.002
    python prova_perno.py --headless --smorzamento 0.004 --attrito 0.002   # tabella, senza finestra

Tasti: Q/A smorzamento x1,25 / :1,25   W/S attrito   E/D aria   Z tutto a zero
       frecce su/giu angolo di partenza +/-10 gradi   R riparti   spazio pausa   Esc esci

Al banco: motore fermo (driver acceso, carrello tenuto), pendolo a 90 gradi, lasciarlo
e cronometrare. Calo dell'ampiezza costante a ogni oscillazione = attrito coulombiano;
calo in proporzione all'ampiezza (rapporto costante) = smorzamento viscoso.
"""
import argparse
import math
from pathlib import Path
import time

import mujoco

ROOT = Path(__file__).resolve().parent
SOGLIE_GRADI = (45., 20., 10., 5., 2., 1.)
FERMO_GRADI = 0.5
OMEGA_FERMO = 0.005   # rad/s
MINIMI = {'smorzamento': 1e-4, 'attrito': 1e-4, 'aria': 1e-5}   # primo valore salendo da zero


class Prova:
    def __init__(self, xml, smorzamento, attrito, aria, angolo):
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        giunto = self.model.joint('rotazione')
        self.dof, self.qadr = int(giunto.dofadr[0]), int(giunto.qposadr[0])
        self.smorzamento, self.attrito, self.aria, self.angolo = smorzamento, attrito, aria, angolo
        self.riparti()

    def applica(self):
        self.model.dof_damping[self.dof] = self.smorzamento
        self.model.dof_frictionloss[self.dof] = self.attrito

    def riparti(self):
        mujoco.mj_resetData(self.model, self.data)
        # Angoli dell'XML: 0 = braccio lungo in alto, pi = appeso.
        self.data.qpos[self.qadr] = math.pi + math.radians(self.angolo)
        self.data.ctrl[:] = 0.          # attuatore di velocita a zero: carrello tenuto fermo
        self.applica()
        mujoco.mj_forward(self.model, self.data)
        self.picchi = [(0., abs(self.angolo))]   # (t, ampiezza in gradi da appeso)
        self.soglie = {}
        self.fermo_t = self.quiete_da = None

    @property
    def angolo_da_appeso(self):
        return math.degrees(math.remainder(float(self.data.qpos[self.qadr]) - math.pi, 2*math.pi))

    def passo(self):
        omega = float(self.data.qvel[self.dof])
        self.data.qfrc_applied[self.dof] = -self.aria*omega*abs(omega)
        mujoco.mj_step(self.model, self.data)
        if self.fermo_t is not None:
            return
        nuova = float(self.data.qvel[self.dof])
        t = float(self.data.time)
        # Picco = inversione del verso; 0,2 s minimi tra due picchi contro il rumore numerico.
        if omega*nuova <= 0 and omega != 0 and t - self.picchi[-1][0] > 0.2:
            ampiezza = abs(self.angolo_da_appeso)
            self.picchi.append((t, ampiezza))
            for s in SOGLIE_GRADI:
                if ampiezza < s and s not in self.soglie:
                    self.soglie[s] = t
            if ampiezza < FERMO_GRADI:
                self.fermo_t = t
        # Bloccato dall'attrito coulombiano (anche a qualche decimo di grado dal fondo):
        # velocita quasi nulla per 1 s; il tempo di arresto e l'inizio di quel secondo.
        if abs(nuova) > OMEGA_FERMO:
            self.quiete_da = None
        elif self.quiete_da is None:
            self.quiete_da = t
        elif t - self.quiete_da >= 1. and self.fermo_t is None:
            self.fermo_t = self.quiete_da

    def periodo(self):
        if len(self.picchi) < 3:
            return None
        t = [p[0] for p in self.picchi[1:]]
        return 2*(t[-1] - t[0])/(len(t) - 1) if len(t) > 1 else None

    def calo(self):
        """Ultimo calo per semi-oscillazione (gradi) e rapporto tra due ampiezze successive."""
        if len(self.picchi) < 2:
            return None, None
        a0, a1 = self.picchi[-2][1], self.picchi[-1][1]
        return a0 - a1, (a1/a0 if a0 else None)

    def parametri(self):
        return (f'smorzamento {self.smorzamento:.5f} N*m*s | attrito {self.attrito:.5f} N*m | '
                f'aria {self.aria:.2e} N*m*s^2 | partenza {self.angolo:.0f} gradi')

    def soglie_testo(self):
        parti = [f'<{s:g}: ' + (f'{self.soglie[s]:.1f} s' if s in self.soglie else '-') for s in SOGLIE_GRADI]
        parti.append('ferma: ' + (f'{self.fermo_t:.1f} s' if self.fermo_t is not None else '-'))
        return ' | '.join(parti)

    def riepilogo(self):
        oscillazioni = (len(self.picchi) - 1)/2
        periodo = self.periodo()
        return (self.parametri() + '\n  ' + self.soglie_testo()
                + f'\n  oscillazioni complete {oscillazioni:.1f}'
                + (f' | periodo {periodo:.3f} s' if periodo else '') + f' | t {self.data.time:.1f} s')


def senza_finestra(prova, durata):
    while prova.data.time < durata and prova.fermo_t is None:
        prova.passo()
    print(prova.parametri())
    print(' osc.     t (s)   ampiezza (gradi)   calo per osc. (gradi)   rapporto')
    picchi = prova.picchi[::2]   # una riga per oscillazione completa (stesso lato)
    for k, (t, a) in enumerate(picchi):
        if k == 0:
            print(f'{k:4d} {t:9.2f} {a:12.2f}')
        else:
            a0 = picchi[k-1][1]
            print(f'{k:4d} {t:9.2f} {a:12.2f} {a0 - a:18.2f} {a/a0 if a0 else 0:16.3f}')
    print(prova.riepilogo())


def con_finestra(prova):
    import glfw
    if not glfw.init():
        raise RuntimeError('Impossibile inizializzare la finestra grafica.')
    window = context = None
    try:
        window = glfw.create_window(1100, 760, 'Prova del perno - pendolo libero', None, None)
        if not window:
            raise RuntimeError('Impossibile creare la finestra grafica.')
        glfw.make_context_current(window)
        glfw.swap_interval(1)
        scene = mujoco.MjvScene(prova.model, maxgeom=1000)
        context = mujoco.MjrContext(prova.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        camera = mujoco.MjvCamera()
        camera.azimuth, camera.elevation, camera.distance = 270, -8, 3.2
        camera.lookat[:] = [0, 0, 1.0]
        options = mujoco.MjvOption()
        stato = {'pausa': False}

        def cambia(nome, fattore):
            valore = getattr(prova, nome)
            if fattore > 1:
                valore = MINIMI[nome] if valore == 0 else valore*fattore
            else:
                valore = 0. if valore*fattore < MINIMI[nome] else valore*fattore
            setattr(prova, nome, valore)
            prova.applica()
            print('Cambiato (R per ripartire con questi valori): ' + prova.parametri(), flush=True)

        def riparti():
            print('Prova finita: ' + prova.riepilogo(), flush=True)
            prova.riparti()
            print('Riparto: ' + prova.parametri(), flush=True)

        def on_key(win, key, scancode, action, mods):
            if action not in (glfw.PRESS, glfw.REPEAT):
                return
            tasti = {glfw.KEY_Q: ('smorzamento', 1.25), glfw.KEY_A: ('smorzamento', 0.8),
                     glfw.KEY_W: ('attrito', 1.25), glfw.KEY_S: ('attrito', 0.8),
                     glfw.KEY_E: ('aria', 1.25), glfw.KEY_D: ('aria', 0.8)}
            if key == glfw.KEY_ESCAPE:
                glfw.set_window_should_close(win, True)
            elif key in tasti:
                cambia(*tasti[key])
            elif key == glfw.KEY_Z:
                prova.smorzamento = prova.attrito = prova.aria = 0.
                prova.applica()
                print('Cambiato (R per ripartire con questi valori): ' + prova.parametri(), flush=True)
            elif key in (glfw.KEY_UP, glfw.KEY_DOWN):
                prova.angolo = min(180., max(10., prova.angolo + (10. if key == glfw.KEY_UP else -10.)))
                riparti()
            elif key == glfw.KEY_R:
                riparti()
            elif key == glfw.KEY_SPACE:
                stato['pausa'] = not stato['pausa']

        def on_scroll(win, xoffset, yoffset):
            camera.distance = min(20., max(0.5, camera.distance*math.exp(-0.1*yoffset)))

        precedente = [None]

        def on_cursor(win, x, y):
            if precedente[0] is not None and glfw.get_mouse_button(win, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS:
                camera.azimuth -= 0.25*(x - precedente[0][0])
                camera.elevation = min(89., max(-89., camera.elevation - 0.25*(y - precedente[0][1])))
            precedente[0] = (x, y)

        glfw.set_key_callback(window, on_key)
        glfw.set_scroll_callback(window, on_scroll)
        glfw.set_cursor_pos_callback(window, on_cursor)
        print('Parto: ' + prova.parametri(), flush=True)
        ultimo, accumulo, fermo_stampato = time.monotonic(), 0., False
        while not glfw.window_should_close(window):
            glfw.poll_events()
            adesso = time.monotonic()
            trascorso, ultimo = min(adesso - ultimo, 0.1), adesso
            if not stato['pausa']:
                accumulo += trascorso
                while accumulo >= prova.model.opt.timestep:
                    prova.passo()
                    accumulo -= prova.model.opt.timestep
            if prova.fermo_t is not None and not fermo_stampato:
                print('Fermo: ' + prova.riepilogo(), flush=True)
                fermo_stampato = True
            elif prova.fermo_t is None:
                fermo_stampato = False
            mujoco.mj_forward(prova.model, prova.data)
            larghezza, altezza = glfw.get_framebuffer_size(window)
            if larghezza == 0 or altezza == 0:
                time.sleep(0.02)
                continue
            viewport = mujoco.MjrRect(0, 0, larghezza, altezza)
            mujoco.mjv_updateScene(prova.model, prova.data, options, None, camera,
                                   mujoco.mjtCatBit.mjCAT_ALL, scene)
            mujoco.mjr_render(viewport, scene, context)
            periodo = prova.periodo()
            calo, rapporto = prova.calo()
            testo = (prova.parametri() + '\n'
                     + f't {prova.data.time:.1f} s | angolo da appeso {prova.angolo_da_appeso:+.1f} gradi | '
                     + f'oscillazioni complete {(len(prova.picchi) - 1)/2:.1f}'
                     + (f' | periodo {periodo:.3f} s' if periodo else '') + '\n'
                     + f'ultima ampiezza {prova.picchi[-1][1]:.1f} gradi'
                     + (f' | calo per semi-oscillazione {calo:.2f} gradi (rapporto {rapporto:.3f})'
                        if calo is not None and rapporto is not None else '') + '\n'
                     + prova.soglie_testo() + '\n'
                     + 'Q/A smorzamento  W/S attrito  E/D aria  Z zero  Su/Giu angolo  R riparti  '
                     + 'Spazio pausa  Esc' + ('\nIN PAUSA' if stato['pausa'] else ''))
            mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                               viewport, testo, '', context)
            glfw.swap_buffers(window)
            time.sleep(0.001)
        print('Ultima prova: ' + prova.riepilogo(), flush=True)
    finally:
        if context is not None:
            context.free()
        if window is not None:
            glfw.destroy_window(window)
        glfw.terminate()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--xml', type=Path, default=ROOT / 'cart_pendolo_asta20_perno.xml')
    parser.add_argument('--smorzamento', type=float, default=0.002, help='Viscoso, N*m*s/rad')
    parser.add_argument('--attrito', type=float, default=0.001, help='Coulombiano, N*m')
    parser.add_argument('--aria', type=float, default=0., help='Coppia -c*omega*|omega|, c in N*m*s^2')
    parser.add_argument('--angolo', type=float, default=90., help='Partenza in gradi da appeso (90 = orizzontale)')
    parser.add_argument('--headless', action='store_true', help='Solo tabella nel terminale')
    parser.add_argument('--durata', type=float, default=300., help='Con --headless: tempo massimo in s')
    args = parser.parse_args()
    if min(args.smorzamento, args.attrito, args.aria) < 0:
        parser.error('Valori non negativi.')
    prova = Prova(args.xml, args.smorzamento, args.attrito, args.aria, args.angolo)
    if args.headless:
        senza_finestra(prova, args.durata)
    else:
        con_finestra(prova)


if __name__ == '__main__':
    main()
