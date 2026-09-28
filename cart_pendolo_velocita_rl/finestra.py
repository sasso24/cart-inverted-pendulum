"""Viewer essenziale: input e grafica sullo stesso thread principale macOS."""
import math
import time

import glfw
import mujoco
import numpy as np


class Comandi:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.paused = False
        self.restart = False
        self.push_until = 0.
        self.push_force = 0.
        self.last = "Nessun tasto ricevuto"
        self.manual_target = None
        self.manual_deadline = 0.

    def move_cart(self, env, direction):
        """Una pressione = 10 cm dalla posizione simulata corrente."""
        if self.manual_target is not None:
            self.last = "Spostamento in corso: attendi oppure C per annullare"
            return
        target = float(env.data.qpos[0]) + direction * 0.10
        if abs(target) > env.limite - 0.02:
            self.last = "Spostamento rifiutato: troppo vicino al finecorsa"
            return
        self.manual_target = target
        self.manual_deadline = env.data.time + 10.
        self.last = f"Manuale: {'+10' if direction > 0 else '-10'} cm"

    def manual_action(self, env):
        """Inseguimento manuale in velocita; l'ambiente applica la rampa."""
        if self.manual_target is None:
            return None
        error = self.manual_target - float(env.data.qpos[0])
        velocity = float(env.data.qvel[0])
        if abs(error) <= 0.0005 and abs(velocity) <= 0.003:
            self.manual_target = None
            self.last = "Spostamento completato: ripreso il controllo precedente"
            return None
        if env.data.time >= self.manual_deadline:
            self.manual_target = None
            self.enabled = False
            self.last = "Timeout manuale: arresto, C per riattivare la policy"
            return None
        target_velocity = float(np.clip(4. * error, -0.3, 0.3))
        p = env.controllo.p
        hz = abs(target_velocity) / p.metri_per_impulso
        if hz == 0:
            return np.array([0.], dtype=np.float32)
        # Inversione della stessa conversione usata dalla policy; anche con minimo.
        fraction = np.clip((hz - p.frequenza_min_hz) /
                           (p.frequenza_max_hz - p.frequenza_min_hz), 0., 1.)
        magnitude = max(1e-6, p.zona_morta + (1. - p.zona_morta) * fraction)
        if magnitude <= p.zona_morta:
            magnitude = float(np.nextafter(np.float32(p.zona_morta), np.float32(1.)))
        return np.array([math.copysign(magnitude, target_velocity)], dtype=np.float32)

    def key(self, key, action, simulation_time):
        if action not in (glfw.PRESS, glfw.REPEAT):
            return
        # Non ripetere i toggle quando un tasto resta premuto.
        if action == glfw.REPEAT and key not in (glfw.KEY_A, glfw.KEY_D):
            return
        if key == glfw.KEY_R:
            self.manual_target = None
            self.restart = True
            self.push_until = 0.
            self.push_force = 0.
            self.last = "R: riavvio"
        elif key == glfw.KEY_C:
            self.manual_target = None
            self.enabled = not self.enabled
            self.last = "C: policy / arresto " + ("attivo" if self.enabled else "disattivo")
        elif key == glfw.KEY_SPACE:
            self.paused = not self.paused
            self.last = "Spazio: " + ("pausa" if self.paused else "ripresa")
        elif key in (glfw.KEY_A, glfw.KEY_D):
            self.push_force = -10. if key == glfw.KEY_A else 10.
            self.push_until = simulation_time + 0.1
            self.last = ("A: spinta a sinistra" if key == glfw.KEY_A
                         else "D: spinta a destra")
        else:
            self.last = f"Tasto {key}: nessun comando associato"

    def force(self, simulation_time):
        return self.push_force if simulation_time < self.push_until else 0.


def prepare_continuous(env):
    """Disattiva il timeout solo su questa istanza, senza alterare il training.

    Reset manuali e arresti fisici restano validi. Non cambiamo il passo fisico
    né il periodo della policy: il loop grafico segue il tempo monotono del PC.
    """
    env.max_steps = math.inf
    # env.reset(options={"exact": True})
    # env.data.qpos[1] = 0.0       # Asta verticale verso l'alto
    # env.data.qvel[:] = 0.0      # Carrello e asta inizialmente fermi
    # mujoco.mj_forward(env.model, env.data)
    # return env.observation(), env.info()
    return env.reset(options={"exact": True})


def run(env, agent):
    # Una sola finestra, un solo event loop: niente callback Python sul thread
    # del viewer passivo. Su macOS eseguire con python, sul thread principale.
    if not glfw.init():
        raise RuntimeError("Impossibile inizializzare la finestra grafica.")
    window = None
    context = None
    try:
        glfw.window_hint(glfw.FOCUSED, glfw.TRUE)
        window = glfw.create_window(1100, 760, "Pendolo stepper - SAC velocita Hz", None, None)
        if not window:
            raise RuntimeError("Impossibile creare la finestra grafica.")
        glfw.make_context_current(window)
        glfw.swap_interval(1)
        model, data = env.model, env.data
        obs, _ = prepare_continuous(env)
        scene = mujoco.MjvScene(model, maxgeom=1000)
        context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        camera = mujoco.MjvCamera()
        camera.azimuth = 270
        camera.elevation = -12
        camera.distance = 5.8
        camera.lookat[:] = [0, 0, 1.3]
        options = mujoco.MjvOption()
        commands = Comandi(enabled=agent is not None)

        def on_key(win, key, scancode, action, mods):
            if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
                glfw.set_window_should_close(win, True)
            if key in (glfw.KEY_LEFT, glfw.KEY_RIGHT):
                # Ignora l'auto-repeat: tenere premuto non moltiplica i 10 cm.
                if action == glfw.PRESS:
                    if stopped:
                        commands.last = "Simulazione arrestata: premi R prima di muovere"
                    else:
                        commands.move_cart(env, -1 if key == glfw.KEY_LEFT else 1)
                return
            if key == glfw.KEY_C and agent is None:
                commands.manual_target = None
                commands.enabled = False
                commands.last = "Nessuna rete caricata: frecce per muovere, R per reset"
                return
            commands.key(key, action, data.time)

        def on_scroll(win, xoffset, yoffset):
            camera.distance = np.clip(camera.distance * math.exp(-0.1 * yoffset), 1., 20.)

        previous = [None]

        def on_cursor(win, x, y):
            if previous[0] is not None and glfw.get_mouse_button(win, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS:
                old_x, old_y = previous[0]
                camera.azimuth -= 0.25 * (x - old_x)
                camera.elevation = np.clip(camera.elevation - 0.25 * (y - old_y), -89., 89.)
            previous[0] = (x, y)

        glfw.set_key_callback(window, on_key)
        glfw.set_scroll_callback(window, on_scroll)
        glfw.set_cursor_pos_callback(window, on_cursor)
        glfw.show_window(window)
        glfw.focus_window(window)
        previous_time = time.monotonic()
        accumulator = 0.
        stopped = False
        end_message = ""
        while not glfw.window_should_close(window):
            glfw.poll_events()
            now = time.monotonic()
            elapsed = min(now - previous_time, 0.1)
            previous_time = now
            if commands.restart:
                obs, _ = env.reset(options={"exact": True})
                commands.restart = False
                accumulator = 0.
                stopped = False
                end_message = ""
            if commands.paused or stopped:
                accumulator = 0.
            else:
                accumulator += elapsed
                while accumulator >= env.dt:
                    action = commands.manual_action(env)
                    if action is not None:
                        pass  # La manovra manuale ha temporaneamente la precedenza.
                    elif commands.enabled and agent is not None:
                        action, _ = agent.predict(obs, deterministic=True)
                    else:
                        # Zero richiede una decelerazione tramite la rampa.
                        action = np.array([0.], dtype=np.float32)
                    obs, _, terminated, truncated, info = env.step(action, push=commands.force(data.time))
                    accumulator -= env.dt
                    if terminated or truncated:
                        # Nessun riavvio automatico: dopo un arresto serve R.
                        reason = info["termination_reason"] if terminated else "limite temporale inatteso"
                        end_message = "Arresto: " + reason + " | Premi R per ripartire"
                        commands.manual_target = None
                        stopped = True
                        accumulator = 0.
                        break

            # Aggiorna le posizioni anche al reset e durante la pausa.
            mujoco.mj_forward(model, data)
            width, height = glfw.get_framebuffer_size(window)
            if width == 0 or height == 0:
                time.sleep(0.02)
                continue
            viewport = mujoco.MjrRect(0, 0, width, height)
            mujoco.mjv_updateScene(model, data, options, None, camera,
                                  mujoco.mjtCatBit.mjCAT_ALL, scene)
            mujoco.mjr_render(viewport, scene, context)
            active = glfw.get_window_attrib(window, glfw.FOCUSED)
            status = ("ARRESTATO" if stopped else "IN PAUSA" if commands.paused else "CONTINUO")
            status += " | " + ("MANUALE 10 cm" if commands.manual_target is not None
                                else "SAC ATTIVO" if commands.enabled else "ARRESTO")
            # HUD compatto: caratteri piccoli e quattro righe, senza ripetere
            # intestazioni e comandi. Conserva gli indicatori utili al controllo.
            info = env.info()
            text = (status + f" | t: {data.time:.1f} s | equilibrio: {info['stable_seconds']:.1f} s\n"
                    + f"x: {data.qpos[0]:.2f} m | STEP: {info['motor_frequency_hz']:+.0f} Hz | "
                    + f"angolo: {info['angle_deg']:.1f} deg | v: {float(env.data.qvel[0]):.2f} m/s\n"
                    + "Frecce: +/-10 cm  C: policy  R: reset  A/D: spinta  Spazio: pausa  Esc: esci\n"
                    + "Mouse: ruota / zoom | " + commands.last)
            if end_message:
                text += "\n" + end_message
            if not active:
                text += "\nClicca nella finestra per usare la tastiera"
            mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL,
                               mujoco.mjtGridPos.mjGRID_TOPLEFT,
                               viewport, text, "", context)
            glfw.swap_buffers(window)
            time.sleep(0.001)
    finally:
        if context is not None:
            context.free()
        if window is not None:
            glfw.destroy_window(window)
        glfw.terminate()
