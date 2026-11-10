#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
selector_tui.py - Interfaz TUI interactiva con Textual para seleccionar la configuración de lanzamiento
de robots UR y pinzas OnRobot (Física o Simulación).
"""

import os
import sys
import json
import argparse
import subprocess
from typing import Dict, Any, Optional
from textual.app import App, ComposeResult
from textual.containers import Container, Horizontal, Vertical, Grid
from textual.widgets import Header, Footer, Static, Button, RadioSet, RadioButton, Label, Checkbox


def ping_ip(ip: str) -> bool:
    """Comprueba si una IP responde a ping con timeout de 1s."""
    try:
        res = subprocess.run(['ping', '-c', '1', '-W', '1', ip],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return res.returncode == 0
    except Exception:
        return False


class RobotSelectorTUI(App[Optional[Dict[str, Any]]]):
    CSS = """
    Screen {
        background: #1e1e2e;
        color: #cdd6f4;
        overflow-y: auto;
    }
    Header {
        background: #313244;
        color: #89b4fa;
        text-style: bold;
    }
    #main-container {
        padding: 0 1;
        height: auto;
        overflow-y: auto;
    }
    .panel {
        background: #181825;
        border: solid #45475a;
        padding: 0 1;
        margin: 0;
        height: auto;
    }
    #network-panel {
        height: 3;
        margin-bottom: 1;
        align: left middle;
    }
    .panel-title {
        color: #f9e2af;
        text-style: bold;
        margin: 0;
    }
    .status-ok {
        color: #a6e3a1;
        text-style: bold;
    }
    .status-err {
        color: #f38ba8;
        text-style: bold;
    }
    #columns-container {
        height: auto;
        margin-top: 0;
    }
    #columns-container > .panel {
        width: 1fr;
        margin-right: 1;
    }
    #columns-container > .panel:last-child {
        margin-right: 0;
    }
    RadioSet {
        background: transparent;
        border: none;
        padding: 0;
    }
    RadioButton {
        background: transparent;
        padding: 0;
        height: 1;
    }
    RadioButton:focus {
        color: #89b4fa;
    }
    #buttons-bar {
        margin-top: 1;
        height: 3;
        align: center middle;
    }
    Button {
        margin: 0 2;
        min-width: 16;
    }
    #btn-launch {
        background: #a6e3a1;
        color: #11111b;
        text-style: bold;
    }
    #btn-cancel {
        background: #f38ba8;
        color: #11111b;
        text-style: bold;
    }
    """

    BINDINGS = [
        ("q", "quit", "Salir"),
        ("escape", "quit", "Cancelar"),
        ("enter", "submit", "Lanzar"),
    ]

    def __init__(self, default_ur: str = "ur5e", default_onrobot: str = "2fg7",
                 default_sim: bool = True, default_env: str = "basic", default_rviz: bool = True):
        super().__init__()
        self.default_ur = default_ur
        self.default_onrobot = default_onrobot
        self.default_sim = default_sim
        self.default_env = default_env if default_env in ('basic', 'left', 'right') else 'basic'
        self.default_rviz = default_rviz
        self.result_config: Optional[Dict[str, Any]] = None

        # IPs de robots físicos
        self.ips = {
            'left': '192.168.1.105',
            'right': '192.168.1.101'
        }
        self.detected = {}

    def on_mount(self) -> None:
        self.title = "🤖 APERTA / UR OnRobot Launcher"
        # Escaneo inicial de red en hilo rápido
        for side, ip in self.ips.items():
            ok = ping_ip(ip)
            self.detected[side] = ok
            label = self.query_one(f"#status-{side}", Static)
            side_str = side.upper()
            if ok:
                label.update(f"{side_str}: 🟢 [b]OK[/b] ({ip})")
                label.set_class(True, "status-ok")
                label.set_class(False, "status-err")
            else:
                label.update(f"{side_str}: 🔴 [dim]Off[/dim] ({ip})")
                label.set_class(False, "status-ok")
                label.set_class(True, "status-err")

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Vertical(id="main-container"):
            # 1. Panel de Estado de Red (Horizontal y compacto)
            with Horizontal(classes="panel", id="network-panel"):
                yield Static("📡 Red Robots: ", classes="bold")
                yield Static("LEFT: 🔍...", id="status-left")
                yield Static("   |   ", classes="bold")
                yield Static("RIGHT: 🔍...", id="status-right")

            # 2. Configuración en 4 columnas
            is_sim = self.default_sim
            is_real_left = (not self.default_sim and self.default_env == "left")
            is_real_right = (not self.default_sim and self.default_env != "left")

            with Horizontal(id="columns-container"):
                # Modo de Ejecución
                with Vertical(classes="panel", id="col-mode"):
                    yield Label("⚙️  Modo", classes="panel-title")
                    with RadioSet(id="mode-radios"):
                        yield RadioButton("🎮 Simulación", value=is_sim, id="mode-sim")
                        yield RadioButton("🤖 Robot LEFT", value=is_real_left, id="mode-real-left")
                        yield RadioButton("🤖 Robot RIGHT", value=is_real_right, id="mode-real-right")

                # Entorno
                with Vertical(classes="panel", id="col-env"):
                    yield Label("🏭 Entorno", classes="panel-title")
                    with RadioSet(id="env-radios"):
                        yield RadioButton("Básico (Solo robot)", value=(self.default_env == "basic"), id="env-basic")
                        yield RadioButton("Izquierdo (Mesa 1)", value=(self.default_env == "left"), id="env-left")
                        yield RadioButton("Derecho (Mesa 2)", value=(self.default_env == "right"), id="env-right")

                # Gripper
                with Vertical(classes="panel", id="col-gripper"):
                    yield Label("🦾 Efector", classes="panel-title")
                    with RadioSet(id="gripper-radios"):
                        yield RadioButton("2FG7 (Paralela)", value=(self.default_onrobot == "2fg7"), id="grip-2fg7")
                        yield RadioButton("3FG15 (3 Dedos)", value=(self.default_onrobot == "3fg15"), id="grip-3fg15")
                        yield RadioButton("VGC10 (Vacío)", value=(self.default_onrobot == "vgc10"), id="grip-vgc10")

                # Visualizador RViz2 (Opciones explícitas con RadioButtons)
                with Vertical(classes="panel", id="col-rviz"):
                    yield Label("👁️ RViz2", classes="panel-title")
                    with RadioSet(id="rviz-radios"):
                        yield RadioButton("🟢 Lanzar RViz", value=self.default_rviz, id="rviz-yes")
                        yield RadioButton("🔴 No lanzar RViz", value=(not self.default_rviz), id="rviz-no")

            # 3. Botones de acción
            with Horizontal(id="buttons-bar"):
                yield Button("🚀 LANZAR", id="btn-launch", variant="success")
                yield Button("❌ CANCELAR", id="btn-cancel", variant="error")

        yield Footer()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "btn-launch":
            self.action_submit()
        elif event.button.id == "btn-cancel":
            self.action_quit()

    def action_submit(self) -> None:
        # 1. Determinar modo
        mode_radios = self.query_one("#mode-radios", RadioSet)
        mode_id = mode_radios.pressed_button.id if mode_radios.pressed_button else "mode-sim"

        if mode_id == "mode-real-left":
            use_sim = "false"
            robot_ip = self.ips['left']
            sim_env = "left"
        elif mode_id == "mode-real-right":
            use_sim = "false"
            robot_ip = self.ips['right']
            sim_env = "right"
        else:
            use_sim = "true"
            robot_ip = "127.0.0.1"
            # Tomar del selector de entorno
            env_radios = self.query_one("#env-radios", RadioSet)
            env_id = env_radios.pressed_button.id if env_radios.pressed_button else "env-basic"
            if env_id == "env-left":
                sim_env = "left"
            elif env_id == "env-right":
                sim_env = "right"
            else:
                sim_env = "basic"

        # 2. Determinar gripper
        grip_radios = self.query_one("#gripper-radios", RadioSet)
        grip_id = grip_radios.pressed_button.id if grip_radios.pressed_button else "grip-2fg7"
        if grip_id == "grip-3fg15":
            onrobot_type = "3fg15"
        elif grip_id == "grip-vgc10":
            onrobot_type = "vgc10"
        else:
            onrobot_type = "2fg7"

        # 3. RViz (RadioSet explícito con rviz-yes / rviz-no)
        rviz_radios = self.query_one("#rviz-radios", RadioSet)
        rviz_id = rviz_radios.pressed_button.id if rviz_radios.pressed_button else "rviz-yes"
        launch_rviz = "false" if rviz_id == "rviz-no" else "true"

        self.result_config = {
            'use_simulation': use_sim,
            'sim_env': sim_env,
            'robot_ip': robot_ip,
            'onrobot_type': onrobot_type,
            'launch_rviz': launch_rviz,
            'cancelled': False,
        }
        self.exit(result=self.result_config)

    def action_quit(self) -> None:
        self.exit(result={'cancelled': True})


def run_tui(default_ur: str = "ur5e", default_onrobot: str = "2fg7",
            default_sim: bool = True, default_env: str = "basic", default_rviz: bool = True) -> Optional[Dict[str, Any]]:
    """Ejecuta la TUI de Textual y devuelve la configuración seleccionada."""
    import asyncio
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        loop = None

    # Si ya hay un event loop de asyncio activo (ej: ROS 2 launch service),
    # delegar a un subproceso limpio para evitar 'asyncio.run() cannot be called from a running event loop'
    if loop is not None and loop.is_running():
        import tempfile
        with tempfile.NamedTemporaryFile('w+', suffix='.json', delete=False) as tf:
            out_file = tf.name

        try:
            cmd = [
                sys.executable,
                os.path.abspath(__file__),
                '--ur', str(default_ur),
                '--onrobot', str(default_onrobot),
                '--sim', 'true' if default_sim else 'false',
                '--env', str(default_env),
                '--rviz', 'true' if default_rviz else 'false',
                '--output', out_file
            ]
            proc = subprocess.run(cmd)
            if proc.returncode == 0 and os.path.exists(out_file) and os.path.getsize(out_file) > 0:
                with open(out_file, 'r') as f:
                    return json.load(f)
            return {'cancelled': True}
        finally:
            if os.path.exists(out_file):
                try:
                    os.remove(out_file)
                except OSError:
                    pass

    app = RobotSelectorTUI(
        default_ur=default_ur,
        default_onrobot=default_onrobot,
        default_sim=default_sim,
        default_env=default_env,
        default_rviz=default_rviz
    )
    return app.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="TUI Selector para robots UR y pinzas OnRobot")
    parser.add_argument("--ur", default="ur5e", help="Robot UR por defecto")
    parser.add_argument("--onrobot", default="2fg7", help="Pinza OnRobot por defecto")
    parser.add_argument("--sim", default="true", help="true para simulación, false para robot real")
    parser.add_argument("--env", default="basic", help="Entorno: basic, left, right")
    parser.add_argument("--rviz", default="true", help="true para lanzar RViz, false en caso contrario")
    parser.add_argument("--output", default=None, help="Archivo JSON para guardar resultado")

    args = parser.parse_args()

    sim_bool = args.sim.lower() in ("true", "1", "yes")
    rviz_bool = args.rviz.lower() in ("true", "1", "yes")

    app = RobotSelectorTUI(
        default_ur=args.ur,
        default_onrobot=args.onrobot,
        default_sim=sim_bool,
        default_env=args.env,
        default_rviz=rviz_bool
    )
    res = app.run()

    if args.output and res:
        with open(args.output, "w") as f:
            json.dump(res, f)

    if not res or res.get("cancelled", False):
        sys.exit(1)
    sys.exit(0)
