#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
selector_tui.py - Interfaz TUI interactiva con Textual para seleccionar la configuración de lanzamiento
de robots UR y pinzas OnRobot (Física o Simulación).
"""

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
    }
    Header {
        background: #313244;
        color: #89b4fa;
        text-style: bold;
    }
    #main-container {
        padding: 1 2;
        height: 100%;
    }
    .panel {
        background: #181825;
        border: round #45475a;
        padding: 1 2;
        margin-bottom: 1;
    }
    .panel-title {
        color: #f9e2af;
        text-style: bold;
        margin-bottom: 1;
    }
    .status-ok {
        color: #a6e3a1;
        text-style: bold;
    }
    .status-err {
        color: #f38ba8;
        text-style: bold;
    }
    RadioSet {
        background: transparent;
        border: none;
    }
    RadioButton {
        background: transparent;
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

    def __init__(self, default_ur: str = "ur5e", default_onrobot: str = "2fg7"):
        super().__init__()
        self.default_ur = default_ur
        self.default_onrobot = default_onrobot
        self.result_config: Optional[Dict[str, Any]] = None

        # IPs de robots físicos
        self.ips = {
            'left': '192.168.1.105',
            'right': '192.168.1.101'
        }
        self.detected = {}

    def on_mount(self) -> None:
        self.title = "🤖 APERTA / UR OnRobot Launcher (ROS 2 Humble)"
        # Escaneo inicial de red en hilo rápido
        for side, ip in self.ips.items():
            ok = ping_ip(ip)
            self.detected[side] = ok
            label = self.query_one(f"#status-{side}", Static)
            if ok:
                label.update(f"🟢 [b]DETECTADO[/b] ({ip})")
                label.set_class(True, "status-ok")
                label.set_class(False, "status-err")
            else:
                label.update(f"🔴 [dim]No disponible[/dim] ({ip})")
                label.set_class(False, "status-ok")
                label.set_class(True, "status-err")

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Vertical(id="main-container"):
            # 1. Panel de Estado de Red
            with Vertical(classes="panel"):
                yield Label("📡 Estado de Red de Robots Físicos", classes="panel-title")
                with Horizontal():
                    yield Static("• Robot LEFT:  ", classes="bold")
                    yield Static("🔍 Verificando...", id="status-left")
                with Horizontal():
                    yield Static("• Robot RIGHT: ", classes="bold")
                    yield Static("🔍 Verificando...", id="status-right")

            # 2. Configuración en columnas
            with Horizontal():
                # Modo de Ejecución
                with Vertical(classes="panel", id="col-mode"):
                    yield Label("⚙️  Modo de Ejecución", classes="panel-title")
                    with RadioSet(id="mode-radios"):
                        yield RadioButton("🎮 Simulación (Fake HW)", value=True, id="mode-sim")
                        yield RadioButton("🤖 Robot Físico LEFT", id="mode-real-left")
                        yield RadioButton("🤖 Robot Físico RIGHT", id="mode-real-right")

                # Entorno
                with Vertical(classes="panel", id="col-env"):
                    yield Label("🏭 Entorno de Celda", classes="panel-title")
                    with RadioSet(id="env-radios"):
                        yield RadioButton("Básico (Solo robot)", value=True, id="env-basic")
                        yield RadioButton("Izquierdo (Mesa 1 + Cinta)", id="env-left")
                        yield RadioButton("Derecho (Mesa 2 + Cinta)", id="env-right")

                # Gripper
                with Vertical(classes="panel", id="col-gripper"):
                    yield Label("🦾 Efector OnRobot", classes="panel-title")
                    with RadioSet(id="gripper-radios"):
                        yield RadioButton("2FG7 (Pinza Paralela)", value=(self.default_onrobot == "2fg7"), id="grip-2fg7")
                        yield RadioButton("3FG15 (Pinza 3 Dedos)", value=(self.default_onrobot == "3fg15"), id="grip-3fg15")
                        yield RadioButton("VGC10 (Vacío Doble Canal)", value=(self.default_onrobot == "vgc10"), id="grip-vgc10")

            # 3. Opciones y Botones
            with Horizontal(classes="panel"):
                yield Checkbox("Abrir visualizador RViz2", value=True, id="chk-rviz")

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

        # 3. RViz
        launch_rviz = "true" if self.query_one("#chk-rviz", Checkbox).value else "false"

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


def run_tui(default_ur: str = "ur5e", default_onrobot: str = "2fg7") -> Optional[Dict[str, Any]]:
    """Ejecuta la TUI de Textual y devuelve la configuración seleccionada."""
    app = RobotSelectorTUI(default_ur=default_ur, default_onrobot=default_onrobot)
    return app.run()


if __name__ == "__main__":
    result = run_tui()
    print("\n[Resultado de TUI]:", result)
