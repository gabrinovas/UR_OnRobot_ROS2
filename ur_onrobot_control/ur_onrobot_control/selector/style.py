# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Appearance of the selector (Textual CSS)."""

STYLE = """
Screen { background: #1e1e2e; color: #cdd6f4; overflow-y: auto; }
Header { background: #313244; color: #89b4fa; text-style: bold; }
#main-container { padding: 0 1; height: auto; overflow-y: auto; }
.panel { background: #181825; border: solid #45475a; padding: 0 1; margin: 0; height: auto; }
#network-panel { height: 3; margin-bottom: 1; align: left middle; }
.panel-title { color: #f9e2af; text-style: bold; margin: 0; }
.status-ok { color: #a6e3a1; text-style: bold; }
.status-err { color: #f38ba8; text-style: bold; }
#columns-container { height: auto; margin-top: 0; }
#columns-container > .panel { width: 1fr; margin-right: 1; }
#columns-container > .panel:last-child { margin-right: 0; }
RadioSet { background: transparent; border: none; padding: 0; }
RadioButton { background: transparent; padding: 0; height: 1; }
RadioButton:focus { color: #89b4fa; }
#buttons-bar { margin-top: 1; height: 3; align: center middle; }
Button { margin: 0 2; min-width: 16; }
#btn-launch { background: #a6e3a1; color: #11111b; text-style: bold; }
#btn-cancel { background: #f38ba8; color: #11111b; text-style: bold; }
"""
