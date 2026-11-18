# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Terminal window to choose how to launch a cell: simulated or real, cell, gripper, RViz."""

from concurrent.futures import ThreadPoolExecutor

from textual.app import App
from textual.containers import Horizontal, Vertical
from textual.widgets import Button, Footer, Header, Label, RadioButton, RadioSet, Static
from ur_onrobot_control.selector.style import STYLE
from ur_workspace_description.robot_choice import ping


class CellSelector(App):
    """
    Four columns of options, one choice in each, and the robots that answer on the network.

    options: {'cells': {name: [robot addresses]}, 'grippers': [names], 'addresses_problem': why
    the addresses of the robots are not known ('' when they are)}
    chosen: the values selected at start, with the keys of the result.
    The result is a dict with use_fake_hardware, cell, onrobot_type and launch_rviz, or None when
    the window is cancelled.
    """

    CSS = STYLE
    BINDINGS = [
        ('q', 'cancel', 'Quit'), ('escape', 'cancel', 'Cancel'), ('enter', 'submit', 'Launch')]

    def __init__(self, options, chosen):
        super().__init__()
        self.options = options
        self.chosen = chosen

    def column(self, title, name, choices, selected):
        """Return a panel with one radio button per choice; choices: {value: label}."""
        buttons = [RadioButton(label, value=(value == selected), id=f'{name}-{value}')
                   for value, label in choices.items()]
        return Vertical(Label(title, classes='panel-title'), RadioSet(*buttons, id=name),
                        classes='panel')

    def compose(self):
        yield Header(show_clock=True)
        with Vertical(id='main-container'):
            with Horizontal(classes='panel', id='network-panel'):
                yield Static('Robots on the network:  ')
                for cell in self.options['cells']:
                    yield Static(f'{cell}: checking...   ', id=f'status-{cell}')
            with Horizontal(id='columns-container'):
                yield self.column('Mode', 'mode', {'true': 'Simulation', 'false': 'Real robot'},
                                  self.chosen['use_fake_hardware'])
                yield self.column('Cell', 'cell', {cell: cell for cell in self.options['cells']},
                                  self.chosen['cell'])
                yield self.column('Gripper', 'gripper',
                                  {gripper: gripper for gripper in self.options['grippers']},
                                  self.chosen['onrobot_type'])
                yield self.column('RViz', 'rviz', {'true': 'Start RViz', 'false': 'No RViz'},
                                  self.chosen['launch_rviz'])
            with Horizontal(id='buttons-bar'):
                yield Button('LAUNCH', id='btn-launch', variant='success')
                yield Button('CANCEL', id='btn-cancel', variant='error')
        yield Footer()

    def on_mount(self):
        self.title = 'UR + OnRobot launcher'
        if self.options['addresses_problem']:
            self.notify(self.options['addresses_problem'], title='Simulation only', timeout=20)
        cells = self.options['cells']
        asked = sorted({address for addresses in cells.values() for address in addresses})
        with ThreadPoolExecutor(max_workers=max(len(asked), 1)) as pool:
            up = dict(zip(asked, pool.map(ping, asked)))
        for cell, addresses in cells.items():
            alive = [address for address in addresses if up[address]]
            state = f"on ({', '.join(alive)})" if alive else 'off' if addresses else 'no address'
            label = self.query_one(f'#status-{cell}', Static)
            label.update(f'{cell}: {state}   ')
            label.set_class(bool(alive), 'status-ok')
            label.set_class(not alive, 'status-err')

    def selected(self, name):
        """Return the value of the pressed button of a column."""
        pressed = self.query_one(f'#{name}', RadioSet).pressed_button
        return pressed.id[len(name) + 1:]

    def on_button_pressed(self, event):
        if event.button.id == 'btn-launch':
            self.action_submit()
        else:
            self.action_cancel()

    def action_submit(self):
        self.exit(result={
            'use_fake_hardware': self.selected('mode'),
            'cell': self.selected('cell'),
            'onrobot_type': self.selected('gripper'),
            'launch_rviz': self.selected('rviz'),
        })

    def action_cancel(self):
        self.exit(result=None)
