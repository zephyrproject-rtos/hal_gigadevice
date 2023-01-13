# Copyright (c) 2023 YuLong Yao <feilongphone@gmail.com>
#
# SPDX-License-Identifier: Apache-2.0

'''gigadevice.py

Gigadevice west extension for prepare develop environment.'''

from textwrap import dedent            # just for nicer code indentation

from west.commands import WestCommand  # your extension must subclass this
from west import log                   # use this for user output

from pathlib import Path
import yaml
import os

REPO_ROOT = Path(__file__).absolute().parents[2]
"""Repository root (used for input/output default folders)."""


class Install:
    def __init__(self, config=REPO_ROOT / 'zephyr' / 'west' / 'gigadevice.yml'):
        self.config = self.parse_config(config)
        self.pyocd_install()

    def parse_config(self, config):
        with open(config, 'r') as f:
            return yaml.safe_load(f)

    def pyocd_install(self):
        for pyocd_packages in self.config['pyocd-packages']:
            self.pyocd_install_one(pyocd_packages)

    def pyocd_install_one(self, package):
        print(package)
        if 'url' in package:
            self.pyocd_install_one_from_url(package)
        else:
            self.pyocd_install_one_from_mdk(package)

    def pyocd_install_one_from_mdk(self, package):
        # run shell command: pyocd pack install package['pack']
        os.system('echo pyocd pack install ' + package['pack'])
        pass

    def pyocd_install_one_from_url(self, package):
        file_name = '%s.rar' % package['pack']
        # Download from url to tmp folder
        log.inf('download', package['url'], 'to', file_name)
        # unrar file to tmp folder
        os.system(f'echo unrar x ${file_name}')
        # find pack file in tmp folder
        # check pack file hash value
        # storage pack file to support folder
        pass


class Gigadevice(WestCommand):
    def __init__(self):
        super().__init__(
            'gigadevice',  # gets stored as self.name
            'Gigadevice tools for west framework',  # self.help
            # self.description:
            dedent('''
            Gigadevice tools for west framework

            Prepare develop environment for Gigadevice MCU.'''))

    def do_add_parser(self, parser_adder):
        # This is a bit of boilerplate, which allows you full control over the
        # type of argparse handling you want. The "parser_adder" argument is
        # the return value of an argparse.ArgumentParser.add_subparsers() call.
        parser = parser_adder.add_parser(self.name,
                                         help=self.help,
                                         description=self.description)

        # Add some example options using the standard argparse module API.
        parser.add_argument('command', help='command to run')

        return parser           # gets stored as self.parser

    def do_run(self, args, unknown_args):
        # This gets called when the user runs the command, e.g.:
        #
        #   $ west my-command-name -o FOO BAR
        #   --optional is FOO
        #   required is BAR
        match args.command:
            case 'install':
                Install()
            case _:
                log.inf('command is', args.command, REPO_ROOT)
