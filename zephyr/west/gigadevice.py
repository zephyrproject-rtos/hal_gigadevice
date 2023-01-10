# Copyright (c) 2023 YuLong Yao <feilongphone@gmail.com>
#
# SPDX-License-Identifier: Apache-2.0

'''gigadevice.py

Gigadevice west extension for prepare develop environment.'''

from textwrap import dedent            # just for nicer code indentation

from west.commands import WestCommand  # your extension must subclass this
from west import log                   # use this for user output


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
                log.inf('command is', args.command)
                log.inf('we can install some pack for pyocd')
                log.inf('TODO: RUN: pyocd pack install gd32e103')
            case _:
                log.inf('command is', args.command)
