# *************************************************************************
#
# Copyright (c) 2021 Andrei Gramakov. All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# site:    https://agramakov.me
# e-mail:  mail@agramakov.me
#
# *************************************************************************

import pprint


def strf(in_str):
    pp = pprint.PrettyPrinter(indent=4, width=60, depth=30)
    return pp.pformat(in_str)


def list2strf(in_list, cell_size, in_hex=False):
    s = "[ "
    cell = ""
    for i in in_list:
        if in_hex:
            cell = hex(i)[2:]
        else:
            cell = str(i)
        c_l = len(cell)
        if c_l > cell_size:
            cell = cell[:cell_size - 1] + "?"
        if c_l < cell_size:
            cell = " " * (cell_size - c_l) + cell
        s = s + cell + " "
    s = s + "]"
    return s
