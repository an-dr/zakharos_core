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

from json import load, loads, dumps, JSONDecodeError
from jsonschema import validate, ValidationError


def _read(in_json: str) -> dict:
    in_json = in_json.strip()
    if in_json[0] == '{':
        json_dict = loads(in_json)
    else:
        with open(in_json) as j:
            json_dict = load(j)  # type: dict
    return json_dict


def string_to_obj(in_str: str) -> list:
    try:
        res = loads(in_str.replace("'", "\""))
    except JSONDecodeError:
        res = []
    return list(res)


def json_to_dict(in_json: str, in_schema: str = None) -> dict:
    """
    Read JSON file or JSON string. If the first symbol in the input params is not '{' - method will use imput argument
    as a file path
    """
    json_dict = _read(in_json)
    if in_schema:
        json_schema = _read(in_schema)
        validate(instance=json_dict, schema=json_schema)
    return json_dict


def dict_to_json_str(in_dict: dict, in_schema: str = None) -> str:
    if in_schema:
        json_schema = _read(in_schema)
        validate(instance=in_dict, schema=json_schema)
    return dumps(in_dict)


if __name__ == '__main__':
    j = json_to_dict(
        "/home/agramakov/Desktop/test.json",
        "/home/agramakov/Files/Code/zakharos_core/src/aliveos_msgs/json/perception-concept-descriptor.json")
    print("Success!")
    print(j)
