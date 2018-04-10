# Copyright 2018-present PlatformIO Plus <contact@pioplus.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from platform import system

from platformio.managers.platform import PlatformBase


class Aceinna_imuPlatform(PlatformBase):

    def get_boards(self, id_=None):
        result = PlatformBase.get_boards(self, id_)
        if not result:
            return result
        if id_:
            return self._add_default_debug_tools(result)
        else:
            for key, value in result.items():
                result[key] = self._add_default_debug_tools(result[key])
        return result

    def _add_default_debug_tools(self, board):
        debug = board.manifest.get("debug", {})
        upload_protocols = board.manifest.get("upload", {}).get(
            "protocols", [])
        if "tools" not in debug:
            debug['tools'] = {}

        # BlackMagic, J-Link, ST-Link
        for link in ("blackmagic", "jlink", "stlink"):
            if link not in upload_protocols or link in debug['tools']:
                continue
            if link == "blackmagic":
                debug['tools']['blackmagic'] = {
                    "hwids": [["0x1d50", "0x6018"]],
                    "require_debug_port": True
                }
                continue
            elif link == "jlink":
                assert debug.get("jlink_device"), (
                    "Missed J-Link Device ID for %s" % board.id)
                debug['tools'][link] = {
                    "server": {
                        "arguments": [
                            "-singlerun",
                            "-if", "SWD",
                            "-select", "USB",
                            "-device", debug.get("jlink_device"),
                            "-port", "2331"
                        ],
                        "executable": ("JLinkGDBServerCL.exe"
                                       if system() == "Windows" else
                                       "JLinkGDBServer")
                    },
                    "onboard": link in debug.get("onboard_tools", [])
                }
                continue

            server_args = []
            if link in debug.get("onboard_tools",
                                 []) and debug.get("openocd_board"):
                server_args = [
                    "-f",
                    "scripts/board/%s.cfg" % debug.get("openocd_board")
                ]
            else:
                assert debug.get("openocd_target"), (
                    "Missed target configuration for %s" % board.id)

                server_args = [
                    "-f",
                    "scripts/interface/%s.cfg" % link, "-c",
                    "transport select %s" % ("hla_swd"
                                             if link == "stlink" else "swd"),
                    "-f",
                    "scripts/target/%s.cfg" % debug.get("openocd_target")
                ]

            debug['tools'][link] = {
                "server": {
                    "package": "tool-openocd",
                    "executable": "bin/openocd",
                    "arguments": server_args
                },
                "onboard": link in debug.get("onboard_tools", []),
                "default": link in debug.get("default_tools", []),
                "extra_cmds": (["set $pc=Reset_Handler"]
                               if link == "stlink" else None)
            }

        board.manifest['debug'] = debug
        return board
