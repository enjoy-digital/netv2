#!/usr/bin/env python3

import os
import sys
import argparse
import datetime


def _get_args():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
        description="""\
NeTV2-SoC - based on LiteX.

This program builds and/or loads NeTV2-SoC components.
One or several actions can be specified:

clean           delete previous build(s).
build           build FPGA bitstream.
load            load FPGA bitstream (in sram).
flash           flash FPGA bitstream (in flash).
release         create a release.
""")
    parser.add_argument("action", nargs="+", help="specify an action")
    return parser.parse_args()


def _prog_fpga_sram(filename):
    from litex.build.openocd import OpenOCD
    prog = OpenOCD("openocd/openocd.cfg")
    prog.load_bitstream(filename)


def _prog_fpga_flash(filename):
    from litex.build.openocd import OpenOCD
    prog = OpenOCD("openocd/openocd.cfg",
                   flash_proxy_basename="openocd/bscan_spi_xc7a35t.bit")
    prog.set_flash_proxy_dir(".")
    prog.flash(0, filename)

def _make_release(name="netv2"):
    today = datetime.date.today()
    today = str(today).replace("-", "_")
    output_dir = "release_{}".format(today)
    if not os.path.exists(output_dir ):
        os.makedirs(output_dir)
    os.system("git archive master | bzip2 > {}/{}_{}.tar.bz2".format(output_dir, name, today))
    os.system("cp build/gateware/{}.bit {}/{}_{}.bit".format(name, output_dir, name, today))
    os.system("cp build/gateware/{}.bin {}/{}_{}.bin".format(name, output_dir, name, today))
    os.system("tar -zcvf {}.tar.gz {}".format(output_dir, output_dir))


if __name__ == "__main__":
    args = _get_args()

    # decode actions
    action_list = ["clean", "build", "load", "flash", "release"]
    actions = {k: False for k in action_list}
    for action in args.action:
        if action in actions:
            actions[action] = True
        else:
            print("Unknown action: {}. Valid actions are:".format(action))
            for a in action_list:
                print("  "+a)
            sys.exit(1)


    print("""
      _  __  _______   _____     ____     _____
     / |/ /_/_  __/ | / /_  |___/ __/__  / ___/
    /    / -_) /  | |/ / __/___/\ \/ _ \/ /__
   /_/|_/\__/_/   |___/____/  /___/\___/\___/

        Copyright 2016-2018 / EnjoyDigital

           NeTV2 SoC based on LiteX
""")

    # dependencies
    if actions["build"]:
        actions["clean"] = True

    # actions
    if actions["clean"]:
        print("[clean]...")
        os.system("rm -rf build/*")
        os.system("rm -rf release*")
        os.system("rm -rf vivado*")

    if actions["build"]:
        print("[build]...")
        os.system("python3 netv2.py")

    if actions["load"]:
        print("[load]...")
        _prog_fpga_sram("build/gateware/netv2.bit")

    if actions["flash"]:
        print("[flash]...")
        _prog_fpga_flash("build/gateware/netv2.bin")

    if actions["release"]:
        print("[release]...")
        _make_release()
