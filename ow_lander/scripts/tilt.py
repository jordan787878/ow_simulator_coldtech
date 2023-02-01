#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper

import argparse

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Set a tilt value for the antenna mast."
)
parser.add_argument(
  'tilt', type=float, nargs='?', default=0,
  help="Antenna tilt value in radians [-3.2, 3.2]"
)
args = parser.parse_args()

node_helper.call_single_use_action_client(actions.TiltServer, **vars(args))
