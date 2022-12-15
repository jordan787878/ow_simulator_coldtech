#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper

import argparse

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Trigger both mast camera to capture a photograph."
)
parser.add_argument(
  'exposure', type=float, nargs='?', default=-1,
  help="Camera exposure in seconds. If <= 0 the previous exposure setting " \
       "will be used."
)
args = parser.parse_args()

node_helper.call_single_use_action_client(
  actions.CameraCaptureServer, **vars(args)
)
