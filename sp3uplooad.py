""" Module: sp3upload
Responsibilities:
  parsing all command line args
  validation of args
  loading data from yaml file
  setup logger, if specified
  call asyncio.run(hub_communication(cmdline_args, data_from_yaml) (delegate hub communication)

Purpose:
This utility supports 2 basic sets of functionality:
  1. Single mode --> upload and possibly run a single py file, supplied by the command line
  2. Batch mode --> upload all files for a project, sets up the imports & possibly run a list of slots
  This 2nd mode takes a project.yaml file as the argument and loads data for entire project

WARNING: This program will blow away the current contents of any slots that it is directed to touch,
as specified by the --slot or in the project.yaml.

TODO: It also supports some basic command line tool args such as logging flags.

"""

import yaml
import argparse
from src.sp3_connector import connect

parser = argparse.ArgumentParser(
    description="Simple Upload & Run for Python on Spike Prime v3 (sp3)."
)

# Arguments for single mode
parser.add_argument(
    "--upload_path",
    help="Path to a single py file that will be uploaded to the slot specified by --slot",
)

parser.add_argument(
    "--slot",
    help="Destination slot for the --upload_path py file.",
    type=int,
    default=0,
)

args = vars(parser.parse_args())

print(args)

# connect()
