import yaml
import argparse

from sp3python.sp3_hub_connector.app import find_hub

parser = argparse.ArgumentParser(
    description="Simple Upload & Run for Python on Spike Prime v3."
)
parser.add_argument(
    "--slots_file", help="Path to slots yaml file.", default="slots.yaml"
)
parser.add_argument(
    "--run_slot",
    help="Slot to run. Will be executed after all slots from the slots file are uploaded.",
    type=int,
)
parser.add_argument(
    "--upload", help="Path to file to upload to slot specified by --slot."
)
parser.add_argument(
    "--slot",
    help="Slot to upload file from --upload. Only used with --upload",
    default=0,
    type=int,
)
args = vars(parser.parse_args())

print(args)

# check for file existence
if args.get("slots_file"):
    print("check here for existence of slots file")

if args.get("upload"):
    print("check here for existence of py file")

# read from yaml portion
import os

print(os.getcwd())
with open("./slots.yaml", "r") as file:
    data = yaml.safe_load(file)

print(data)

slots = data.get("slots")


# initiate contact with the hub
async def connect():
    device = await find_hub()
    return device


for slot in slots:
    print(f"Slot {slot.get('number')} : {slot.get('file')}")
    # here we do the

# remain listening to hub until person disconnects (either ctrl-c or main button on hub)
import asyncio

asyncio.run(connect())
