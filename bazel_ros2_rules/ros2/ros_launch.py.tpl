import os
import sys

from launch import LaunchService
from launch.frontend import Parser

with open(os.path.join(os.path.dirname(__file__), "{}"), "r") as f:
    re, parser = Parser.load(f)
    ld = parser.parse_description(re)
    ls = LaunchService()
    ls.include_launch_description(ld)
    sys.exit(ls.run())
