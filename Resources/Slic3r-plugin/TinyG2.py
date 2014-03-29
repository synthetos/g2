#!/usr/bin/env python
import re
import sys
import argparse

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--eto', nargs=1, default='A')
    parser.add_argument('filenames', nargs='+')
    args = parser.parse_args()
    return args

def process(filename, eto):
  with open(filename, "r") as f:
    lines = f.readlines()

  with open(filename, "w") as f:
    for line in lines:
      line_split = list(line.partition(";"))

      # Fix E to A or B
      line_split[0] = re.sub(r"^([^;\(\)]+)[Ee]([-.0-9]+)", "\\1"+eto+"\\2", line_split[0])

      # Fix assinine G0 F...
      line_split[0] = re.sub(r"^[Gg]0 *(.*?)([Ff][-.0-9]+ *)(.*)$", "G0 \\1\\3", line_split[0])

      # Retraction should be using G0, not G1 with a feedrate
      match = re.search(r"^[Gg]1 *((?:[FfAaBb][-.0-9]+ *?)+)( *;.*)?$", line_split[0])
      if (match != None):
        tail = match.group(2)
        if (tail == None):
          tail = ""
        new_line = ["G0"]
        for option in re.findall(r"([FfAaBb])([-.0-9]+) *", match.group(1)):
          if option[0] in ['a', 'A', 'b', 'B']:
            new_line.append("".join(option))
        line_split[0] = " ".join(new_line) + tail + "\n"

      # G92 -> G28.3
      line_split[0] = re.sub(r"^[Gg]92([^0-9.].*)$", "G28.3\\1", line_split[0])

      # Disable fan controls
      line_split[0] = re.sub(r"([Mm]107)\b", ";\\1", line_split[0])

      # Disable temperature controls
      line_split[0] = re.sub(r"([Mm]10[469])\b", ";\\1", line_split[0])

      # Remove errant T[01]s
      line_split[0] = re.sub(r"^([Tt][01])\b", ";\\1", line_split[0])

      line = "".join(line_split)
      f.write(line)


def main(argv):
  args = parse_args()

  for filename in args.filenames:
    process(filename, args.eto)

if __name__ == '__main__':
  sys.exit(main(sys.argv))