import re
import json

codes = {}
with open('../TinyG2/tinyg2.h') as fp:
    for line in fp.readlines():
        match = re.match('\s*#define\s*(\w+)\s*([0-9]+)\s*(?://([\w\s\'0-9]*))?', line)
        if match:
            name, value, description = match.groups()
            description = description.strip() if description else ''
            codes[value] = (name, description)

with open('g2_errors.json', 'w') as fp:
    json.dump(codes, fp, indent=3)

print 'Error codes written to g2_errors.json.'
