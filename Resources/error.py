import re
import json

codes = {}
with open('../TinyG2/error.h') as fp:
    for line in fp.readlines():
        match = re.match(r'\s*#define\s*(\w+)\s*([0-9]+)\s*(?://([\w\s\'0-9]*))?', line)
        if match:
            name, value, description = match.groups()
            description = description.strip() if description else ''
            codes[value] = (name, description)
        else:
            match = re.match(r'\s*static const char stat_\([0-9]+\)\[\] PROGMEM = \"(\\"|[^"])+\"', line)
            if match:
                value, description = match.groups()
                description = description.strip() if description else ''
                codes[value][1] = description

with open('g2_errors.json', 'w') as fp:
    json.dump(codes, fp, indent=3)

print 'Error codes written to g2_errors.json.'
