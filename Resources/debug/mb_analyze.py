import re
import sys

STATES = {
    0 : 'EMPTY',
    1 : 'PLANNING',
    2 : 'QUEUED',
    3 : 'PENDING',
    4 : 'RUNNING'
}

TUMBLER = ['r', 'w', 'p', 'c']


def load_pool(filename):
    tumbler_pos = 0
    current_buffer = 0
    with open(filename) as mb:
        lines = mb.readlines()
    pool = {}
    for line in lines:
        buf = re.search('\(mpBuf_t\s\*\)\s(0x[0-9a-fA-F]+)', line)
        if buf:
            current_buffer = int(buf.groups()[0], 16)
            pool[TUMBLER[tumbler_pos]] = current_buffer
            tumbler_pos += 1

        buf = re.search('\(mpBuffer\s\*\)\s(0x[0-9a-fA-F]+)', line)
        if buf:
            current_buffer = int(buf.groups()[0], 16)
            pool[current_buffer] = {'move_time': 0}
            if current_buffer == pool['r']:
                pool[current_buffer]['is_r'] = True
            if current_buffer == pool['w']:
                pool[current_buffer]['is_w'] = True
            if current_buffer == pool['p']:
                pool[current_buffer]['is_p'] = True
            if current_buffer == pool['c']:
                pool[current_buffer]['is_c'] = True
        else:
            match = re.search('([a-zA-Z_]+)\s=\s((?:0x[0-9a-fA-F]+|[0-9.]+|[A-Z_]+)|true|false)', line)
            if match:
                key, val = match.groups()
                if key == 'pv':
                    pv = int(val, 16)
                    pool[current_buffer]['pv'] = pv
                # if key == 'locked':
                #     pool[current_buffer]['locked'] = True if val == 'true' else False
                elif key == 'nx':
                    nv = int(val, 16)
                    pool[current_buffer]['nx'] = nv
                elif key == 'buffer_state':
                    pool[current_buffer]['buffer_state'] = val
                elif key == 'linenum':
                    pool[current_buffer]['linenum'] = val
                elif key == 'buffer_number':
                    pool[current_buffer]['buffer_number'] = val
                elif key == 'junction_vmax':
                    pool[current_buffer]['junction_vmax'] = val
                elif key == 'cruise_velocity':
                    pool[current_buffer]['cruise_velocity'] = val
                elif key == 'exit_velocity':
                    pool[current_buffer]['exit_velocity'] = val
                elif key == 'length':
                    pool[current_buffer]['length'] = val
                elif key == 'move_time':
                    pool[current_buffer]['move_time'] = val
                elif key == 'plannable':
                    pool[current_buffer]['plannable'] = val
                elif key == 'hint':
                    pool[current_buffer]['hint'] = val
    return pool

def check_integrity(pool):
    key = pool.keys()[0]
    buffers = set()
    count = 0
    while True:
        if key in buffers:
            break
        buffers.add(key)
        key = pool[key]['nx']
        count += 1

    if count != 48:
        raise Exception("Buffer pool integrity is bad.")

def check_pool(filename):
    pool = load_pool(filename)
    check_integrity(pool)
    return pool

def print_pool(pool):
    for key in sorted(pool.keys()):
        buffer = pool[key]
        if isinstance(buffer, dict):
            flags = [
                'r' if 'is_r' in buffer else None,
                'w' if 'is_w' in buffer else None,
                'p' if 'is_p' in buffer else None,
                'c' if 'is_c' in buffer else None
                ]
            if any(flags):
                pointer = '(%s)' % (','.join([flag for flag in flags if flag]))
            else:
                pointer = ''

            print '0x%08x [%02d] : N%04d %-22s %-8s L% 8.2f Ti% 8.2f C% 10.2f X% 10.2f J% 10.2f %5s %10s' % (
                key,
                float(buffer['buffer_number']),
                float(buffer['linenum']),
                buffer['buffer_state'].strip(),
                pointer,
                float(buffer['length']),
                float(buffer['move_time']),
                float(buffer['cruise_velocity']),
                float(buffer['exit_velocity']),
                float(buffer['junction_vmax']),
                buffer['plannable'],
                buffer['hint']
                )

if __name__ == "__main__":
    pool = check_pool(sys.argv[1])
    print_pool(pool)
