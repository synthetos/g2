import re
import sys

STATES = {
    0 : 'EMPTY',
    1 : 'PLANNING',
    2 : 'QUEUED',
    3 : 'PENDING',
    4 : 'RUNNING'
}

TUMBLER = ['r', 'w', 'q']


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
            pool[current_buffer] = {}
            if current_buffer == pool['r']:
                pool[current_buffer]['is_r'] = True
            if current_buffer == pool['w']:
                pool[current_buffer]['is_w'] = True
            if current_buffer == pool['q']:
                pool[current_buffer]['is_q'] = True
        else:
            match = re.search('([a-zA-Z_]+)\s=\s((?:0x[0-9a-fA-F]+|[0-9.]+|[A-Z_]+)|true|false)', line)
            if match:
                key, val = match.groups()
                if key == 'pv':
                    pv = int(val, 16)
                    pool[current_buffer]['pv'] = pv
                if key == 'locked':
                    pool[current_buffer]['locked'] = True if val == 'true' else False
                elif key == 'nx':
                    nv = int(val, 16)
                    pool[current_buffer]['nx'] = nv
                elif key == 'buffer_state':
                    pool[current_buffer]['buffer_state'] = val
                elif key == 'real_move_time':
                    pool[current_buffer]['real_move_time'] = val
    return pool

def check_integrity(pool):
    key = pool.keys()[0]
    buffers = set()
    count = 0;
    while True:
        if key in buffers:
            break
        buffers.add(key)
        key = pool[key]['nx']
        count += 1

    if count != 28:
        raise "Buffer pool integrity is bad."

def check_pool(filename):
    pool = load_pool(filename)
    check_integrity(pool)
    return pool

def print_pool(pool):
    for key in sorted(pool.keys()):
        buffer = pool[key]
        if isinstance(buffer, dict):
            flags = ['r' if 'is_r' in buffer else None, 'w' if 'is_w' in buffer else None, 'q' if 'is_q' in buffer else None]
            if any(flags):
                pointer = '(%s)' % (','.join([flag for flag in flags if flag]))
            else:
                pointer = ''

            print '0x%08x : %-20s %-8s %-6s %04.2f' % (key, buffer['buffer_state'].strip(), 'locked' if buffer['locked'] else 'unlocked', pointer, float(buffer['real_move_time'])*60000)

if __name__ == "__main__":
    pool = check_pool(sys.argv[1])
    print_pool(pool)
