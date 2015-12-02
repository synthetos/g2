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
    current_buffer = {}
    with open(filename) as mb:
        lines = mb.readlines()
    pool = {}
    pool_key_lookup = {}
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
            if current_buffer == pool['p']:
                pool[current_buffer]['is_p'] = True
            if current_buffer == pool['c']:
                pool[current_buffer]['is_c'] = True
        else:
            match = re.search('([a-zA-Z_]+[a-zA-Z0-9_]+)\s=\s((?:0x[0-9a-fA-F]+|[0-9.]+(?:e[-+0-9]+)?|[A-Z_]+[a-zA-Z0-9_]+)|true|false)', line)
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
                elif key == 'move_time_ms':
                    pool[current_buffer]['move_time_ms'] = val
                elif key == 'linenum':
                    pool[current_buffer]['linenum'] = val
                elif key == 'buffer_number':
                    pool[current_buffer]['buffer_number'] = val
                    pool_key_lookup[int(val)] = current_buffer
                    print "found buf %d" % (int(val))
                # elif key == 'entry_velocity':
                #     pool[current_buffer]['entry_velocity'] = val
                elif key == 'cruise_velocity':
                    pool[current_buffer]['cruise_velocity'] = val
                elif key == 'exit_velocity':
                    pool[current_buffer]['exit_velocity'] = val
                elif key == 'head_length':
                    pool[current_buffer]['head_length'] = val
                elif key == 'body_length':
                    pool[current_buffer]['body_length'] = val
                elif key == 'tail_length':
                    pool[current_buffer]['tail_length'] = val
                elif key == 'head_time':
                    pool[current_buffer]['head_time'] = val
                elif key == 'body_time':
                    pool[current_buffer]['body_time'] = val
                elif key == 'tail_time':
                    pool[current_buffer]['tail_time'] = val
                elif key == 'move_time':
                    pool[current_buffer]['move_time'] = val
                elif key == 'hint':
                    pool[current_buffer]['hint'] = val
                # elif key == 'zoid_exit':
                #     pool[current_buffer]['zoid_exit'] = val
    return pool, pool_key_lookup

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

    if count != 48:
        raise Exception("Buffer pool integrity is bad.")

# def check_pool(filename):
#     pool = load_pool(filename)
#     check_integrity(pool)
#     return pool

def print_pool_table(pool):
    print 'buffer_number\tlinenum\tbuffer_state\tpointer\tmove_time_ms\tentry_velocity\tcruise_velocity\texit_velocity\thead_length\thead_time\tbody_length\tbody_time\ttail_length\ttail_time\thint\tzoid_exit';
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

            print '%02d\t%04d\t%-22s\t%-8s\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%-14s\t%-24s' % (
                float(buffer['buffer_number']),
                float(buffer['linenum']),
                buffer['buffer_state'].strip(),
                pointer,
                float(buffer['cruise_velocity']),
                float(buffer['exit_velocity']),
                float(buffer['head_length']),
                float(buffer['head_time']) * 60000,
                float(buffer['body_length']),
                float(buffer['body_time']) * 60000,
                float(buffer['tail_length']),
                float(buffer['tail_time']) * 60000,
                buffer['hint'],
                buffer['zoid_exit']
                )


def print_pool(pool, pool_key_lookup):
    print "\t".join(['buffer_number', 'linenum', 'type', 'start_t', 'end_l', 'end_velocity'])
    running_time = 0
    running_length = 0
    for key in sorted(pool_key_lookup.keys()):
        buffer = pool[pool_key_lookup[key]]
        # if (isinstance(buffer, dict)):
                #and buffer['buffer_state'].strip() in ["MP_BUFFER_PREPPED", "MP_BUFFER_PLANNED"]):

        if float(buffer['head_time']) > 0.0:
            print '%d\t%d\t%s\t%f\t%f\t%f' % (
                float(buffer['buffer_number']),
                float(buffer['linenum']),
                'head',
                running_time,
                running_length,
                float(buffer['exit_velocity'])
                )
            running_time += float(buffer['head_time']) * 60000
            running_length += float(buffer['head_length'])

        if float(buffer['body_time']) > 0.0:
            print '%d\t%d\t%s\t%f\t%f\t%f' % (
                float(buffer['buffer_number']),
                float(buffer['linenum']),
                'body',
                running_time,
                running_length,
                float(buffer['exit_velocity'])
                )
            running_time += float(buffer['body_time']) * 60000
            running_length += float(buffer['body_length'])

        if float(buffer['tail_time']) > 0.0:
            print '%d\t%d\t%s\t%f\t%f\t%f' % (
                float(buffer['buffer_number']),
                float(buffer['linenum']),
                'tail',
                running_time,
                running_length,
                float(buffer['exit_velocity'])
                )
            running_time += float(buffer['tail_time']) * 60000
            running_length += float(buffer['tail_length'])

    print '%d\t%d\t%s\t%f\t%f\t%f' % (
        float(buffer['buffer_number']),
        float(buffer['linenum']),
        'end',
        running_time,
        running_length,
        float(buffer['exit_velocity'])
        )

if __name__ == "__main__":
    pool, pool_key_lookup = load_pool(sys.argv[1])
    print_pool(pool, pool_key_lookup)
