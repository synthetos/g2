#!/usr/bin/env python
# coding=utf-8

import argparse
import sys
from re import match as regexp_match
from struct import unpack
from os import SEEK_END, SEEK_SET
from time import sleep

from serial import Serial
from serial.tools.list_ports import comports as list_ports

class ArgException(Exception):
    pass

_verbose = False

CHIP_TABLE = {
    # SAM3X
    0x286e0a60 : {
        'name':           "ATSAM3X8H",
        'address':        0x80000L,
        'pages':          2048,
        'size':           256,
        'planes':         2,
        'lock_regions':   32,
        'user':           0x20001000L,
        'stack':          0x20010000L,
        'registers_addr': 0x400e0a00L,
        'can_brownout':   False
    },
    0x285e0a60 : {
        'name':           "ATSAM3X8E",
        'address':        0x80000L,
        'pages':          2048,
        'size':           256,
        'planes':         2,
        'lock_regions':   32,
        'user':           0x20001000L,
        'stack':          0x20010000L,
        'registers_addr': 0x400e0a00L,
        'can_brownout':   False
    },
    0x284e0a60 : {
        'name':           "ATSAM3X8C",
        'address':        0x80000L,
        'pages':          2048,
        'size':           256,
        'planes':         2,
        'lock_regions':   32,
        'user':           0x20001000L,
        'stack':          0x20010000L,
        'registers_addr': 0x400e0a00L,
        'can_brownout':   False
    }
}

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port' , '-p',
                        required=False,
                        action='store',
                        help='Port to upload to.')

    parser.add_argument('--file' , '-f',
                        required=True,
                        action='store',
                        help='File to upload.')

    parser.add_argument('--verbose', '-v',
                        action='store_true',
                        default=False,
                        help='increase verbosity to DEBUG.')

    args = parser.parse_args()

    return args

class SamBA:
    def __init__(self, port_path):
        """Create a SamBA object"""

        self._port_path = port_path
        self._port = Serial(port=self._port_path, timeout=0.1)

        # Make sure we're in binary mode
        self._write("N#")
        self._read(2)

    def _read(self, length):
        return self._port.read(length)

    def _write(self, data):
        self._port.write(data)

    def _flush(self):
        self._port.flush()

    def chipId(self):
        # Read the ARM reset vector
        vector = self.readWord(0x0);

        # If the vector is a ARM7TDMI branch,
        # then assume Atmel SAM7 registers
        if (vector & 0xff000000) == 0xea000000:
            cid = self.readWord(0xfffff240)

        # Else use the Atmel SAM3 registers
        else:
            print "It's a SAM3"
            cid = self.readWord(0x400e0740);
            if cid == 0:
                cid = self.readWord(0x400e0940);

        return cid

    def readWord(self, address):
        self._write("w%08X,4#" % (address))
        value = self._read(4)
        value = unpack("<L", value)[0]
        
        if _verbose:
            print "readWord(addr=%#08x)=%#08x" % (address, value)

        return value

    def writeWord(self, address, value):
        self._write("W%08X,%08X#" % (address, value))

        if _verbose:
            print "writeWord(addr=%#08x, %#08x)" % (address, value)

        self._flush()

    def go(self, address):
        self._write("G%08X#" % (address))

        if _verbose:
            print "go(addr=%#08x)" % (address)

    def write(self, address, data):
        if _verbose:
            print "write(addr=%#08x, count=%3i)" % (address, len(data))

        self._write("S%08X,%08X#" % (address, len(data)))
        self._flush()
        amt_written = self._write(data)

class WordCopyApplet:
    dst_ptr = 0x00000028L
    reset_ptr = 0x00000024L
    src_ptr = 0x0000002cL
    stack_ptr = 0x00000020L
    start_ptr = 0x00000000L
    words_ptr = 0x00000030L
    code = b'\x09\x48\x0a\x49\x0a\x4a\x02\xe0\x08\xc9\x08\xc0\x01\x3a\x00' \
           b'\x2a\xfa\xd1\x04\x48\x00\x28\x01\xd1\x01\x48\x85\x46\x70\x47' \
           b'\xc0\x46\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00' \
           b'\x00\x00\x00\x00\x00\x00\x00'

    def __init__(self, samba, base_address):
        self.samba = samba
        self.base_addr = base_address
        self.samba.write(base_address, self.code)

    def setStack(self, stack):
        if _verbose:
            print "setStack(%#08x)" % stack
        self.samba.writeWord(self.base_addr + self.stack_ptr, stack)

    def setWords(self, words):
        if _verbose:
            print "setWords(%#08x)" % words
        self.samba.writeWord(self.base_addr + self.words_ptr, words)

    def setSource(self, addr):
        if _verbose:
            print "setSource(%#08x)" % addr
        self.samba.writeWord(self.base_addr + self.src_ptr, addr)

    def setDestination(self, addr):
        if _verbose:
            print "setDestination(%#08x)" % addr
        self.samba.writeWord(self.base_addr + self.dst_ptr, addr)

    def run(self):
        if _verbose:
            print "run()"
        self.samba.writeWord(self.base_addr + self.reset_ptr,
                             self.base_addr + self.start_ptr+1)
        self.samba.go(self.base_addr + self.stack_ptr)

    def size(self):
        return len(self.code)


class EEFCFlash:
    EEFC_KEY       = 0x5a

    EEFC0_FMR      = 0x00
    EEFC0_FCR      = 0x04
    EEFC0_FSR      = 0x08
    EEFC0_FRR      = 0x0C

    EEFC1_FMR      = 0x200
    EEFC1_FCR      = 0x204
    EEFC1_FSR      = 0x208
    EEFC1_FRR      = 0x20C

    EEFC_FCMD_GETD = 0x0
    EEFC_FCMD_WP   = 0x1
    EEFC_FCMD_WPL  = 0x2
    EEFC_FCMD_EWP  = 0x3
    EEFC_FCMD_EWPL = 0x4
    EEFC_FCMD_EA   = 0x5
    EEFC_FCMD_SLB  = 0x8
    EEFC_FCMD_CLB  = 0x9
    EEFC_FCMD_GLB  = 0xa
    EEFC_FCMD_SGPB = 0xb
    EEFC_FCMD_CGPB = 0xc
    EEFC_FCMD_GGPB = 0xd

    def __init__(self, samba, chip_data):
        self.samba = samba
        self.chip_data = chip_data
        
        self.registers_address = self.chip_data["registers_addr"]
        
        self.applet = WordCopyApplet(samba, self.chip_data["user"])
        self.applet.setWords(self.chip_data["size"] / 4)
        self.applet.setStack(self.chip_data["stack"])
        
        # SAM3 Errata (FWS must be 6)
        self.writeRegister(self.EEFC0_FMR, 0x6 << 8)
        if self.chip_data["planes"] == 2:
            self.writeRegister(self.EEFC1_FMR, 0x6 << 8)
        
        self.page_buffer_a = self.chip_data["user"] + self.applet.size()
        self.page_buffer_b = self.page_buffer_a + self.chip_data["size"]

        self.on_buffer = self.page_buffer_a

    def writeRegister(self, offset, value):
        if _verbose:
            print "writeRegister(%#08x, %#08x)" % (offset, value)
        self.samba.writeWord(offset + self.registers_address, value)

    def readRegister(self, offset):
        return self.samba.readWord(offset + self.registers_address)

    def write(self, filename):
        # Open the file
        with open(filename, "rb") as f:
            # Get the file size, by seeking to the end, getting the position
            # then seekig back to the front.
            f.seek(0, SEEK_END)
            file_size = f.tell()
            f.seek(0, SEEK_SET)

            # calculate number of pages needed
            page_size = self.chip_data['size']
            page_count = (file_size + page_size - 1) / page_size;
            if (page_count > self.chip_data['pages']):
                raise Exception("File '%s' is too big for this chip: %i > %i." %
                                    (filename,
                                     file_size,
                                     page_size * self.chip_data['pages']))

            # loop for pages:
            for page_num in range(0, page_count):
                data = f.read(page_size)
                amt_read = len(data)
                done = False

                if amt_read < page_size:
                    # Pad the page with nulls
                    data += '\0' * (page_size - amt_read)
                    done = True

                # load the buffer
                self.samba.write(self.on_buffer, data)

                # Tell it to write the page
                self.writePage(page_num)

                # Update to the correct buffer
                if (self.on_buffer == self.page_buffer_a):
                    self.on_buffer = self.page_buffer_b
                else:
                    self.on_buffer = self.page_buffer_a
            
                if done:
                    # this should be redundant
                    break

    def writePage(self, page):
        if page >= self.chip_data["pages"]:
            raise Excpetion("writePage page %i is out of range (> %i)" %
                            (page, self.chip_data["pages"]))

        page_size = self.chip_data['size']
        page_addr = self.chip_data['address'] + page * page_size
        self.applet.setDestination(page_addr)
        self.applet.setSource(self.on_buffer)

        self.waitFSR()
        self.applet.run()

        if (self.chip_data['planes'] == 2 and
            page >= self.chip_data['pages'] / 2):
            self.writeFCR1(self.EEFC_FCMD_EWP,
                           long(page - self.chip_data['pages'] / 2))
        else:
            self.writeFCR0(self.EEFC_FCMD_EWP,
                           long(page))

    def waitFSR(self):
        tries = 0
        fsr1 = 0x1

        while ++tries <= 500:
            fsr0 = self.readRegister(self.EEFC0_FSR)
            if fsr0 & (1 << 2):
                raise Exception("Flash lock error")

            if self.chip_data['planes'] == 2:
                fsr1 = self.readRegister(self.EEFC1_FSR)
                if fsr1 & (1 << 2):
                    raise Exception("Flash lock error")

            if fsr0 & fsr1 & 0x1:
                break

            sleep(0.1)

            if (tries > 500):
                raise Exception("Flash command error")

    def writeFCR0(self, cmd, arg):
        if _verbose:
            print "writeFCR0(%#08x, %#08x)" % (cmd, arg)
        self.writeRegister(self.EEFC0_FCR,
                         ((self.EEFC_KEY << 24L) | (arg << 8L) | cmd))

    def writeFCR1(self, cmd, arg):
        if _verbose:
            print "writeFCR1(%#08x, %#08x)" % (cmd, arg)
        self.writeRegister(self.EEFC1_FCR,
                         ((self.EEFC_KEY << 24L) | (arg << 8L) | cmdL))

    def setBootFlash(self, enable_flash):
        self.waitFSR()
        if enable_flash:
            cmd = self.EEFC_FCMD_SGPB
        else:
            cmd = self.EEFC_FCMD_CGPB
            
        if self.chip_data['can_brownout']:
            value = 3
        else:
            value = 1
        self.writeFCR0(cmd, value)
        self.waitFSR()

        sleep(10)

def main(argv):
    try:
        args = parse_args()
    except ArgException as e:
        print 'Error: %r' % (e)
        return 1

    global _verbose
    _verbose = args.verbose

    if args.port:
        _port_name = args.port
    else:
        port_list = list_ports()
        _port_name = None
        for port_i in port_list:
            port_match = regexp_match(
                    r"USB VID:PID=" \
                    "(?P<VID>[a-zA-Z0-9]{3,4}):(?P<PID>[a-zA-Z0-9]{3,4})",
                    port_i[2])
            if port_match:
                _vid = int(port_match.group("VID"),16)
                _pid = int(port_match.group("PID"),16)
                if _vid == 0x03eb and _pid == 0x6124:
                    print "Found %s" % port_i[0]
                    _port_name = port_i[0]
                    break
        if not _port_name:
            raise Exception("Auto-locate port failed.")

    samba = SamBA(_port_name)
    chipId = samba.chipId()

    if chipId not in CHIP_TABLE:
        raise Exception("Unknow chip ID %#x" % chipId)

    eefc = EEFCFlash(samba, CHIP_TABLE[chipId])
    eefc.write(args.file)
    
    eefc.setBootFlash(True)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))


#EefcFlash(samba, "ATSAM3X8", 0x80000, 2048, 256, 2, 32, 0x20001000, 0x20010000, 0x400e0a00, false);