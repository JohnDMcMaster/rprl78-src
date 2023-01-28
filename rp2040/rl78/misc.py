from .util import tohex, StructStreamer, print_json_pretty
from .proto import try_a1, try_ocd, ProtoANACK, ProtectError
import json
import sys


def decode_sig(buf, hexlify=True, verbose=False):
    """
    RL78/G13
    25.5.5 Description of signature data
    Table 25-9. Signature Data List

    Device code
    3 bytes
    The serial number assigned to the device
    
    Device name
    10 bytes
    Device name (ASCII code)

    Code flash memory area last address
    3 bytes
    Last address of code flash memory area
    (Sent from lower address.
    Example. 00000H to 0FFFFH (64 KB) => FFH, 1FH, 00H)

    Data flash memory area last address
    3 bytes
    Last address of data flash memory area
    (Sent from lower address.
    Example. F1000H to F1FFFH (4 KB) => FFH, 1FH, 0FH)

    Firmware version
    3 bytes
    Version information of firmware for programming
    (Sent from upper address.
    Example. From Ver. 1.23 => 01H, 02H, 03H)
    """
    assert len(buf) == 22
    ss = StructStreamer(buf, verbose=0)
    ss.bytes("device_code", 3)
    ss.strn("device_name", 10)
    # Example. 00000H to 0FFFFH (64 KB) => FFH, 1FH, 00H)
    v = ss.bytes("code_flash_addr_hi_raw", 3)
    ss.d["code_flash_addr_hi"] = (v[2] << 16 | v[1] << 8 | v[0])
    # Example. F1000H to F1FFFH (4 KB) => FFH, 1FH, 0FH)
    v = ss.bytes("data_flash_addr_hi_raw", 3)
    ss.d["data_flash_addr_hi"] = (v[2] << 16 | v[1] << 8 | v[0])
    v = ss.bytes("fw_ver_raw", 3)
    ss.d["fw_ver"] = "%u.%u%u" % (v[0], v[1], v[2])

    if verbose:
        print("Device code:", ss.get("device_code"))
        print("Device name:", ss.get("device_name"))
        print("Code flash address hi: 0x%06X" % ss.get("code_flash_addr_hi"))
        print("Data flash address hi: 0x%06X" % ss.get("data_flash_addr_hi"))
        # print("Data flash address hi", ss.get("data_flash_addr_hi_raw"))
        # print("Data flash address hi", ss.get("data_flash_addr_hi_raw"))
        print("FW ver:", ss.get("fw_ver"))

    if hexlify:
        ss.d["device_code"] = tohex(ss.d["device_code"])
        ss.d["fw_ver_raw"] = tohex(ss.d["fw_ver_raw"])
        ss.d["code_flash_addr_hi_raw"] = tohex(ss.d["code_flash_addr_hi_raw"])
        ss.d["data_flash_addr_hi_raw"] = tohex(ss.d["data_flash_addr_hi_raw"])

    return ss


def iter_flash_write_blocks(ss):
    block_size = 0x400
    block_mask = 0xFFFC00
    code_addr_low = 0x000000
    code_addr_high = ss.get("code_flash_addr_hi") & block_mask
    data_addr_low = 0x0F1000
    data_addr_high = ss.get("data_flash_addr_hi") & block_mask
    block_addrs = [
        (code_addr_low, code_addr_high),
        (data_addr_low, data_addr_high),
    ]
    for addr_min, addr_max in block_addrs:
        for start_addr in range(addr_min, addr_max, block_size):
            yield start_addr


def iter_flash_erase_blocks(ss):
    block_size = 0x1000
    block_mask = 0xFFFC00
    code_addr_low = 0x000000
    code_addr_high = ss.get("code_flash_addr_hi") & block_mask
    data_addr_low = 0x0F1000
    data_addr_high = ss.get("data_flash_addr_hi") & block_mask
    block_addrs = [
        (code_addr_low, code_addr_high),
        (data_addr_low, data_addr_high),
    ]
    for addr_min, addr_max in block_addrs:
        for start_addr in range(addr_min, addr_max, block_size):
            yield start_addr


def block_blank_checks(rl78, ss, hexlify=True, verbose=False):
    ret = {}
    """
    XXX: verify this is right and then sub into below
    Code flash address hi: 0x007FFF
    Data flash address hi: 0x0F1FFF
    """
    verbose and print("code_flash_addr_hi",
                      "0x%06X" % ss.get("code_flash_addr_hi"))
    verbose and print("data_flash_addr_hi",
                      "0x%06X" % ss.get("data_flash_addr_hi"))
    block_size = 0x400
    block_mask = 0xFFFC00
    code_addr_low = 0x000000
    code_addr_high = ss.get("code_flash_addr_hi") & block_mask
    data_addr_low = 0x0F1000
    data_addr_high = ss.get("data_flash_addr_hi") & block_mask
    """
    block_addrs = [
        (0x000000, 0x007C00),
        (0x0F1000, 0x0F1C00),
        ]
    """
    block_addrs = [
        (code_addr_low, code_addr_high),
        (data_addr_low, data_addr_high),
    ]
    d01 = 0
    verbose and print("Iterating...")
    for addr_min, addr_max in block_addrs:
        for start_addr in range(addr_min, addr_max, block_size):
            raw_tx, raw_st1 = rl78.a.blank_check(start_addr,
                                                 size=block_size,
                                                 d01=d01,
                                                 raw=True)
            jthis = {
                "raw_tx":
                tohex(raw_tx),
                "raw_st1":
                tohex(raw_st1),
                "is_blank":
                rl78.a.blank_check(start_addr, size=block_size, d01=d01),
                "start_addr":
                start_addr,
            }
            ret["0x%06X" % start_addr] = jthis
    return ret


def erase_all(rl78=None, ss=None):
    if not rl78:
        rl78 = try_a1()
    if not ss:
        ss = decode_sig(rl78.a.silicon_sig())
    for addr in iter_flash_erase_blocks(ss):
        rl78.a.erase_block(addr)


def dump_meta_json(rl78=None):
    if not rl78:
        rl78 = try_a1()

    sig_raw = rl78.a.silicon_sig()
    sig = decode_sig(sig_raw)
    sec_raw = rl78.a.security_get()
    bbcs = block_blank_checks(rl78, sig)

    j = {
        "silicon_sig": sig.d,
        "security_get": {
            "raw_rx": tohex(sec_raw),
        },
        "block_blank_checks": bbcs,
    }
    j["silicon_sig"]["raw_rx"] = tohex(sig_raw)
    print("")
    print_json_pretty(j)

    return rl78


def dump_checksum(rl78=None, ss=None, printj=False, omit_blank=True):
    """
    printj => ran out of memory with many things I tried

    ran into memory allocation errors...

    0x000000: 0x5F49
    0x000400: 0x0400
    0x000800: 0x0400
    0x000000: 0x6349
    """
    if not rl78:
        rl78 = try_a1()
    if not ss:
        ss = decode_sig(rl78.a.silicon_sig())

    if not printj:
        print("code_flash_addr_hi", "0x%06X" % ss.get("code_flash_addr_hi"))
        print("data_flash_addr_hi", "0x%06X" % ss.get("data_flash_addr_hi"))
    block_size = 0x100
    block_mask = 0xFFFC00
    code_addr_low = 0x000000
    code_addr_high = ss.get("code_flash_addr_hi") & block_mask
    data_addr_low = 0x0F1000
    data_addr_high = ss.get("data_flash_addr_hi") & block_mask
    block_addrs = [
        (code_addr_low, code_addr_high),
        (data_addr_low, data_addr_high),
    ]
    # ret = {}
    if printj:
        print("{")
        print("    \"checksum\": {")
    else:
        print("Iterating...")
    #ret = []
    blanks = 0
    for addr_min, addr_max in block_addrs:
        for start_addr in range(addr_min, addr_max, block_size):
            checksum = rl78.a.checksum(start_addr, block_size)
            # print("0x%06X: 0x%04X" % (start_addr, checksum))
            if printj:
                print(
                    '        "0x%06X": {"checksum": %u, "address": %u, "size": %u},'
                    % (start_addr, checksum, start_addr, block_size))
            else:
                if omit_blank and checksum == 0x100:
                    blanks += 1
                    continue
                else:
                    print("0x%06X: 0x%04X" % (start_addr, checksum))
            # ret["0x%06X" % start_addr] = {"checksum": checksum, "address": start_addr, "size": block_size}
            # ret["0x%06X" % start_addr] = checksum
            #ret.append((start_addr, checksum))
    if printj:
        print("    }")
        print("}")
    else:
        print("Blank blocks: %u" % blanks)

    # return ret


def dump_checksum_bf():
    """
    Brute force: dump all possible addressable locations
    Goal: find undocumented areas

    0x000000: 0x5F49
    0x000400: 0x0400
    0x000800: 0x0400
    0x000000: 0x6349
    """
    rl78 = try_a1()
    """
    Can we find hidden memory locations?
    """
    for start_addr in range(0, 0x1000000, 0x100):
        try:
            checksum = rl78.a.checksum(start_addr, 0x100)
            print("0x%06X: 0x%04X" % (start_addr, checksum))
        except ProtoANACK:
            continue


def probe_ocd(rl78=None, aggressive=False, printj=False):
    j = {}

    try:
        try_ocd(rl78=rl78)
        if not printj:
            print("OCD interface locked: no")
        ocd_locked = False
    except ProtectError:
        if not printj:
            print("OCD interface locked: yes")
        ocd_locked = True

    j["ocd_locked"] = ocd_locked
    if ocd_locked:
        printj and print_json_pretty(j)
        return
    """
    I believe its fairly safe to check if a password is required
    Note that default 0 password was set on my project
    So even if its locked a default password may be set
    However, attempting to unlock may trigger erase
    (never observed in the wild though)
    """

    if not aggressive:
        printj and print_json_pretty(j)
        return

    # This is probably safe
    try:
        rl78.ocd.unlock(ocd_id=None)
        if not printj:
            print("OCD security ID: no")
        has_security_id = False
    except ProtectError:
        if not printj:
            print("OCD security ID: yes")
        has_security_id = True

    j["has_security_id"] = has_security_id
    printj and print_json_pretty(j)


def probe(rl78=None, aggressive=False, printj=False):
    """
    Try to gather as much info as possible without changing anything
    Ex: can check if OCD can be entered (safe) but don't try the default password
    (as that can trigger an erase)

    aggressive: try things that might modify the device
        ex: default OCD password which could trigger erase
    """
    rl78 = try_a1(rl78=rl78)

    print("")

    # Vast majority of the info
    # Reads bulk metadata like silicon sig and security info
    if printj:
        print("<dump_meta_json()>")
        dump_meta_json(rl78=rl78)
        print("</dump_meta_json()>")
    else:
        print("dump_meta_json()")
        dump_meta_json(rl78=rl78)

    print("")

    if printj:
        print("<dump_checksum()>")
        dump_checksum(rl78=rl78, printj=True)
        print("</dump_checksum()>")
    else:
        print("dump_checksum()")
        dump_checksum(rl78=rl78, omit_blank=True)

    print("")

    if printj:
        print("<probe_ocd()>")
        probe_ocd(rl78=rl78, aggressive=aggressive, printj=printj)
        print("</probe_ocd()>")
    else:
        print("probe_ocd()")
        probe_ocd(rl78=rl78, aggressive=aggressive, printj=printj)

    # Most things expect a1 protocol
    try_a1(rl78=rl78)
