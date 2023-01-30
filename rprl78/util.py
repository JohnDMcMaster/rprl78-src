import json


def add_bool_arg(parser, yes_arg, default=False, **kwargs):
    dashed = yes_arg.replace("--", "")
    dest = dashed.replace("-", "_")
    parser.add_argument(yes_arg,
                        dest=dest,
                        action="store_true",
                        default=default,
                        **kwargs)
    parser.add_argument("--no-" + dashed,
                        dest=dest,
                        action="store_false",
                        **kwargs)


def writej(fn, j):
    with open(fn, 'w') as f:
        f.write(json.dumps(j, sort_keys=True, indent=4,
                           separators=(",", ": ")))


def printj(j):
    print(json.dumps(j, sort_keys=True, indent=4, separators=(",", ": ")))


def tobytes(buff):
    if type(buff) is str:
        #return bytearray(buff, 'ascii')
        return bytearray([ord(c) for c in buff])
    elif type(buff) is bytearray or type(buff) is bytes:
        return buff
    else:
        assert 0, type(buff)


def tostr(buff):
    if type(buff) is str:
        return buff
    elif type(buff) is bytearray or type(buff) is bytes:
        return ''.join([chr(b) for b in buff])
    else:
        assert 0, type(buff)
