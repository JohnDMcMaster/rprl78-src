import json


def writej(fn, j):
    with open(fn, 'w') as f:
        f.write(json.dumps(j, sort_keys=True, indent=4,
                           separators=(",", ": ")))


def printj(j):
    print(json.dumps(j, sort_keys=True, indent=4, separators=(",", ": ")))
