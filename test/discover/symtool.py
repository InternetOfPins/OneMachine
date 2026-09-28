#!/usr/bin/env python3
# Symbol tables of an ELF or object, with the type-list arguments (hapi::Chain<...>) folded away: World and the
# identification fold carry the entry list in their names, so they would look like new symbols whenever an entry is added.
#   symtool.py names <file>            one normalized name per line, sorted
#   symtool.py bytes <file> <pattern>  bytes (total, RAM) of the symbols whose normalized name matches the pattern
#   symtool.py diff <a> <b>            '- name' for each symbol a has and b lacks, '+ name' for the reverse (multisets)
import re, subprocess, sys

def fold(name):
    out, i = [], 0
    while True:
        j = name.find("hapi::Chain<", i)
        if j < 0:
            out.append(name[i:]); break
        out.append(name[i:j]); k = j + 12; depth = 1
        while depth and k < len(name):
            depth += (name[k] == "<") - (name[k] == ">"); k += 1
        out.append("hapi::Chain<..>"); i = k
    return "".join(out)

def symbols(path):
    text = subprocess.run(["avr-nm", "-C", "-S", "-t", "d", path], capture_output=True, text=True, check=True).stdout
    for line in text.splitlines():
        m = re.match(r"^(?:([0-9]+) ([0-9]+)|\s+) ([A-Za-z]) (.*)$", line)
        if not m:
            m = re.match(r"^([0-9]+) ([A-Za-z]) (.*)$", line)
            if not m: continue
            yield 0, m.group(2), fold(m.group(3))
        else:
            yield int(m.group(2) or 0), m.group(3), fold(m.group(4))

if sys.argv[1] == "names":
    for _, _, n in sorted(symbols(sys.argv[2]), key=lambda t: t[2]): print(n)
elif sys.argv[1] == "diff":
    from collections import Counter
    a = Counter(n for _, _, n in symbols(sys.argv[2])); b = Counter(n for _, _, n in symbols(sys.argv[3]))
    for n in sorted((a - b).elements()): print("- " + n)
    for n in sorted((b - a).elements()): print("+ " + n)
elif sys.argv[1] == "bytes":
    pat = re.compile(sys.argv[3]); tot = ram = 0
    for size, typ, n in symbols(sys.argv[2]):
        if pat.search(n) and typ in "tTwWdDbBrRvV":
            tot += size
            if typ in "dDbB": ram += size
    print(tot, ram)
