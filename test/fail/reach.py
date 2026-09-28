#!/usr/bin/env python3
"""reach.py ELF ENTRY-SUBSTRING: functions reachable from ENTRY through direct calls/jumps, and the indirect
call/jump instructions among them. Prints one line 'reachable=N indirect=M' then the demangled names."""
import re, subprocess, sys

elf, entry = sys.argv[1], sys.argv[2]
dis = subprocess.run(["avr-objdump", "-d", "--no-show-raw-insn", elf], capture_output=True, text=True).stdout

funcs, cur = {}, None
for line in dis.splitlines():
    m = re.match(r"^[0-9a-f]+ <(.+)>:$", line)
    if m:
        cur = m.group(1); funcs[cur] = {"calls": set(), "ind": 0, "n": 0}; continue
    if cur is None: continue
    m = re.match(r"^\s+[0-9a-f]+:\t(\w+)\t?([^;]*)(?:;.*<([^>+]+)(?:\+0x[0-9a-f]+)?>)?", line)
    if not m: continue
    op, tgt = m.group(1), m.group(3)
    funcs[cur]["n"] += 1
    if op in ("icall", "eicall", "ijmp", "eijmp"): funcs[cur]["ind"] += 1
    elif op in ("call", "rcall", "jmp", "rjmp") and tgt and tgt != cur and tgt in funcs or (op in ("call", "rcall", "jmp", "rjmp") and tgt):
        if tgt != cur: funcs[cur]["calls"].add(tgt)

roots = [f for f in funcs if entry in f]
if not roots:
    print("reachable=0 indirect=0 (entry not found)"); sys.exit(2)
seen, todo = set(), list(roots)
while todo:
    f = todo.pop()
    if f in seen or f not in funcs: continue
    seen.add(f); todo.extend(funcs[f]["calls"])
ind = sum(funcs[f]["ind"] for f in seen)
print(f"reachable={len(seen)} indirect={ind}")
names = subprocess.run(["avr-c++filt"], input="\n".join(sorted(seen)), capture_output=True, text=True).stdout.splitlines()
for n in names:
    n = re.sub(r"<[^<>]*>", "<>", n)
    n = re.sub(r"<[^<>]*>", "<>", n)
    print("  " + n[:110])
