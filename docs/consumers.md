# Consumers: serving a machine to another protocol

A machine speaks one thing: its link. A *consumer* is a program on the other side of that link that presents the machine in
another protocol's terms: a SiLA 2 server for lab software, a broker client for a home-automation system, a node set for an
industrial controller. The machine is built once and knows none of them; each consumer is written once and serves any machine
whose roles it understands.

[`examples/sila`](../examples/sila) is the worked example: a Python gateway that turns any machine built from its eight role
kinds into a SiLA 2 server.

## What the machine provides

Nothing consumer-specific. The firmware is built with the link (`role::Link`) and its roles' kinds; it carries no protocol
library, no protocol vocabulary and no generated files. A consumer gets two things over the link:

- **The description**, read once at connect (`m`, `c`, `r`): roles, kinds, parameters, field names and types, and the lines a
  kind prints about its values (`value`, `scale`, `unit`; see [role.md](role.md#the-consumers-contract)).
- **The frames**, binary and positional, checked by hash, for every command and report.

Everything the other protocol needs (its own self-description, types, constraints, units, names) is built from those two on
the consumer's side. A machine that a consumer can serve can be served by every other consumer that knows the same kinds, with
no change to its firmware.

## The rules

These hold for any consumer; the SiLA gateway is how each looks in practice.

1. **One table: kind to the protocol's words.** The consumer's knowledge of machines is a table with one row per role kind. A
   row is about a kind, never about a machine: "a `light`'s `level` is bounded by its `max` parameter", not "the lamp goes to
   200". Everything machine-specific (role names, limits, allowed values, labels, scales, units, text lengths) is read from the
   description at connect. In `examples/sila` the table is `KINDS` in `gateway.py`.

2. **The protocol's self-description is generated, never written by hand.** The SiLA gateway writes each feature definition
   (FDL) from the description at start. Change a limit or a label in the firmware and, after a reconnect, the consumer serves
   the new one; nothing to edit, nothing to keep in step.

3. **What the consumer does not know, it refuses, by name.** A role of a kind not in its table, a description line the kind's
   row does not use, a unit symbol not in its unit table: the consumer stops and names the role and the thing. It never guesses
   a meaning, and never serves part of a machine silently.

4. **A binding exposes only what its protocol can represent.** A protocol constraint that a client reads once at connect (a
   SiLA minimum or maximum, an allowed set) is generated only from values fixed in the firmware. A value that can change at run
   time is a readable value, never a constraint.

5. **Validation at the consumer's edge; the device is the backstop.** Values outside a constraint are refused by the consumer
   with the protocol's own error, and nothing reaches the device. A value that reaches the device anyway (a client bypassing
   the consumer) is handled by the kind: clamped to its bound, or ignored when not one of its allowed values. Either way the
   report carries the value the device holds, and the report is the truth (a command in presented units is rounded to the
   nearest raw value and read back as it was applied).

6. **The consumer holds nothing.** Each command is forwarded once, when the client sends it; nothing is resent, nothing kept
   alive on the client's behalf. Whatever the device does when commands stop is the device's own behaviour, unchanged by which
   consumer is attached. Commands are serialised on the link, so concurrent client calls cannot interleave in a frame.

7. **One consumer process serves one machine, with a stable identity.** The SiLA gateway derives its server UUID from its
   originator and the machine hash: the same firmware is always the same server, and a different description is a different
   server.

## Adding a kind

A kind is usable by a consumer once that consumer's table has a row for it. Until then, a machine with a role of that kind is
refused by that consumer, by name. Kinds and consumers grow independently: a new kind is one row in each consumer that wants
it; a new consumer is one table over the kinds that exist.

When a kind's meaning needs more than its fields and parameters say (which values are allowed, what they are called, what unit
a field is in), the kind prints it in the description (`value`, `scale`, `unit`), so every consumer reads it from the same
place instead of carrying its own copy.

## Adding a consumer

1. Map each kind you need to the protocol's words: type, constraints, unit, readable or commandable. Write it as the table.
2. Generate the protocol's self-description from the machine description; check it against the protocol's own schema where one
   exists (SiLA: the FDL schema from `sila_base`).
3. Refuse everything outside the table, by name.
4. Check with a client you did not write: an independent implementation of the protocol, against a real machine.
