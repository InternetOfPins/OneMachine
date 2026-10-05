#!/usr/bin/env bash
# The test tools' own checks: lastok.sh fails a program that exits non-zero even when its last line is OK (what `prog | tail -1 | grep -q '^OK'` let
# through), and baseline.sh ignores GCC's clone numbering but not the kind of clone or the name.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/lastok.sh"
rc=0
chk() { if "$@"; then :; else echo "FAIL: $*"; rc=1; fi; }
chk lastok sh -c 'echo OK: fine'
chk eval '! lastok sh -c "echo OK: but it aborted; exit 3"'
chk eval '! lastok sh -c "echo something else"'
chk eval '! lastok sh -c "echo OK; echo not ok"'
chk env LASTOK_N=2 bash -c ". '$HERE/lastok.sh'; lastok sh -c 'echo OK; echo summary'"
[ $rc -eq 0 ] && echo "OK: lastok.sh fails a non-zero exit after an OK line, a missing OK, and an OK that is not last"
"$HERE/baseline.sh" selftest || rc=1
exit $rc
