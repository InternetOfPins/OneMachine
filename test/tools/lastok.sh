# Sourced by the test scripts: lastok <program> [args]. Runs a test program and succeeds only when it exits 0 AND its last line (LASTOK_N lines, default 1)
# starts with OK. `prog | tail -1 | grep -q '^OK'` looks at the text alone: a sanitizer or leak abort after the program printed OK, or a program that
# exits non-zero with an OK line, passed. Its error output is kept in $LASTOK_OUT for the script to show.
lastok() {
  local s
  LASTOK_OUT=$("$@" 2>&1); s=$?
  [ $s -eq 0 ] && printf '%s\n' "$LASTOK_OUT" | tail -n "${LASTOK_N:-1}" | grep -q '^OK'
}
