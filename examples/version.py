# PlatformIO pre-script: stamps the build with the git commit of each repo it is built from, as BUILD_REV, so the
# serial banner says what is actually on flash. "+" after a commit means uncommitted changes; "?" means not found.
# Use from an example's platformio.ini:  extra_scripts = pre:../version.py
Import("env")
import os, subprocess

project = env.subst("$PROJECT_DIR")
root = os.path.normpath(os.path.join(project, "..", ".."))   # this repo
parent = os.path.dirname(root)                                # its siblings, as lib_extra_dirs finds them

def rev(path):
    try:
        sha = subprocess.check_output(["git", "-C", path, "rev-parse", "--short", "HEAD"], stderr=subprocess.DEVNULL).decode().strip()
        dirty = subprocess.call(["git", "-C", path, "diff", "--quiet", "HEAD"], stderr=subprocess.DEVNULL) != 0
        return sha + ("+" if dirty else "")
    except Exception:
        return "?"

parts = ["OneMachine " + rev(root)] + [n + " " + rev(os.path.join(parent, n)) for n in ("OneBus", "OneChip", "HAPI")]
env.Append(CPPDEFINES=[("BUILD_REV", env.StringifyMacro(" ".join(parts)))])
