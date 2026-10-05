# PlatformIO post script: the link's description, written at build time from the same types the firmware is built from (describe.cpp, src/air_tree.h).
# The firmware answers 'd' with the hash; the text is <build dir>/description/<hash>.txt (python: Tree(link, descriptions=that dir)).
# Built with the host's g++ (CXX to choose another); the libraries are the firmware's, next to this repository (lib_extra_dirs = ../../..).
import os, subprocess
Import("env")
here = env.subst("$PROJECT_DIR")
root = os.path.normpath(os.path.join(here, "..", "..", ".."))
libs = ["OneMachine", "HAPI", "OneBus", "OneData", "OneMenu", "OneItem", "OneOutput", "OneBit", "OnePin", "OneChip", "OneParse", "OneInput", "OneIO"]
inc = [f for n in libs for f in ("-I", os.path.join(root, n, "include"))]
out = os.path.join(env.subst("$BUILD_DIR"), "description")
os.makedirs(out, exist_ok=True)
exe = os.path.join(out, "describe")
cxx = os.environ.get("CXX", "g++")
subprocess.run([cxx, "-std=c++17", "-O1"] + inc + [os.path.join(here, "describe.cpp"), "-o", exe], check=True)
print("description: " + subprocess.run([exe, out], check=True, capture_output=True, text=True).stdout.strip())
