#!/usr/bin/env bash
# Syntax-checks one plugin source with the workspace's real compile flags.
# Pulled from the existing compile_commands.json; nothing is built or written
# into the raisin workspace.
#
#   tools/plugin_syntax_check.sh src/lidar_slam/initialpose/global_localization.cpp
set -euo pipefail

TARGET="${1:?usage: plugin_syntax_check.sh <path fragment of the source file>}"
DATABASE=/home/cgt24/raisin_master/cmake-build-release/compile_commands.json

python3 - "$DATABASE" "$TARGET" <<'PYTHON' > /tmp/plugin_syntax_cmd.sh
import json, shlex, sys

database, target = sys.argv[1], sys.argv[2]
entries = [e for e in json.load(open(database)) if target in e["file"]]
if len(entries) != 1:
    sys.exit("expected exactly one match for {!r}, found {}".format(target, len(entries)))
entry = entries[0]
words = shlex.split(entry["command"])
kept = []
skip_next = False
for word in words:
    if skip_next:
        skip_next = False
        continue
    if word == "-o":
        skip_next = True
        continue
    if word in ("-c",):
        continue
    kept.append(word)
kept.insert(1, "-fsyntax-only")
print("cd {} && {}".format(shlex.quote(entry["directory"]), " ".join(shlex.quote(w) for w in kept)))
PYTHON

bash /tmp/plugin_syntax_cmd.sh && echo "syntax ok: $TARGET"
