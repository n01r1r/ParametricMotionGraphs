#!/usr/bin/env bash
# Re-fetch subject-16 CMU clips (gitignored). Source: github.com/Shriinivas/cmubvh
set -e
cd "$(dirname "$0")"
mkdir -p bvh && cd bvh
base="https://raw.githubusercontent.com/Shriinivas/cmubvh/main/Sequence-015-019/16/Data"
for t in 16_22 16_21 16_15 16_16 16_31 16_35 16_36 16_45 16_55 16_56; do
  curl -sL "$base/$t.zip" -o "$t.zip" && unzip -oq "$t.zip" && rm "$t.zip"
done
echo "fetched: $(ls -1 *.bvh | wc -l) clips"
