#!/usr/bin/env bash
# Re-fetch CMU clips (gitignored, regenerable). Source: github.com/Shriinivas/cmubvh
# Usage: ./fetch.sh [subject]   subject = 16 (default, locomotion) | 86 (multi-family)
# Full corpus for dev analysis: clone the repo above and pass --corpus-root <path>.
set -e
cd "$(dirname "$0")"

subject="${1:-16}"
case "$subject" in
  16) seq="Sequence-015-019"; dest="bvh";   trials="16_22 16_21 16_15 16_16 16_31 16_35 16_36 16_45 16_55 16_56" ;;
  86) seq="Sequence-086-094"; dest="bvh86"; trials="86_02 86_03 86_06" ;;
  *)  echo "unknown subject '$subject' (have: 16, 86)"; exit 1 ;;
esac

base="https://raw.githubusercontent.com/Shriinivas/cmubvh/main/$seq/$subject/Data"
mkdir -p "$dest" && cd "$dest"
for t in $trials; do
  curl -sL "$base/$t.zip" -o "$t.zip" && unzip -oq "$t.zip" && rm "$t.zip"
done
echo "fetched subject $subject: $(ls -1 ${subject}_*.bvh | wc -l) clips -> experiments/cmu/$dest/"
