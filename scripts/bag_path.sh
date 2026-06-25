#!/usr/bin/env bash
# Resolve bag path: latest mobile bag, short name, or full directory path.
resolve_bag_path() {
  local root="$1"
  local arg="${2:-}"

  if [[ -z "$arg" || "$arg" == "latest" ]]; then
    local latest
    latest="$(ls -td "$root"/output/bags/mobile_* 2>/dev/null | head -1 || true)"
    if [[ -n "$latest" && -d "$latest" ]]; then
      printf '%s' "$latest"
      return 0
    fi
    echo "ERROR: no mobile bag under $root/output/bags/" >&2
    return 1
  fi

  if [[ -d "$arg" ]]; then
    printf '%s' "$(cd "$arg" && pwd)"
    return 0
  fi
  if [[ -d "$root/$arg" ]]; then
    printf '%s' "$(cd "$root/$arg" && pwd)"
    return 0
  fi
  if [[ -d "$root/output/bags/$arg" ]]; then
    printf '%s' "$(cd "$root/output/bags/$arg" && pwd)"
    return 0
  fi

  echo "ERROR: bag not found: $arg" >&2
  return 1
}
