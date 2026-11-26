#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

if [[ ! -f .env ]]; then
  echo "Missing .env. Copy .env.example to .env and review it first." >&2
  exit 1
fi

set -a
source .env
set +a

key_file="${DEEPSEEK_API_KEY_FILE:-./secrets/deepseek_api_key.txt}"
if [[ ! -s "${key_file}" ]]; then
  echo "Missing DeepSeek key file: ${key_file}" >&2
  exit 1
fi

mkdir -p outputs
docker compose up -d
