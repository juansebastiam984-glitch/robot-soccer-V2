#!/usr/bin/env bash
# Descarga las dos dependencias grandes en las versiones exactas con las que este robot compila y funciona.
# (En la plantilla original eran submodulos de git; aqui se bajan con este script.)
#   components/arduino -> Arduino-ESP32 3.3.12
#   external/btstack   -> BTstack
set -e
cd "$(dirname "$0")"

traer() {
  local dir=$1 url=$2 sha=$3
  if [ -n "$(ls -A "$dir" 2>/dev/null)" ]; then
    echo "$dir ya tiene archivos: se salta."
    return
  fi
  mkdir -p "$dir"
  git -C "$dir" init -q
  git -C "$dir" remote add origin "$url"
  echo "Descargando $dir ($sha)..."
  git -C "$dir" fetch -q --depth 1 origin "$sha"
  git -C "$dir" checkout -q FETCH_HEAD
  echo "$dir listo."
}

traer components/arduino https://github.com/espressif/arduino-esp32.git 94afccf35fb1e401facddbcf9e13bcf7c76a31d8
traer external/btstack   https://github.com/bluekitchen/btstack.git     8a8b616e04b07612a273978131efad32a083b1ed

echo "Listo. Ahora compila con: pio run -e esp32dev"
