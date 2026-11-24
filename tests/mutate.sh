#!/bin/bash
# Mutationsprobe: jede Mutation muss vom Pruefprogramm (oder den Pruefungen in gen.py) erkannt werden
Y=../ambientika-atom-lite.yaml
run() {  # $1 Name, $2 alt, $3 neu
  python3 - "$Y" mut.yaml "$2" "$3" <<'PY'
import sys
s=open(sys.argv[1]).read(); a=sys.argv[3]; b=sys.argv[4]
assert s.count(a)==1, a
open(sys.argv[2],"w").write(s.replace(a,b))
PY
  rm -f hm
  if ! "${PYTHON:-python3}" gen.py mut.yaml sim_gen.h >/dev/null 2>&1; then echo "erkannt: $1 (gen.py-Pruefung)"; return; fi
  g++ -std=c++17 -O2 -o hm harness.cpp 2>/dev/null || { echo "FEHLER beim Bauen: $1"; return; }
  r=$(./hm | tail -1)
  case "$r" in *"fehlgeschlagen: 0") echo "NICHT ERKANNT: $1";; *) echo "erkannt: $1 ($r)";; esac
}
run "Boost endet nie"            '(uint32_t)(now - id(amb_boost_ms)) >= ${boost_ms}UL) {' '(uint32_t)(now - id(amb_boost_ms)) >= 0xFFFFFFFFUL) {'
run "Boost behaelt Feuchtestufe" 'if (boost) { dir = 1; follow = 0; hum = 0; spd = 3; flap = 1; }' 'if (boost) { dir = 1; follow = 0; spd = 3; flap = 1; }'
run "Boost-Zeit ab Tastendruck"  'if (boost && !id(amb_boost_started) && d1 == 0x27) {' 'if (boost && !id(amb_boost_started)) {'
run "Boost umgeht Schutzlogik"   'if (sig != 0 && sig != id(amb_last_sig)) {' 'if (!boost && sig != 0 && sig != id(amb_last_sig)) {'
run "Boost ohne follow-Reset"    'if (boost) { dir = 1; follow = 0; hum = 0; spd = 3; flap = 1; }' 'if (boost) { dir = 1; hum = 0; spd = 3; flap = 1; }'
run "Toggle setzt Startzeit nicht zurueck" "id(amb_boost) = !id(amb_boost);
          id(amb_boost_started) = false;" "id(amb_boost) = !id(amb_boost);"
run "Grundeinstellung Feuchtestufe 1" 'initial_value: "1"        # Feuchtestufe 2 = Automatik' 'initial_value: "0"        # Feuchtestufe 2 = Automatik'
run "Funkstille entfernt"        'if ((uint32_t)(now - id(amb_quiet_start)) < 3000UL) return;' ''
run "Funkstille 2900 ms"         'if ((uint32_t)(now - id(amb_quiet_start)) < 3000UL) return;' 'if ((uint32_t)(now - id(amb_quiet_start)) < 2900UL) return;'
run "Sendeluecken-Schutz entfernt" 'if ((uint32_t)(now - id(amb_tx_ms)) > lim) id(amb_last_sig) = 0xFF;' ''
run "Kurze Lueckenschwelle 4,5 s" 'const uint32_t lim = id(amb_q_since_tx) ? ${gap_ms}UL : ${gap_short_ms}UL;' 'const uint32_t lim = ${gap_ms}UL;'
run "0x00 ohne Pause davor"      'if (!ok) { flap = 1; d2 = 0x00; }' ''
run "Nacht/Tag in Feuchtestufe 2/3 ohne Pause" '(hum != 0 ? spd : 0));' '0);'
run "Pause/Aus beendet Boost nicht" 'if (k >= 5) { id(amb_boost) = false; id(amb_boost_started) = false; }' ''
run "TX/RX vertauscht"           'tx_pin: GPIO19
    rx_pin: GPIO22' 'tx_pin: GPIO22
    rx_pin: GPIO19'
run "Boost 21 min"               'boost_ms: "1200000"' 'boost_ms: "1260000"'
run "Erste Statusabfrage erst nach 60 s" 'initial_value: "100"      # erste Statusabfrage 10 s nach dem Start' 'initial_value: "0"      # erste Statusabfrage 10 s nach dem Start'
rm -f mut.yaml hm
"${PYTHON:-python3}" gen.py $Y sim_gen.h >/dev/null
