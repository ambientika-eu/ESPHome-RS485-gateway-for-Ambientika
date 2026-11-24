#!/usr/bin/env python3
"""Zieht die Lambdas aus der ESPHome-YAML und baut daraus sim_gen.h fuer das C++-Pruefprogramm.
Fassung fuer ambientika-atom-lite.yaml (mit Boost-Skripten)."""
import sys, yaml, re

src = sys.argv[1]
out = sys.argv[2]


class L(yaml.SafeLoader):
    pass


L.add_constructor("!secret", lambda loader, node: "<secret:" + loader.construct_scalar(node) + ">")


cfg = yaml.load(open(src, encoding="utf-8"), Loader=L)
subs = cfg.get("substitutions", {})


def sub(s):
    return re.sub(r"\$\{(\w+)\}", lambda m: str(subs[m.group(1)]), s)


scripts = {s["id"]: s["then"][0]["lambda"] for s in cfg["script"]}
rx = cfg["uart"][0]["debug"]["sequence"][0]["lambda"]
sel = cfg["select"]
txt = cfg["text_sensor"][0]["lambda"]
boot = cfg["esphome"]["on_boot"]["then"][0]["lambda"]

# Tasten: Filter-Reset als Lambda, Boost-Tasten rufen Skripte auf
buttons = {b["name"]: b["on_press"][0] for b in cfg["button"]}
assert buttons["Ambientika Filter zuruecksetzen"]["lambda"].strip() == "id(amb_reset_left) = 8;"
assert buttons["Ambientika Boost 20 min"] == {"script.execute": "boost_start"}
assert buttons["Ambientika Boost beenden"] == {"script.execute": "boost_stop"}
key = [b for b in cfg["binary_sensor"] if b.get("id") == "atom_key"][0]
assert key["on_press"] == [{"script.execute": "boost_toggle"}]
assert key["pin"]["number"] == "GPIO39"
u = cfg["uart"][0]
assert (u["tx_pin"], u["rx_pin"], u["baud_rate"]) == ("GPIO19", "GPIO22", 9600)  # ESP32-Sicht laut M5-Beispielen
assert int(subs["pause_ms"]) >= 15000 and int(subs["boost_ms"]) == 1200000
assert int(subs["gap_short_ms"]) <= 1500 and int(subs["gap_ms"]) <= 4500
assert key.get("use_interrupt") is False
assert "flow_control_pin" not in u
assert cfg["esp32"]["board"] == "m5stack-atom"

parts = []
for sid, code in scripts.items():
    parts.append(f"inline void L_{sid}() {{\n" + sub(code) + "\n}\n")
parts.append("inline void L_rx(const std::vector<uint8_t> &bytes) {\n" + sub(rx) + "\n}\n")
parts.append("inline void L_on_boot() {\n" + sub(boot) + "\n}\n")
parts.append("inline std::string L_txt() {\n" + sub(txt) + "\n}\n")
for i, s in enumerate(sel):
    parts.append(f"inline std::string L_sel{i}_get() {{\n" + sub(s["lambda"]) + "\n}\n")
    parts.append(f"inline void L_sel{i}_set(const std::string &x) {{\n" + sub(s["set_action"][0]["lambda"]) + "\n}\n")
    opts = ", ".join('"' + o + '"' for o in s["options"])
    parts.append(f"static const std::vector<std::string> SEL{i}_OPTS = {{{opts}}};\n")

# Initialwerte der Globals aus der YAML (damit der Test die echten Vorgaben nutzt)
g = {x["id"]: x for x in cfg["globals"]}
for name in ["amb_mode", "amb_speed", "amb_hum", "amb_tick"]:
    parts.append(f"static const uint8_t INIT_{name} = {g[name]['initial_value']};\n")

for k in ["pause_ms", "gap_ms", "gap_short_ms", "wrg_ms", "boost_ms"]:
    parts.append(f"static const uint32_t SUB_{k.upper()} = {subs[k]}UL;\n")
open(out, "w", encoding="utf-8").write("\n".join(parts))
print("ok", out, "scripts:", list(scripts))
