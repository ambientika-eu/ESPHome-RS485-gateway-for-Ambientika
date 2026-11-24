// Pruefprogramm fuer ambientika-atom-lite.yaml (Fassung Atom 1.1)
// Basis: Pruefprogramm rev3. Die Lambdas werden 1:1 aus der YAML uebernommen
// (sim_gen.h, erzeugt von gen.py).
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <random>
#include <set>
#include <algorithm>

// ---------------------------------------------------------------- Mocks
static uint32_t g_now = 0;
uint32_t millis() { return g_now; }

struct Frame { uint32_t t; std::vector<uint8_t> wire; };
static std::vector<Frame> g_frames;
static std::vector<uint8_t> g_cur;

struct Uart {
  void write_byte(uint8_t b) {
    g_cur.push_back(b);
    if (b == 0x03) { g_frames.push_back({g_now, g_cur}); g_cur.clear(); }
  }
  void write_array(const uint8_t *a, size_t n) { for (size_t i = 0; i < n; i++) write_byte(a[i]); }
};
struct BinSensor { int state = -1; void publish_state(bool s) { state = s ? 1 : 0; } };

// Globale
uint8_t amb_mode, amb_speed, amb_hum;                       // restore_value: yes
uint8_t amb_wrg_dir, amb_last_sig, amb_reset_left, amb_last_d1, amb_led_state;
uint32_t amb_stop_ms, amb_run_ms, amb_quiet_start, amb_status_ms, amb_tx_ms, amb_boost_ms;
uint8_t amb_tick;
bool amb_quiet, amb_status_seen, amb_boost, amb_boost_started, amb_q_since_tx;
Uart uart_bus;
BinSensor amb_daemmerung, amb_filteralarm, amb_feuchtealarm;

#define id(x) x
#include "sim_gen.h"
#undef id

static void boot_globals(bool first) {
  if (first) { amb_mode = INIT_amb_mode; amb_speed = INIT_amb_speed; amb_hum = INIT_amb_hum; }
  amb_wrg_dir = 2; amb_last_sig = 0xFF; amb_stop_ms = 0; amb_run_ms = 0;
  amb_reset_left = 0; amb_last_d1 = 0; amb_quiet = false; amb_quiet_start = 0;
  amb_status_seen = false; amb_status_ms = 0; amb_tx_ms = 0; amb_tick = INIT_amb_tick; amb_q_since_tx = false;
  amb_boost = false; amb_boost_started = false; amb_boost_ms = 0; amb_led_state = 0;
}

// ---------------------------------------------------------------- Pruefer
static int g_fail = 0;
static long g_checks = 0;
#define CHECK(cond, ...) do { g_checks++; if (!(cond)) { g_fail++; if (g_fail < 40) { printf("FAIL t=%u: ", g_now); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static bool documented(uint8_t d1, uint8_t d2) {
  if (d2 == 0x01) return d1 == 0x00;
  if (d2 != 0x00) return false;
  if (d1 == 0x00 || d1 == 0x20) return true;
  if (d1 >= 0x24 && d1 <= 0x2B) return true;
  if (d1 >= 0x34 && d1 <= 0x3B) return true;
  static const uint8_t a[] = {0x64, 0x66, 0x68, 0x6A, 0xA4, 0xA6, 0xA8, 0xAA};
  for (uint8_t v : a) if (d1 == v) return true;
  return false;
}

static int hx(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;   // Kleinbuchstaben sind nicht zulaessig
}

struct Checker {
  bool unknown = true;          // nach (Neu)start: Motor koennte laufen
  bool running = false;
  uint8_t run_sig = 0;
  bool have_stop = false;
  uint32_t first_stop = 0;
  bool have_last_mode = false;
  uint32_t last_mode_t = 0;
  bool have_q = false;
  uint32_t last_q = 0;
  size_t seen = 0;
  long mode_frames = 0, status_frames = 0, reset_frames = 0, boost_frames = 0;
  std::set<uint8_t> d1_seen;

  bool allow_gap = false;
  bool jitter = false;          // Takt mit Jitter: Luecken erlaubt, Schutz trotzdem pruefen
  void reboot() { allow_gap = false; unknown = true; running = false; have_stop = false; have_last_mode = false; }

  void feed() {
    for (; seen < g_frames.size(); seen++) {
      const Frame &f = g_frames[seen];
      const auto &w = f.wire;
      if (have_q) CHECK((uint32_t)(f.t - last_q) >= 3000UL, "Telegramm %u ms nach Statusabfrage", (uint32_t)(f.t - last_q));
      CHECK(w.size() >= 2 && w.front() == 0x02 && w.back() == 0x03, "STX/ETX fehlt");
      std::vector<uint8_t> b;
      bool ok = (w.size() % 2 == 0);
      for (size_t i = 1; ok && i + 1 < w.size(); i += 2) {
        int h = hx(w[i]), l = hx(w[i + 1]);
        if (h < 0 || l < 0) { ok = false; break; }
        b.push_back((uint8_t) (h * 16 + l));
      }
      CHECK(ok && !b.empty(), "Rahmen nicht hexadezimal (Grossbuchstaben)");
      if (!ok || b.empty()) continue;
      uint8_t x = 0;
      for (size_t i = 0; i + 1 < b.size(); i++) x ^= b[i];
      CHECK(x == b.back(), "Pruefsumme falsch");
      if (b[0] == 0x02) {
        CHECK(b.size() == 2 && w.size() == 6, "Statusabfrage mit falscher Laenge %zu", w.size());
        if (have_last_mode) CHECK((uint32_t)(f.t - last_mode_t) >= 400UL, "Statusabfrage nur %u ms nach Moduskommando", (uint32_t)(f.t - last_mode_t));
        have_q = true; last_q = f.t; status_frames++;
        continue;
      }
      CHECK(b[0] == 0x01 && b.size() == 4 && w.size() == 10, "Moduskommando mit falscher Laenge/Typ");
      if (b.size() != 4) continue;
      uint8_t d1 = b[1], d2 = b[2];
      mode_frames++;
      d1_seen.insert(d1);
      if (d2 == 0x01) reset_frames++;
      if (d1 == 0x27) boost_frames++;
      CHECK(documented(d1, d2), "nicht dokumentierter Befehl d1=0x%02X d2=0x%02X", d1, d2);
      if (have_last_mode) {
        uint32_t gap = f.t - last_mode_t;
        bool q_between = have_q && (uint32_t)(f.t - last_q) <= gap;
        if (!allow_gap && !jitter) CHECK(gap <= (q_between ? 3600UL : 500UL), "Luecke im 500-ms-Takt: %u ms", gap);
        // Geraete-Timeout unbekannt: ab 1,2 s (nach Statusabfrage 4,5 s) gilt der Motorzustand als unbekannt
        if (gap > (q_between ? 4500UL : 1200UL)) { unknown = true; running = false; }
        allow_gap = false;
      }
      have_last_mode = true; last_mode_t = f.t;
      bool run = (d1 & 0x0C) != 0;
      uint8_t sig = d1 & 0xDC;
      if (d1 & 0xC0) sig |= (d1 & 0x03);   // Feuchtestufe 2/3: Nacht/Tag ist Moduswechsel
      if (d1 == 0x00) CHECK(!unknown && !running && have_stop && (uint32_t)(f.t - first_stop) >= 15000UL,
                            "0x00 (Klappe zu) ohne 15 s Motor Pause davor");
      if (!run) {
        if (unknown || running) { have_stop = true; first_stop = f.t; }
        unknown = false; running = false;
      } else {
        if (unknown) {
          CHECK(false, "Lauf-Telegramm 0x%02X nach Neustart ohne vorherige Pause", d1);
        } else if (running) {
          CHECK(sig == run_sig, "Wechsel im Lauf 0x%02X -> 0x%02X ohne Pause", run_sig, d1);
        } else {
          CHECK(have_stop && (uint32_t)(f.t - first_stop) >= 15000UL,
                "Pause vor 0x%02X nur %u ms", d1, (uint32_t)(f.t - first_stop));
        }
        unknown = false; running = true; run_sig = sig;
      }
    }
  }
};

static uint8_t frame_d1(const Frame &f) {
  return (uint8_t) (hx(f.wire[3]) * 16 + hx(f.wire[4]));
}

static uint8_t last_mode_d1(size_t before) {
  for (size_t i = before; i-- > 0;) if (g_frames[i].wire.size() == 10) return frame_d1(g_frames[i]);
  return 0xFF;
}

// ---------------------------------------------------------------- Ablauf
struct Sim {
  Checker ck;
  uint32_t next500 = 0;
  bool jitter = false;
  std::mt19937 rng;
  explicit Sim(uint32_t seed, uint32_t t0, bool first = true) : rng(seed) {
    g_now = t0; g_frames.clear(); g_cur.clear();
    boot_globals(first);
    start();
  }
  void start() { L_on_boot(); next500 = g_now + 500; ck.reboot(); ck.jitter = jitter; }
  void reboot(uint32_t off_ms) {
    g_now += off_ms;
    boot_globals(false);
    start();
  }
  void stall(uint32_t ms) { g_now += ms; next500 = g_now + 1; ck.allow_gap = true; }
  void step_to(uint32_t dt) {
    for (uint32_t i = 0; i < dt; i++) {
      g_now++;
      if (g_now == next500) {
        size_t n0 = g_frames.size();
        L_send_mode();
        uint32_t step = 500;
        if (jitter) { uint32_t r = rng() % 1000; step += (r < 990) ? rng() % 41 : 600 + rng() % 3900; }
        next500 += step;
        // Boost-Invarianten nach jedem Takt
        if (amb_boost && amb_boost_started)
          CHECK((uint32_t)(g_now - amb_boost_ms) < SUB_BOOST_MS + 600UL, "Boost laeuft ueber 20 min hinaus");
        if (g_frames.size() > n0 && g_frames.back().wire.size() == 10 && amb_boost) {
          uint8_t d1 = frame_d1(g_frames.back());
          bool run = (d1 & 0x0C) != 0;
          CHECK(!run || d1 == 0x27, "waehrend Boost Lauf-Telegramm 0x%02X", d1);
        }
      }
      ck.feed();
    }
  }
};

struct Phase { uint8_t d1; uint32_t t0, t1; };
static std::vector<Phase> phases(size_t from) {
  std::vector<Phase> p;
  for (size_t i = from; i < g_frames.size(); i++) {
    if (g_frames[i].wire.size() != 10) continue;
    uint8_t d1 = frame_d1(g_frames[i]);
    if (!p.empty() && p.back().d1 == d1) p.back().t1 = g_frames[i].t;
    else p.push_back({d1, g_frames[i].t, g_frames[i].t});
  }
  return p;
}
static void print_phases(const char *tag, const std::vector<Phase> &p, size_t n = 8) {
  printf("%s:", tag);
  for (size_t i = 0; i < n && i < p.size(); i++)
    printf(" 0x%02X(%us)", p[i].d1, (i + 1 < p.size() ? p[i + 1].t0 - p[i].t0 : p[i].t1 - p[i].t0) / 1000);
  printf("\n");
}

static void test_selects() {
  boot_globals(true);
  const std::vector<std::string> *opts[] = {&SEL0_OPTS, &SEL1_OPTS, &SEL2_OPTS};
  for (int s = 0; s < 3; s++) {
    for (size_t k = 0; k < opts[s]->size(); k++) {
      const std::string &o = (*opts[s])[k];
      if (s == 0) { L_sel0_set(o); CHECK(amb_mode == k && L_sel0_get() == o, "Betriebsart %s", o.c_str()); }
      if (s == 1) { L_sel1_set(o); CHECK(amb_speed == k && L_sel1_get() == o, "Geschwindigkeit %s", o.c_str()); }
      if (s == 2) { L_sel2_set(o); CHECK(amb_hum == k && L_sel2_get() == o, "Feuchtestufe %s", o.c_str()); }
    }
  }
  uint8_t m = amb_mode; L_sel0_set("Unsinn"); CHECK(amb_mode == m, "ungueltige Option aendert Betriebsart");
  amb_mode = 9; CHECK(L_sel0_get() == SEL0_OPTS[0], "Betriebsart ausserhalb Bereich");
  amb_hum = 7; CHECK(L_sel2_get() == SEL2_OPTS[0], "Feuchtestufe ausserhalb Bereich");
  amb_last_d1 = 0x2A; CHECK(L_txt() == "0x2A", "Text-Sensor");
  boot_globals(true);
  CHECK(L_sel0_get() == "Wärmerückgewinnung" && L_sel1_get() == "Stufe 2" && L_sel2_get() == "Feuchtestufe 2",
        "Grundeinstellung B+ (WRG, Tag, Feuchtestufe 2)");
  printf("Auswahlfelder/Text-Sensor/Grundeinstellung geprueft\n");
}

static std::vector<uint8_t> status_wire(uint8_t type, std::vector<uint8_t> data, bool badcrc = false) {
  std::vector<uint8_t> raw = {type};
  raw.insert(raw.end(), data.begin(), data.end());
  uint8_t x = 0; for (uint8_t v : raw) x ^= v;
  raw.push_back(badcrc ? (uint8_t) (x ^ 1) : x);
  std::vector<uint8_t> w = {0x02};
  const char *h = "0123456789ABCDEF";
  for (uint8_t v : raw) { w.push_back(h[v >> 4]); w.push_back(h[v & 15]); }
  w.push_back(0x03);
  return w;
}

static void test_rx() {
  boot_globals(true);
  g_now = 1000;
  for (uint8_t d1 = 0x08; d1 <= 0x0F; d1++) {
    L_rx(status_wire(0x00, {d1, 0, 0}));
    CHECK(amb_daemmerung.state == (d1 & 1) && amb_filteralarm.state == ((d1 >> 1) & 1) &&
          amb_feuchtealarm.state == ((d1 >> 2) & 1) && amb_status_seen, "Statusmeldung 0x%02X", d1);
  }
  amb_status_seen = false;
  L_rx(status_wire(0x01, {0x2F, 0x00}));
  CHECK(!amb_status_seen, "Echo-Rahmen ausgewertet");
  L_rx(status_wire(0x00, {0x0F, 0, 0}, true));
  CHECK(!amb_status_seen, "Rahmen mit falscher Pruefsumme ausgewertet");
  L_rx(status_wire(0x00, {0x07, 0, 0}));
  CHECK(!amb_status_seen, "Rahmen ohne Bit 3 ausgewertet");
  L_rx(status_wire(0x00, {0x0F}));
  CHECK(!amb_status_seen, "Rahmen mit falscher Laenge ausgewertet");
  auto w = status_wire(0x00, {0x0B, 0, 0});
  std::vector<uint8_t> a = {'x', 0x02, '0', '0'};
  a.insert(a.end(), w.begin(), w.begin() + 5);
  std::vector<uint8_t> b(w.begin() + 5, w.end());
  L_rx(a); CHECK(!amb_status_seen, "Teilrahmen zu frueh ausgewertet");
  L_rx(b); CHECK(amb_status_seen && amb_filteralarm.state == 1 && amb_daemmerung.state == 1 && amb_feuchtealarm.state == 0,
                 "aufgeteilter Rahmen");
  printf("Empfangsparser geprueft\n");
}

// WRG-Takt; hum < 0: Grundeinstellung aus der YAML
static void test_wrg(uint32_t t0, int hum, uint8_t want_zu, uint8_t want_ab) {
  Sim s(1, t0);
  if (hum >= 0) { amb_mode = 0; amb_speed = 2; amb_hum = (uint8_t) hum; }
  s.step_to(12 * 60 * 1000);
  auto p = phases(0);
  CHECK(p.size() >= 9, "zu wenige WRG-Phasen: %zu", p.size());
  CHECK(p[0].d1 == 0x20, "Start nicht mit Motor Pause");
  for (size_t i = 1; i + 1 < p.size(); i++) {
    uint32_t dur = p[i + 1].t0 - p[i].t0;
    if (i % 2 == 1) {
      uint8_t want = ((i / 2) % 2 == 0) ? want_zu : want_ab;
      CHECK(p[i].d1 == want, "WRG-Phase %zu: 0x%02X statt 0x%02X", i, p[i].d1, want);
      CHECK(dur >= 60000 && dur <= 64000, "WRG-Laufzeit %u ms", dur);
    } else {
      CHECK(p[i].d1 == 0x20, "zwischen den Richtungen keine Motor Pause");
      CHECK(dur >= 15000 && dur <= 19000, "WRG-Pause %u ms", dur);
    }
  }
  print_phases(hum < 0 ? "WRG Grundeinstellung B+" : "WRG Feuchtestufe 1", p, 6);
}

// Boost aus der Grundeinstellung (WRG, Feuchtestufe 2)
static void test_boost_from_wrg(uint32_t t0, uint32_t press_after) {
  Sim s(5, t0);
  s.step_to(press_after);
  size_t from = g_frames.size();
  L_boost_start();
  s.step_to(25 * 60 * 1000);
  auto p = phases(from);
  // Ablauf: [laufende Phase] -> 0x20 -> 0x27 (20 min) -> 0x20 -> 0x6A/0x66
  size_t i = 0;
  while (i < p.size() && p[i].d1 != 0x27) i++;
  CHECK(i < p.size(), "kein Boost-Telegramm");
  if (i >= p.size()) return;
  CHECK(i >= 1 && p[i - 1].d1 == 0x20, "vor Boost keine Motor Pause");
  CHECK(i + 2 < p.size(), "Boost-Ende fehlt");
  if (i + 2 >= p.size()) return;
  uint32_t dur = p[i + 1].t0 - p[i].t0;
  CHECK(dur >= SUB_BOOST_MS && dur <= SUB_BOOST_MS + 4000, "Boost-Dauer %u ms", dur);
  CHECK(p[i + 1].d1 == 0x20, "nach Boost keine Motor Pause");
  uint32_t pz = p[i + 2].t0 - p[i + 1].t0;
  CHECK(pz >= 15000 && pz <= 19000, "Pause nach Boost %u ms", pz);
  CHECK(p[i + 2].d1 == 0x6A || p[i + 2].d1 == 0x66, "nach Boost nicht zurueck in WRG/Automatik: 0x%02X", p[i + 2].d1);
  CHECK(!amb_boost, "Boost-Flag nach Ende noch gesetzt");
  printf("Boost aus WRG (Druck nach %us): Boost %u ms, danach Pause %u ms, weiter mit 0x%02X\n",
         press_after / 1000, dur, pz, p[i + 2].d1);
}

static void test_boost_from_manual() {
  Sim s(6, 5000);
  amb_mode = 1; amb_speed = 1; amb_hum = 0;
  s.step_to(40000);                        // 0x25 laeuft
  size_t from = g_frames.size();
  L_boost_start();
  s.step_to(22 * 60 * 1000);
  auto p = phases(from);
  CHECK(last_mode_d1(from) == 0x25, "vor Druck kein 0x25");
  CHECK(p.size() >= 2 && p[0].d1 == 0x27 && p[1].d1 == 0x25, "Ablauf manuell Abluft -> Boost -> Abluft");
  if (p.size() >= 2) {
    uint32_t dur = p[1].t0 - p[0].t0;
    CHECK(dur >= SUB_BOOST_MS && dur <= SUB_BOOST_MS + 4000, "Boost-Dauer %u ms", dur);
    print_phases("Boost aus Abluft Stufe 1 (gleiche Richtung, nur Drehzahl)", p, 2);
  }
}

static void test_boost_from_zuluft() {
  Sim s(7, 5000);
  amb_mode = 2; amb_speed = 1; amb_hum = 0;
  s.step_to(40000);                        // 0x29 laeuft
  size_t from = g_frames.size();
  L_boost_start();
  s.step_to(22 * 60 * 1000);
  auto p = phases(from);
  CHECK(last_mode_d1(from) == 0x29, "vor Druck kein 0x29");
  CHECK(p.size() >= 4 && p[0].d1 == 0x20 && p[1].d1 == 0x27 && p[2].d1 == 0x20 && p[3].d1 == 0x29 &&
        p[1].t0 - p[0].t0 >= 15000 && p[3].t0 - p[2].t0 >= 15000,
        "Ablauf Zuluft -> Pause -> Boost -> Pause -> Zuluft");
  print_phases("Boost aus Zuluft Stufe 1", p, 5);
}

static void test_boost_stop_early() {
  Sim s(8, 5000);
  s.step_to(3 * 60 * 1000);
  L_boost_start();
  s.step_to(90 * 1000);
  CHECK(amb_boost_started, "Boost nach 90 s noch nicht gestartet");
  size_t from = g_frames.size();
  L_boost_stop();
  s.step_to(60 * 1000);
  auto p = phases(from);
  CHECK(last_mode_d1(from) == 0x27, "vor Abbruch kein 0x27");
  CHECK(p.size() >= 2 && p[0].d1 == 0x20 && (p[1].d1 == 0x6A || p[1].d1 == 0x66), "Boost vorzeitig beenden");
  if (p.size() >= 2) CHECK(p[1].t0 - p[0].t0 >= 15000, "Pause nach Boost-Abbruch");
  print_phases("Boost vorzeitig beendet", p, 2);
}

static void test_boost_toggle_restart_reboot() {
  // Taste am Atom: zweimal druecken = aus
  Sim s(9, 5000);
  L_boost_toggle(); CHECK(amb_boost, "Taste 1x: Boost an");
  L_boost_toggle(); CHECK(!amb_boost, "Taste 2x: Boost aus");
  // Neustart beendet den Boost
  L_boost_toggle();
  s.step_to(60000);
  CHECK(amb_boost && amb_boost_started, "Boost laeuft vor Neustart");
  s.ck.feed(); s.reboot(2000);
  CHECK(!amb_boost, "Boost nach Neustart noch aktiv");
  s.step_to(30000);
  // Erneut druecken waehrend Boost: Zeit startet neu
  Sim s2(10, 5000);
  amb_mode = 1; amb_speed = 1; amb_hum = 0;
  s2.step_to(40000);
  size_t from = g_frames.size();
  L_boost_start();
  s2.step_to(10 * 60 * 1000);
  L_boost_start();                          // nach 10 min nochmal
  s2.step_to(25 * 60 * 1000);
  auto p = phases(from);
  CHECK(p.size() >= 2 && p[0].d1 == 0x27 && p[1].d1 == 0x25, "Neustart Boost Ablauf");
  if (p.size() >= 2) {
    uint32_t dur = p[1].t0 - p[0].t0;
    CHECK(dur >= 10 * 60 * 1000 + SUB_BOOST_MS - 1000 && dur <= 10 * 60 * 1000 + SUB_BOOST_MS + 4000,
          "Boost nach erneutem Druck %u ms", dur);
    printf("Boost erneut gedrueckt nach 10 min: Boost insgesamt %u s\n", dur / 1000);
  }
}

static void test_boost_from_off_and_reset() {
  Sim s(11, 5000);
  amb_mode = 6;
  s.step_to(60000);                        // 0x00 seit 60 s
  size_t from = g_frames.size();
  L_boost_start();
  s.step_to(5000);
  auto p = phases(from);
  CHECK(last_mode_d1(from) == 0x00 && !p.empty() && p[0].d1 == 0x27, "Boost aus 'Aus' heraus");
  // Filter-Reset waehrend Boost
  from = g_frames.size();
  amb_reset_left = 8;
  s.step_to(40000);
  p = phases(from);
  CHECK(last_mode_d1(from) == 0x27 && p.size() >= 3 && p[0].d1 == 0x20 && p[1].d1 == 0x00 && p[2].d1 == 0x27 &&
        p[1].t0 - p[0].t0 >= 15000,
        "Filter-Reset waehrend Boost");
  CHECK(s.ck.reset_frames == 8, "Reset-Telegramme %ld", s.ck.reset_frames);
  print_phases("Filter-Reset waehrend Boost", p, 3);
}


static void test_double_press() {
  // Taste am Atom zweimal schnell (innerhalb eines Takts) nach 5 min Boost:
  // aus und wieder an -> Boost startet neu und laeuft weitere 20 min
  Sim s(12, 5000);
  amb_mode = 1; amb_speed = 1; amb_hum = 0;
  s.step_to(40000);
  size_t from = g_frames.size();
  L_boost_toggle();
  s.step_to(5 * 60 * 1000 + 100);
  L_boost_toggle(); s.step_to(100); L_boost_toggle();
  s.step_to(25 * 60 * 1000);
  auto p = phases(from);
  CHECK(p.size() >= 2 && p[0].d1 == 0x27 && p[1].d1 == 0x25, "Doppeldruck Ablauf");
  if (p.size() >= 2) {
    uint32_t dur = p[1].t0 - p[0].t0;
    CHECK(dur >= 5 * 60 * 1000 + SUB_BOOST_MS - 1000 && dur <= 5 * 60 * 1000 + SUB_BOOST_MS + 4000, "Doppeldruck: Boost %u ms", dur);
    printf("Doppeldruck nach 5 min: Boost insgesamt %u s\n", dur / 1000);
  }
}

static void test_reset() {
  Sim s(2, 5000);
  amb_mode = 1; amb_speed = 1; amb_hum = 0;
  s.step_to(40000);
  size_t from = g_frames.size();
  amb_reset_left = 8;
  s.step_to(40000);
  auto p = phases(from);
  CHECK(p.size() >= 3 && (p[0].d1 == 0x25 || p[0].d1 == 0x20), "Reset-Ablauf Beginn");
  size_t i = (p[0].d1 == 0x25) ? 1 : 0;
  CHECK(p[i].d1 == 0x20 && p[i + 1].d1 == 0x00 && p[i + 2].d1 == 0x25, "Reset-Ablauf 0x20 -> 0x00/0x01 -> 0x25");
  CHECK(p[i + 1].t0 - p[i].t0 >= 15000, "Reset vor 15 s Pause");
  CHECK(s.ck.reset_frames == 8, "Reset-Telegramme: %ld", s.ck.reset_frames);
  print_phases("Filter-Reset aus Abluft", p, 4);
}

static void test_reset_in_quiet() {
  Sim s(3, 5000);
  amb_mode = 2;
  long q0 = s.ck.status_frames;
  int guard = 0;
  while (s.ck.status_frames == q0 && guard++ < 200000) s.step_to(1);
  s.step_to(100);
  CHECK(amb_quiet, "Testaufbau: nicht in der Funkstille");
  amb_reset_left = 8;
  s.step_to(30000);
  CHECK(s.ck.reset_frames == 8, "Reset in Funkstille: %ld Telegramme", s.ck.reset_frames);
}

static void test_stall() {
  Sim s(4, 5000);
  amb_mode = 1; amb_speed = 2; amb_hum = 0;
  s.step_to(40000);
  s.stall(30000);
  size_t from = g_frames.size();
  s.step_to(20000);
  auto p = phases(from);
  CHECK(!p.empty() && p[0].d1 == 0x20, "nach Sendeluecke zuerst 0x%02X statt Motor Pause", p.empty() ? 0 : p[0].d1);
  CHECK(p.size() >= 2 && p[1].d1 == 0x26 && p[1].t0 - p[0].t0 >= 15000, "Wiederanlauf nach Sendeluecke");
  printf("Sendeluecke 30 s: zuerst 0x%02X, Wiederanlauf 0x%02X nach %u ms\n", p[0].d1, p.size() > 1 ? p[1].d1 : 0, p.size() > 1 ? p[1].t0 - p[0].t0 : 0);
}


static void test_off_and_mode_ends_boost() {
  // Aus aus dem Lauf: erst 15 s 0x20, dann 0x00
  Sim s(13, 5000);
  amb_mode = 2; amb_speed = 2; amb_hum = 0;
  s.step_to(40000);
  size_t from = g_frames.size();
  L_sel0_set("Aus, Klappe zu");
  s.step_to(30000);
  auto p = phases(from);
  CHECK(p.size() >= 2 && p[0].d1 == 0x20 && p[1].d1 == 0x00 && p[1].t0 - p[0].t0 >= 15000, "Aus aus dem Lauf");
  print_phases("Aus aus Zuluft Stufe 2", p, 2);
  // Motor Pause / Aus beenden den Boost
  L_boost_start(); s.step_to(30000); CHECK(amb_boost, "Boost an");
  L_sel0_set("Motor Pause"); CHECK(!amb_boost, "Motor Pause beendet Boost nicht");
  L_boost_start(); L_sel0_set("Aus, Klappe zu"); CHECK(!amb_boost, "Aus beendet Boost nicht");
  L_boost_start(); L_sel0_set("Zuluft"); CHECK(amb_boost, "Zuluft-Auswahl darf Boost nicht beenden");
}

static void test_short_gap() {
  // 1,3 s ohne Telegramm (ohne Statusabfrage): zuerst wieder 15 s Pause
  Sim s(14, 5000);
  amb_mode = 1; amb_speed = 2; amb_hum = 0;
  s.step_to(40000);
  s.stall(1300);
  size_t from = g_frames.size();
  s.step_to(20000);
  auto p = phases(from);
  CHECK(p.size() >= 2 && p[0].d1 == 0x20 && p[1].d1 == 0x26 && p[1].t0 - p[0].t0 >= 15000, "nach 1,3 s Luecke zuerst Pause");
  // 1,0 s Luecke: weiterlaufen
  Sim s2(15, 5000);
  amb_mode = 1; amb_speed = 2; amb_hum = 0;
  s2.step_to(40000);
  s2.stall(1000);
  from = g_frames.size();
  s2.step_to(2000);
  p = phases(from);
  CHECK(!p.empty() && p[0].d1 == 0x26, "nach 1,0 s Luecke weiterlaufen");
  printf("Sendeluecke 1,3 s -> Pause, 1,0 s -> weiter\n");
}

static void test_first_status_query() {
  Sim s(16, 5000);
  s.step_to(15000);
  CHECK(s.ck.status_frames == 1, "erste Statusabfrage nicht innerhalb 15 s (%ld)", s.ck.status_frames);
}

static void fuzz_jitter(uint32_t seed, uint32_t dur_ms) {
  Sim s(seed, 1000);
  s.jitter = true; s.ck.jitter = true;
  std::uniform_int_distribution<int> act(0, 99);
  std::uniform_int_distribution<uint32_t> wait(1, 40000);
  uint32_t left = dur_ms;
  while (left > 0) {
    uint32_t w = std::min(wait(s.rng), left);
    if (act(s.rng) < 20) w = std::min<uint32_t>(1 + s.rng() % 900, left);
    s.step_to(w); left -= w;
    int a = act(s.rng);
    if (a < 35) L_sel0_set(SEL0_OPTS[s.rng() % SEL0_OPTS.size()]);
    else if (a < 50) L_sel1_set(SEL1_OPTS[s.rng() % SEL1_OPTS.size()]);
    else if (a < 62) L_sel2_set(SEL2_OPTS[s.rng() % SEL2_OPTS.size()]);
    else if (a < 70) amb_reset_left = 8;
    else if (a < 78) L_boost_start();
    else if (a < 83) L_boost_stop();
    else if (a < 90) L_boost_toggle();
    else if (a < 94) { s.ck.feed(); s.reboot(500 + s.rng() % 8000); }
  }
  printf("Fuzz mit Takt-Jitter seed %u: %ld Moduskommandos, %ld Statusabfragen\n", seed, s.ck.mode_frames, s.ck.status_frames);
}

static void fuzz(uint32_t seed, uint32_t t0, uint32_t dur_ms) {
  Sim s(seed, t0);
  std::uniform_int_distribution<int> act(0, 99);
  std::uniform_int_distribution<uint32_t> wait(1, 40000);
  uint32_t left = dur_ms;
  long reboots = 0, boosts = 0;
  while (left > 0) {
    uint32_t w = std::min(wait(s.rng), left);
    if (act(s.rng) < 20) w = std::min<uint32_t>(1 + s.rng() % 900, left);
    if (act(s.rng) < 3) w = std::min<uint32_t>(1200000 + s.rng() % 120000, left);   // ganze Boosts laufen lassen
    s.step_to(w); left -= w;
    int a = act(s.rng);
    if (a < 35) L_sel0_set(SEL0_OPTS[s.rng() % SEL0_OPTS.size()]);
    else if (a < 50) L_sel1_set(SEL1_OPTS[s.rng() % SEL1_OPTS.size()]);
    else if (a < 62) L_sel2_set(SEL2_OPTS[s.rng() % SEL2_OPTS.size()]);
    else if (a < 70) amb_reset_left = 8;
    else if (a < 78) { L_boost_start(); boosts++; }
    else if (a < 83) L_boost_stop();
    else if (a < 90) L_boost_toggle();
    else if (a < 94) { s.ck.feed(); s.reboot(500 + s.rng() % 8000); reboots++; }
    else if (a < 96) { s.ck.feed(); s.stall(1000 + s.rng() % 40000); }
  }
  CHECK(s.ck.status_frames >= (long) (dur_ms / 70000u), "zu wenige Statusabfragen: %ld", s.ck.status_frames);
  printf("Fuzz seed %u: %ld Moduskommandos, %ld Statusabfragen, %ld Neustarts, %ld Boost-Starts, %ld Boost-Telegramme, %zu Datenbytes\n",
         seed, s.ck.mode_frames, s.ck.status_frames, reboots, boosts, s.ck.boost_frames, s.ck.d1_seen.size());
}

int main() {
  test_selects();
  test_rx();
  test_wrg(10, -1, 0x6A, 0x66);                     // Grundeinstellung B+
  test_wrg(10, 0, 0x2A, 0x26);                      // wie rev3
  test_wrg(0xFFFFFFFFu - 200000u, -1, 0x6A, 0x66);  // ueber den millis-Ueberlauf
  for (uint32_t pa : {1000u, 20000u, 70000u, 85000u, 150000u}) test_boost_from_wrg(1000, pa);
  test_boost_from_wrg(0xFFFFFFFFu - 600000u, 100000u);   // Boost ueber den Ueberlauf
  test_boost_from_manual();
  test_boost_from_zuluft();
  test_boost_stop_early();
  test_boost_toggle_restart_reboot();
  test_boost_from_off_and_reset();
  test_double_press();
  test_off_and_mode_ends_boost();
  test_short_gap();
  test_first_status_query();
  test_reset();
  test_reset_in_quiet();
  test_stall();
  for (uint32_t seed = 11; seed < 31; seed++) fuzz(seed, 1000, 6u * 3600u * 1000u);
  fuzz(99, 0xFFFFFFFFu - 3600000u, 3u * 3600u * 1000u);
  for (uint32_t seed = 41; seed < 51; seed++) fuzz_jitter(seed, 6u * 3600u * 1000u);
  printf("\nPruefpunkte: %ld, fehlgeschlagen: %d\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
