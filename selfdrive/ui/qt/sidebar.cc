#include "selfdrive/ui/qt/sidebar.h"

#include <algorithm>
#include <cmath>

#include <QDateTime>
#include <QDebug>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMouseEvent>
#include <QPainter>

#include "common/util.h"
#include "selfdrive/ui/qt/ti_ab_logic.h"
#include "selfdrive/ui/qt/ti_pages_logic.h"
#include "selfdrive/ui/qt/ti_tests_logic.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/input.h"

// Mazda TI on-car tests. Mirrors selfdrive/car/mazda/ti_tests.py; selfdrive/car/tests/test_mazda_ti_tests.py checks
// that these limits, key sets and the allow-list agree with the Python copy.
namespace {

constexpr int TI_TEST_MAX_TESTS = 4;
constexpr int TI_TEST_MAX_SETTINGS = 3;
constexpr int TI_TEST_MAX_BINS = 5;
constexpr int TI_TEST_MAX_STEPS = 16;
constexpr int TI_TEST_MIN_ARMS = 2;
constexpr int TI_TEST_MAX_ARMS = 4;
constexpr double TI_TEST_MIN_TARGET_S = 10;
constexpr double TI_TEST_MAX_TARGET_S = 7200;
constexpr double TI_TEST_MIN_DWELL_S = 30;
constexpr double TI_TEST_MAX_DWELL_S = 3600;
constexpr double TI_TEST_MAX_SPEED_MPH = 100;
constexpr int TI_TEST_MAX_TITLE = 40;
constexpr int TI_TEST_MAX_DESCRIPTION = 200;
constexpr int TI_TEST_MAX_ID = 32;
constexpr int TI_TEST_MAX_LABEL = 16;
constexpr int TI_TEST_MAX_STATUS = 80;
constexpr int TI_TEST_STALE_S = 12;          // the car side publishes every 5 s
constexpr int TI_TEST_MISMATCH_S = 2;        // requested but not applied for longer than this: NOT APPLIED
constexpr int TI_TEST_IDLE_CLOSE_MS = 15000;  // the Tests page closes itself after this long without a touch (not when locked)
constexpr int TI_TEST_GATE_STALE_S = 4;       // the car side writes the gate block every second
constexpr int TI_TEST_INPUT_DELAY_MS = 400;   // ...and ignores touches this long after it opens
constexpr int TI_TEST_RESET_POLLS = 15;       // a reset request is repeated for this many polls until echoed
constexpr double TI_TEST_CONTROLS_STALE_S = 0.5;  // controlsState older than this counts as an alert

const char *const TI_TEST_DEFINITIONS = "../car/mazda/ti_tests.json";  // the UI runs in selfdrive/ui
const char *const TI_TEST_PROGRESS_KEY = "TiTestProgress";
const char *const TI_TEST_LIVE_KEY = "_live";
const char *const TI_TEST_RESET_KEY = "_reset";
const char *const TI_TEST_CRASH_LOG = "/data/error_logs/error.txt";
const char *const TI_TEST_OTHER = "other";
const char *const TI_TEST_ALL = "all";
const char *const TI_TEST_UNSUPPORTED = "alternate is not supported yet (test ignored)";
const char *const TESTS_QUEUE_KEY = "TiTestQueue";  // UI only: the id of the queued test the Tests page shows
const char *const TESTS_ALL_ID = "_all";            // UI only: TESTS_QUEUE_KEY value for the run-all overview page
const char *const TI_TEST_GATE_KEY = "TiTestGate";  // tmpfs, car side, 1 Hz: the limit now closing the armed test's gate
const char *const TI_TEST_LOCK_KEY = "TiTestLock";  // tmpfs, UI only: the Tests page is locked on screen
const char *const TESTS_ALL_KEY = "TiIdentAll";     // the run-all mode switch (allow-listed; in no definition)

struct TiTestAllowed {
  const char *key;     // the param a definition may switch
  const char *toggle;  // the frogpilot_toggles value that shows what the car applies
  bool on;             // the value that means "on"; "off" is the other one
};
const TiTestAllowed TI_TEST_ALLOWED[] = {
  {"TiDither", "ti_dither", true},
  {"TiSweep", "ti_sweep", true},
  {"TiLadder", "ti_ladder", true},
  {"TiPulses", "ti_pulses", true},
  {"TiIdentAll", "ti_ident_all", true},
};

const QStringList TI_TEST_ACTIVE_SOURCES = {"TiDither"};
const QStringList TI_TEST_METRICS = {"active_seconds", "engaged_seconds", "step_seconds"};
const QStringList TI_TEST_BIN_BY = {"speed_mph"};
const QStringList TI_TEST_SWITCH_WHEN = {"straight_only", "between_engagements"};
const QStringList TI_TEST_FILE_KEYS = {"version", "tests"};
const QStringList TI_TEST_TEST_KEYS = {"id", "title", "description", "settings", "progress", "auto_off", "alternate"};
const QStringList TI_TEST_TEST_REQUIRED = {"id", "title", "description", "settings", "progress"};
const QStringList TI_TEST_SETTING_KEYS = {"key", "on"};
const QStringList TI_TEST_PROGRESS_KEYS = {"metric", "target_seconds", "bin_by", "bins", "steps", "needs"};
const QStringList TI_TEST_PROGRESS_REQUIRED = {"metric", "target_seconds"};
const QStringList TI_TEST_AUTO_OFF_KEYS = {"at_ignition_off"};
const QStringList TI_TEST_ALTERNATE_KEYS = {"arms", "min_dwell_s", "switch_when"};
const QStringList TI_TEST_ARM_KEYS = {"id", "settings"};

const TiTestAllowed *tiTestAllowed(const QString &key) {
  for (const TiTestAllowed &a : TI_TEST_ALLOWED) {
    if (key == a.key) {
      return &a;
    }
  }
  return nullptr;
}

bool tiTestBool(const QJsonValue &v) {
  return v == QJsonValue(true) || v == QJsonValue(false);
}

// A string that is not blank and at most `longest` characters (a non-string reads as empty)
bool tiTestText(const QJsonValue &v, int longest) {
  const QString s = v.toString();
  return !s.trimmed().isEmpty() && s.length() <= longest;
}

// id: a-z, then a-z 0-9 _ (ti_tests.ID_RE). Label: A-Z a-z 0-9 _ . - (ti_tests.LABEL_RE)
bool tiTestChars(const QString &s, int longest, bool id) {
  const std::string t = s.toStdString();
  if (t.empty() || (int)t.size() > longest) {
    return false;
  }
  for (size_t k = 0; k < t.size(); k++) {
    const char c = t[k];
    const bool lower = c >= 'a' && c <= 'z';
    const bool digit = c >= '0' && c <= '9';
    const bool ok = id ? (lower || (k > 0 && (digit || c == '_')))
                       : (lower || digit || (c >= 'A' && c <= 'Z') || c == '_' || c == '.' || c == '-');
    if (!ok) {
      return false;
    }
  }
  return true;
}

// Every key of o is allowed, and every required key is present
bool tiTestKeys(const QJsonObject &o, const QStringList &allowed, const QStringList &required) {
  for (const QString &k : o.keys()) {
    if (!allowed.contains(k)) {
      return false;
    }
  }
  for (const QString &k : required) {
    if (!o.contains(k)) {
      return false;
    }
  }
  return true;
}

// Empty when valid
QString tiTestSettings(const QJsonValue &raw, Params &params, std::vector<QPair<QString, bool>> &out) {
  out.clear();
  const QString count = QString("settings must list 1-%1 toggles").arg(TI_TEST_MAX_SETTINGS);
  if (!raw.isArray()) {
    return count;
  }
  for (const QJsonValue &s : raw.toArray()) {
    if ((int)out.size() >= TI_TEST_MAX_SETTINGS) {
      return count;
    }
    const QJsonObject o = s.toObject();
    if (!s.isObject() || !tiTestKeys(o, TI_TEST_SETTING_KEYS, TI_TEST_SETTING_KEYS)) {
      return QString("each setting is exactly {key, on}");
    }
    const QString key = o.value("key").toString();
    const TiTestAllowed *allowed = tiTestAllowed(key);
    if (allowed == nullptr || !params.checkKey(key.toStdString())) {
      return QString("setting %1 is not allowed").arg(key);
    }
    if (!tiTestBool(o.value("on")) || o.value("on").toBool() != allowed->on) {
      return QString("setting %1: wrong on value").arg(key);
    }
    for (const QPair<QString, bool> &pair : out) {
      if (pair.first == key) {
        return QString("setting %1 twice").arg(key);
      }
    }
    out.push_back(QPair<QString, bool>(key, allowed->on));
  }
  return out.empty() ? count : QString();
}

// `steps` and `needs` (step_seconds only; undefined otherwise): per bin, in the order the bins are written, the
// count of steps and the seconds each step needs (0: optional, collected only as extra)
QString tiTestBins(const QJsonValue &raw, const QJsonValue &steps, const QJsonValue &needs, std::vector<TiTestBin> &out) {
  out.clear();
  const QString count = QString("bins must list 1-%1 bins").arg(TI_TEST_MAX_BINS);
  if (!raw.isArray()) {
    return count;
  }
  for (const QJsonValue &b : raw.toArray()) {
    if ((int)out.size() >= TI_TEST_MAX_BINS) {
      return count;
    }
    QJsonValue parts[4];
    int n = 0;
    for (const QJsonValue &part : b.toArray()) {
      if (n < 4) {
        parts[n] = part;
      }
      n++;
    }
    if (!b.isArray() || n != 4) {
      return QString("each bin is [low, high, label, required]");
    }
    TiTestBin bin;
    bin.label = parts[2].toString();
    if (!tiTestChars(bin.label, TI_TEST_MAX_LABEL, false) || bin.label == TI_TEST_OTHER || bin.label == TI_TEST_ALL) {
      return QString("bad bin label");
    }
    bin.low = parts[0].toDouble();
    bin.high = parts[1].toDouble();
    if (!parts[0].isDouble() || !parts[1].isDouble() || !(bin.low >= 0 && bin.low < bin.high && bin.high <= TI_TEST_MAX_SPEED_MPH)) {
      return QString("bin %1: need 0 <= low < high <= %2").arg(bin.label).arg(TI_TEST_MAX_SPEED_MPH);
    }
    if (!tiTestBool(parts[3])) {
      return QString("bin %1: required must be true or false").arg(bin.label);
    }
    bin.required = parts[3].toBool();
    out.push_back(bin);
  }
  if (out.empty()) {
    return count;
  }
  if (!steps.isUndefined()) {
    const QJsonArray counts = steps.toArray();
    if (!steps.isArray() || counts.size() != (int)out.size()) {
      return QString("steps must list one count per bin");
    }
    for (int k = 0; k < counts.size(); k++) {
      const double n = counts.at(k).toDouble();
      if (!counts.at(k).isDouble() || !(n >= 1 && n <= TI_TEST_MAX_STEPS) || n != double(int(n))) {
        return QString("steps must be whole numbers 1-%1").arg(TI_TEST_MAX_STEPS);
      }
      out[k].steps = int(n);
    }
    const QJsonArray per_bin = needs.toArray();
    if (!needs.isArray() || per_bin.size() != (int)out.size()) {
      return QString("needs must list each bin's seconds per step");
    }
    for (int k = 0; k < per_bin.size(); k++) {
      const QJsonArray secs = per_bin.at(k).toArray();
      if (!per_bin.at(k).isArray() || secs.size() != out[k].steps) {
        return QString("needs must list each bin's seconds per step");
      }
      out[k].needs.clear();
      for (const QJsonValue &x : secs) {
        if (!x.isDouble() || !(x.toDouble() >= 0 && x.toDouble() <= TI_TEST_MAX_TARGET_S)) {
          return QString("needs must list each bin's seconds per step");
        }
        out[k].needs.push_back(x.toDouble());
      }
    }
  }
  std::sort(out.begin(), out.end(), [](const TiTestBin &x, const TiTestBin &y) { return x.low < y.low; });
  bool any_required = out[0].required;
  for (size_t k = 1; k < out.size(); k++) {
    if (out[k].low < out[k - 1].high) {
      return QString("bins %1 and %2 overlap").arg(out[k - 1].label).arg(out[k].label);
    }
    for (size_t j = 0; j < k; j++) {
      if (out[j].label == out[k].label) {
        return QString("bin label twice");
      }
    }
    any_required = any_required || out[k].required;
  }
  return any_required ? QString() : QString("at least one bin must be required");
}

// The reserved `alternate` block: empty when it has the reserved shape
QString tiTestAlternate(const QJsonValue &raw, Params &params) {
  const QJsonObject a = raw.toObject();
  if (!raw.isObject() || !tiTestKeys(a, TI_TEST_ALTERNATE_KEYS, TI_TEST_ALTERNATE_KEYS) || !a.value("arms").isArray()) {
    return QString("alternate is exactly {arms, min_dwell_s, switch_when}");
  }
  QStringList ids;
  for (const QJsonValue &arm : a.value("arms").toArray()) {
    const QJsonObject o = arm.toObject();
    if (!arm.isObject() || !tiTestKeys(o, TI_TEST_ARM_KEYS, TI_TEST_ARM_KEYS)) {
      return QString("alternate: each arm is exactly {id, settings}");
    }
    const QString arm_id = o.value("id").toString();
    if (!tiTestChars(arm_id, TI_TEST_MAX_LABEL, false) || ids.contains(arm_id)) {
      return QString("alternate: bad or repeated arm id");
    }
    ids.append(arm_id);
    std::vector<QPair<QString, bool>> arm_settings;
    const QString why = tiTestSettings(o.value("settings"), params, arm_settings);
    if (!why.isEmpty()) {
      return QString("alternate arm %1: %2").arg(arm_id).arg(why);
    }
  }
  if (ids.size() < TI_TEST_MIN_ARMS || ids.size() > TI_TEST_MAX_ARMS) {
    return QString("alternate: arms must list %1-%2 arms").arg(TI_TEST_MIN_ARMS).arg(TI_TEST_MAX_ARMS);
  }
  const double dwell = a.value("min_dwell_s").toDouble();
  if (!a.value("min_dwell_s").isDouble() || !(dwell >= TI_TEST_MIN_DWELL_S && dwell <= TI_TEST_MAX_DWELL_S)) {
    return QString("alternate: min_dwell_s must be %1-%2 s").arg(TI_TEST_MIN_DWELL_S).arg(TI_TEST_MAX_DWELL_S);
  }
  if (!TI_TEST_SWITCH_WHEN.contains(a.value("switch_when").toString())) {
    return QString("alternate: unknown switch_when");
  }
  return QString();
}

// One test entry (ti_tests.validate_test). Empty when valid; a well-formed `alternate` block returns UNSUPPORTED.
QString tiTestValidate(const QJsonValue &raw, Params &params, TiTestDef &def) {
  const QJsonObject t = raw.toObject();
  if (!raw.isObject()) {
    return QString("not an object");
  }
  if (!tiTestKeys(t, TI_TEST_TEST_KEYS, TI_TEST_TEST_REQUIRED)) {
    return QString("unknown or missing key");
  }
  def.id = t.value("id").toString();
  if (!tiTestChars(def.id, TI_TEST_MAX_ID, true)) {
    return QString("id must be a-z then up to %1 of a-z 0-9 _").arg(TI_TEST_MAX_ID - 1);
  }
  if (!tiTestText(t.value("title"), TI_TEST_MAX_TITLE)) {
    return QString("title must be 1-%1 characters").arg(TI_TEST_MAX_TITLE);
  }
  def.title = t.value("title").toString();
  if (!tiTestText(t.value("description"), TI_TEST_MAX_DESCRIPTION)) {
    return QString("description must be 1-%1 characters").arg(TI_TEST_MAX_DESCRIPTION);
  }
  QString why = tiTestSettings(t.value("settings"), params, def.settings);
  if (!why.isEmpty()) {
    return why;
  }

  const QJsonObject p = t.value("progress").toObject();
  if (!t.value("progress").isObject() || !tiTestKeys(p, TI_TEST_PROGRESS_KEYS, TI_TEST_PROGRESS_REQUIRED)) {
    return QString("progress: unknown or missing key");
  }
  def.metric = p.value("metric").toString();
  if (!TI_TEST_METRICS.contains(def.metric)) {
    return QString("unknown metric");
  }
  if (def.metric == "active_seconds") {
    bool source = false;
    for (const QPair<QString, bool> &s : def.settings) {
      source = source || TI_TEST_ACTIVE_SOURCES.contains(s.first);
    }
    if (!source) {
      return QString("active_seconds needs a setting that reports applied time");
    }
  }
  def.target = p.value("target_seconds").toDouble();
  if (!p.value("target_seconds").isDouble() || !(def.target >= TI_TEST_MIN_TARGET_S && def.target <= TI_TEST_MAX_TARGET_S)) {
    return QString("target_seconds must be %1-%2").arg(TI_TEST_MIN_TARGET_S).arg(TI_TEST_MAX_TARGET_S);
  }
  def.bins.clear();
  if (p.contains("bins") != p.contains("bin_by")) {
    return QString("bins and bin_by go together");
  }
  if ((p.contains("steps") || p.contains("needs")) && def.metric != "step_seconds") {
    return QString("steps go with step_seconds");
  }
  if (def.metric == "step_seconds" && !(p.contains("bins") && p.contains("steps") && p.contains("needs"))) {
    return QString("step_seconds needs bins, steps and needs");
  }
  if (p.contains("bins")) {
    if (!TI_TEST_BIN_BY.contains(p.value("bin_by").toString())) {
      return QString("unknown bin_by");
    }
    why = tiTestBins(p.value("bins"), p.value("steps"), p.value("needs"), def.bins);
    if (!why.isEmpty()) {
      return why;
    }
  }

  def.off_at_ignition_off = false;
  if (t.contains("auto_off")) {
    const QJsonObject a = t.value("auto_off").toObject();
    const QJsonValue at_off = a.value("at_ignition_off");
    if (!t.value("auto_off").isObject() || !tiTestKeys(a, TI_TEST_AUTO_OFF_KEYS, QStringList()) ||
        (a.contains("at_ignition_off") && !tiTestBool(at_off))) {
      return QString("auto_off is {at_ignition_off: true/false}");
    }
    def.off_at_ignition_off = at_off.toBool();
  }

  if (t.contains("alternate")) {
    why = tiTestAlternate(t.value("alternate"), params);
    return why.isEmpty() ? QString(TI_TEST_UNSUPPORTED) : why;
  }
  return QString();
}

// A progress document without one top-level key; written back only when the key was there
void tiTestRemove(Params &store, const QString &key) {
  const QJsonObject before = QJsonDocument::fromJson(QByteArray::fromStdString(store.get(TI_TEST_PROGRESS_KEY))).object();
  if (!before.contains(key)) {
    return;
  }
  QJsonObject after;
  for (const QString &k : before.keys()) {
    if (k != key) {
      after[k] = before.value(k);
    }
  }
  store.put(TI_TEST_PROGRESS_KEY, QJsonDocument(after).toJson(QJsonDocument::Compact).toStdString());
}

// A reset request in the tmpfs copy: the entry deleted and {"_reset": {id: token}} left for the car side, which drops
// the entry on its next publish too and echoes the token in _live.reset_done (ti_tests.py). An empty tmpfs copy (after
// a reboot) starts from the flash copy, so the car side, which then reads the tmpfs copy, keeps the other entries.
void tiTestMark(Params &store, Params &fallback, const QString &key, qint64 token) {
  std::string raw = store.get(TI_TEST_PROGRESS_KEY);
  if (raw.empty()) {
    raw = fallback.get(TI_TEST_PROGRESS_KEY);
  }
  const QJsonObject before = QJsonDocument::fromJson(QByteArray::fromStdString(raw)).object();
  QJsonObject after;
  for (const QString &k : before.keys()) {
    if (k != key) {
      after[k] = before.value(k);
    }
  }
  QJsonObject marks = before.value(TI_TEST_RESET_KEY).toObject();
  marks[key] = double(token);
  after[TI_TEST_RESET_KEY] = marks;
  store.put(TI_TEST_PROGRESS_KEY, QJsonDocument(after).toJson(QJsonDocument::Compact).toStdString());
}

// One allow-listed setting to "off", written only if it is currently "on" (ALL TESTS OFF and the end-of-drive
// switch-off both use exactly this)
bool tiTestSwitchOff(Params &store, const QString &key, bool on) {
  const std::string k = key.toStdString();
  if (tiTestAllowed(key) == nullptr || !store.checkKey(k) || store.getBool(k) != on) {
    return false;
  }
  store.putBool(k, !on);
  return true;
}

// The on/off state in words and colour, from what is requested, applied and counted
void tiTestState(TiTestView &view) {
  view.light = false;
  if (view.requested && view.mismatch_ticks > TI_TEST_MISMATCH_S) {
    view.state = "NOT APPLIED";
    view.color = QColor(201, 34, 49);
  } else if (view.requested && view.active) {
    view.state = "ON · ACTIVE";
    view.color = QColor(0x17, 0x86, 0x44);
  } else if (view.requested || view.any_on) {
    view.state = view.requested ? "ON · WAIT" : "PARTLY ON";
    view.color = QColor(218, 202, 37);
    view.light = true;
  } else {
    view.state = "OFF";
    view.color = QColor(0x54, 0x54, 0x54);
  }
}

// Seconds as m:ss
QString tiTestClock(int s) {
  return QString("%1:%2").arg(s / 60).arg(s % 60, 2, 10, QChar('0'));
}

// True whenever the road view would show an alert. Mirrors OnroadAlerts::getAlert (onroad/alerts.cc:31-90), which is
// a protected member there: the crash log, a controlsState or frogpilotControlsState alert of any size,
// controlsState not yet received this drive, or controlsState stale (the UI's own "Controls Unresponsive" alerts)
bool tiTestsAlertShowing(const UIState &s, const FrogPilotUIState &fs) {
  auto &sm = *(s.sm);
  const SubMaster &fpsm = *(fs.sm);
  if (QFile::exists(TI_TEST_CRASH_LOG)) {
    return true;
  }
  if (sm.rcv_frame("controlsState") < s.scene.started_frame) {
    return true;
  }
  if ((nanos_since_boot() - sm.rcv_time("controlsState")) / 1e9 > TI_TEST_CONTROLS_STALE_S) {
    return true;
  }
  const cereal::ControlsState::Reader &cs = sm["controlsState"].getControlsState();
  const cereal::FrogPilotControlsState::Reader &fpcs = fpsm["frogpilotControlsState"].getFrogpilotControlsState();
  return cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE ||
         static_cast<cereal::ControlsState::AlertSize>(fpcs.getAlertSize()) != cereal::ControlsState::AlertSize::NONE;
}

// The tests card and page exist only on a TI-equipped Mazda at tuning level 3, as the Settings toggle does
bool tiTestsAllowed(const UIState &s, const FrogPilotUIState &fs) {
  auto &sm = *(s.sm);
  return sm.rcv_frame("carParams") > 0 && fs.frogpilot_toggles.value("car_make").toString() == "mazda" &&
         (sm["carParams"].getCarParams().getFlags() & 8) != 0 &&  // MazdaFlags.TORQUE_INTERCEPTOR (selfdrive/car/mazda/values.py)
         fs.frogpilot_toggles.value("tuning_level").toInt() >= 3;
}

}  // namespace

void TiTests::load() {
  defs.clear();
  errors.clear();

  const QString json = QString::fromStdString(util::read_file(TI_TEST_DEFINITIONS));
  const QJsonDocument doc = QJsonDocument::fromJson(json.toUtf8());
  const QJsonObject root = doc.object();
  if (json.isEmpty()) {
    errors.append(QString("Invalid definition file: cannot read selfdrive/car/mazda/ti_tests.json"));
  } else if (!doc.isObject() || !tiTestKeys(root, TI_TEST_FILE_KEYS, TI_TEST_FILE_KEYS) ||
             !root.value("version").isDouble() || root.value("version").toDouble() != 1.0) {
    errors.append(QString("Invalid definition file: must be exactly {version: 1, tests: [...]}"));
  } else if (!root.value("tests").isArray()) {
    errors.append(QString("Invalid definition file: tests must be a list"));
  } else {
    int index = 0;
    for (const QJsonValue &raw : root.value("tests").toArray()) {
      index++;
      const QString given = raw.toObject().value("id").toString();
      const QString name = given.isEmpty() ? QString("#%1").arg(index) : given;
      if (index > TI_TEST_MAX_TESTS) {
        errors.append(QString("Invalid definition: %1: more than %2 tests").arg(name).arg(TI_TEST_MAX_TESTS));
        continue;
      }
      TiTestDef def;
      const QString why = tiTestValidate(raw, tests_params, def);
      bool twice = false;
      for (const TiTestDef &d : defs) {
        twice = twice || d.id == def.id;
      }
      if (why == TI_TEST_UNSUPPORTED) {
        errors.append(QString("Not supported yet: %1 (automatic alternating), test ignored").arg(name));
      } else if (!why.isEmpty()) {
        errors.append(QString("Invalid definition: %1: %2").arg(name).arg(why));
      } else if (twice) {
        errors.append(QString("Invalid definition: %1: id twice").arg(name));
      } else {
        defs.push_back(def);
      }
    }
  }
  views.assign(defs.size(), TiTestView());

  if (errors != logged_errors) {
    for (const QString &e : errors) {
      qWarning() << "ti_tests:" << e;
    }
    logged_errors = errors;
  }
}

void TiTests::poll(const QJsonObject &toggles) {
  // The tmpfs copy is the live one (with the car side's _live block); the flash copy after a reboot
  std::string raw = tests_params_memory.get(TI_TEST_PROGRESS_KEY);
  if (raw.empty()) {
    raw = tests_params.get(TI_TEST_PROGRESS_KEY);
  }
  const QJsonObject progress = QJsonDocument::fromJson(QByteArray::fromStdString(raw)).object();
  const QJsonObject live = progress.value(TI_TEST_LIVE_KEY).toObject();
  const double age = QDateTime::currentSecsSinceEpoch() - live.value("t").toDouble();
  const bool fresh = live.value("t").isDouble() && age > -TI_TEST_STALE_S && age < TI_TEST_STALE_S;

  // Reset requests not yet echoed by the car side: repeat one whose marker a car-side write overwrote
  if (!pending_resets.keys().isEmpty()) {
    reset_polls++;
    const QJsonObject echoed = live.value("reset_done").toObject();
    const QJsonObject marks = progress.value(TI_TEST_RESET_KEY).toObject();
    QMap<QString, qint64> still;
    for (const QString &id : pending_resets.keys()) {
      const qint64 token = pending_resets.value(id);
      if (echoed.value(id).toDouble() == double(token) || reset_polls > TI_TEST_RESET_POLLS) {
        continue;
      }
      if (!(marks.value(id).toDouble() == double(token))) {
        tiTestMark(tests_params_memory, tests_params, id, token);
      }
      still.insert(id, token);
    }
    pending_resets = still;
  }

  views.resize(defs.size());
  for (size_t i = 0; i < defs.size(); i++) {
    const TiTestDef &def = defs[i];
    TiTestView &view = views[i];

    int on_count = 0;
    bool effective = true;
    for (const QPair<QString, bool> &s : def.settings) {
      if (tests_params.getBool(s.first.toStdString()) == s.second) {
        on_count++;
      }
      const TiTestAllowed *allowed = tiTestAllowed(s.first);
      if (allowed == nullptr || !(toggles.value(allowed->toggle) == QJsonValue(s.second))) {
        effective = false;
      }
    }
    view.requested = on_count == (int)def.settings.size();
    view.any_on = on_count > 0;
    view.mismatch_ticks = (view.requested && !effective) ? view.mismatch_ticks + 1 : 0;

    view.active = false;
    if (fresh) {
      for (const QJsonValue &a : live.value("active").toArray()) {
        view.active = view.active || a.toString() == def.id;
      }
    }
    view.status = fresh ? live.value("status").toObject().value(def.id).toString().left(TI_TEST_MAX_STATUS) : QString();
    view.arm = fresh ? live.value("arm").toObject().value(def.id).toString().left(TI_TEST_MAX_LABEL) : QString();

    QJsonObject entry = progress.value(def.id).toObject();
    if (!view.arm.isEmpty() && entry.value(view.arm).isObject()) {
      entry = entry.value(view.arm).toObject();
    }
    view.seconds.clear();
    for (const QString &key : entry.keys()) {
      if (entry.value(key).isDouble()) {
        view.seconds.insert(key, entry.value(key).toDouble());
      }
    }
    // A step_seconds bin is {step index: seconds}; it counts its steps 0..steps-1, each up to what it needs
    view.step_seconds.clear();
    view.step_extra.clear();
    view.step_have.clear();
    if (def.metric == "step_seconds") {
      for (const TiTestBin &bin : def.bins) {
        const QJsonObject steps = entry.value(bin.label).toObject();
        double sum = 0;
        double extra = 0;
        std::vector<double> per_step;
        for (int k = 0; k < bin.steps; k++) {
          const QJsonValue v = steps.value(QString::number(k));
          const double step_need = k < (int)bin.needs.size() ? bin.needs[k] : def.target;
          per_step.push_back(v.isDouble() ? v.toDouble() : 0.0);
          if (v.isDouble()) {
            sum += std::min(v.toDouble(), step_need);
            extra += std::max(0.0, v.toDouble() - step_need);
          }
        }
        view.step_seconds.insert(bin.label, sum);
        view.step_extra.insert(bin.label, extra);
        view.step_have.insert(bin.label, per_step);
      }
    }
    tiTestState(view);
  }

  // The run-all mode: its toggle, and the car side's _live.all block (there only while it was on in the last interval)
  const TiTestAllowed *all_allowed = tiTestAllowed(QString(TESTS_ALL_KEY));
  const QJsonObject live_all = fresh ? live.value("all").toObject() : QJsonObject();
  all_view.requested = all_allowed != nullptr && tests_params.checkKey(TESTS_ALL_KEY) &&
                       tests_params.getBool(TESTS_ALL_KEY) == all_allowed->on;
  all_view.any_on = all_view.requested;
  const bool all_effective = all_allowed != nullptr && toggles.value(all_allowed->toggle) == QJsonValue(all_allowed->on);
  all_view.mismatch_ticks = (all_view.requested && !all_effective) ? all_view.mismatch_ticks + 1 : 0;
  all_test = live_all.value("test").toString().left(TI_TEST_MAX_ID);
  all_view.status = live_all.value("status").toString().left(TI_TEST_MAX_STATUS);
  all_view.active = live_all.value("on") == QJsonValue(true) && !all_test.isEmpty();
  live_fresh = fresh;
  live_all_on = fresh && live_all.value("on") == QJsonValue(true);
  live_tests_on = fresh && !live.value("on").toArray().isEmpty();
  tiTestState(all_view);

  // The limit that closes the armed test's gate right now (command, bend, lateral demand), written by the car side once a
  // second; an empty or old block is no limit
  const QJsonObject gj = QJsonDocument::fromJson(QByteArray::fromStdString(tests_params_memory.get(TI_TEST_GATE_KEY))).object();
  const double gate_age = QDateTime::currentSecsSinceEpoch() - gj.value("t").toDouble();
  gate = TiTestGate();
  if (gj.value("t").isDouble() && gj.value("reason").isString() && gj.value("value").isDouble() && gj.value("limit").isDouble() &&
      gate_age > -TI_TEST_STALE_S && gate_age < TI_TEST_GATE_STALE_S) {
    gate.valid = true;
    gate.test = gj.value("test").toString().left(TI_TEST_MAX_ID);
    gate.reason = gj.value("reason").toString().left(TI_TEST_MAX_ID);
    gate.what = gj.value("what").toString().left(TI_TEST_MAX_LABEL * 2);
    gate.unit = gj.value("unit").toString().left(TI_TEST_MAX_LABEL);
    gate.window = gj.value("window").toString().left(TI_TEST_MAX_LABEL);
    gate.value = gj.value("value").toDouble();
    gate.limit = gj.value("limit").toDouble();
    gate.rearm = gj.value("rearm") == QJsonValue(true);
    gate.running = gj.value("running") == QJsonValue(true);
  }
}

void TiTests::setTest(int i, bool on) {
  for (const QPair<QString, bool> &s : defs[i].settings) {
    tests_params.putBool(s.first.toStdString(), on ? s.second : !s.second);
  }
}

// Same writes as the Settings toggles; updateFrogPilotToggles() makes them live, as in Sidebar::mousePressEvent
void TiTests::toggle(int i) {
  if (i < 0 || i >= (int)defs.size() || i >= (int)views.size()) {
    return;
  }
  const bool on = !views[i].requested;
  if (on) {
    tests_params_memory.putBool("TiPlanOn", false); // the drive plan runs the tests itself: a single test ends it
    tests_params_memory.putBool("TiAbOn", false);   // one injection at a time: A/B and the tests exclude each other
    tests_params_memory.remove("TiAbPin");
  }
  setTest(i, on);
  views[i].mismatch_ticks = 0;
  if (on && all_view.requested) {
    setAll(false);  // one injection at a time: a single test switched on ends run all
    all_view.mismatch_ticks = 0;
  }
  updateFrogPilotToggles();
}

void TiTests::setAll(bool on) {
  const TiTestAllowed *allowed = tiTestAllowed(QString(TESTS_ALL_KEY));
  if (allowed != nullptr && tests_params.checkKey(TESTS_ALL_KEY)) {
    tests_params.putBool(TESTS_ALL_KEY, on ? allowed->on : !allowed->on);
  }
}

// The run-all mode on, written as toggle() writes a test's settings: one injection at a time, so the single
// identification tests go off (the dither stays as it is and, while on, holds everything else)
void TiTests::startAll() {
  runOps(ti_pages::runAllOps());
}

// The ordered writes of ti_pages_logic.h (START, STOP, RUN ALL, ALL TESTS OFF), executed one by one in that order
void TiTests::runOps(const std::vector<ti_pages::Op> &ops, const std::function<void()> &save_config) {
  for (const ti_pages::Op &op : ops) {
    switch (op.kind) {
      case ti_pages::OpKind::MemBool:
        tests_params_memory.putBool(op.key, op.value);
        break;
      case ti_pages::OpKind::MemRemove:
        tests_params_memory.remove(op.key);
        break;
      case ti_pages::OpKind::AllowedOff:   // every allow-listed key to "off", whatever the definitions say (works with a broken file too)
        for (const TiTestAllowed &a : TI_TEST_ALLOWED) {
          tiTestSwitchOff(tests_params, QString(a.key), a.on);
        }
        for (TiTestView &view : views) {
          view.mismatch_ticks = 0;
        }
        all_view.mismatch_ticks = 0;
        break;
      case ti_pages::OpKind::RunAllOn:
        setAll(true);
        all_view.mismatch_ticks = 0;
        break;
      case ti_pages::OpKind::SingleStepTestsOff:   // the dither stays as it is and, while on, holds everything else
        for (size_t i = 0; i < defs.size() && i < views.size(); i++) {
          if (defs[i].metric == "step_seconds" && views[i].any_on) {
            setTest((int)i, false);
            views[i].mismatch_ticks = 0;
          }
        }
        break;
      case ti_pages::OpKind::SaveAbConfig:
        if (save_config) {
          save_config();
        }
        break;
      case ti_pages::OpKind::UpdateToggles:
        updateFrogPilotToggles();
        break;
    }
  }
}

bool TiTests::singleTestOn() const {
  for (const TiTestView &view : views) {
    if (view.any_on) {
      return true;
    }
  }
  return false;
}

// The run-all mode on or off
void TiTests::toggleAll() {
  if (!all_view.requested) {
    startAll();
    return;
  }
  setAll(false);
  all_view.mismatch_ticks = 0;
  updateFrogPilotToggles();
}

// ALL TESTS OFF: the plan off and every allow-listed key to "off"
void TiTests::allOff() {
  runOps(ti_pages::allTestsOffOps());
}

// Reset = the test's entry deleted from both copies (ti_tests.py); the car side only ever adds to what is there
void TiTests::resetProgress(int i) {
  if (i < 0 || i >= (int)defs.size()) {
    return;
  }
  const qint64 token = QDateTime::currentMSecsSinceEpoch();
  tiTestRemove(tests_params, defs[i].id);
  tiTestMark(tests_params_memory, tests_params, defs[i].id, token);
  pending_resets.insert(defs[i].id, token);
  reset_polls = 0;
  if (i < (int)views.size()) {
    views[i].seconds.clear();
    views[i].step_seconds.clear();
    views[i].step_extra.clear();
    views[i].step_have.clear();
  }
}

// The queued test (TESTS_QUEUE_KEY): that id when it is a loaded test, else the first test not done, else the first.
// (TESTS_ALL_ID, the run-all overview, is not a test of the queue: see overviewQueued.)
int TiTests::queued() {
  const std::string key = TESTS_QUEUE_KEY;
  const QString id = tests_params.checkKey(key) ? QString::fromStdString(tests_params.get(key)) : QString();
  for (int i = 0; i < (int)defs.size(); i++) {
    if (defs[i].id == id) {
      return i;
    }
  }
  for (int i = 0; i < (int)defs.size(); i++) {
    if (!done(i)) {
      return i;
    }
  }
  return 0;
}

// The queue: the loaded tests. The run-all overview is a page of its own, opened by the RUN ALL button.
int TiTests::queueSize() const {
  return (int)defs.size();
}

bool TiTests::hasStepTests() const {
  for (const TiTestDef &def : defs) {
    if (def.metric == "step_seconds") {
      return true;
    }
  }
  return false;
}

// The page the queue key last left: the run-all overview
bool TiTests::overviewQueued() {
  const std::string key = TESTS_QUEUE_KEY;
  return hasStepTests() && tests_params.checkKey(key) && QString::fromStdString(tests_params.get(key)) == QString(TESTS_ALL_ID);
}

// The queue pointer from test `from` to test `to`. A test that is on moves with it: off for the one left, on for the
// new one; one that is off only moves the pointer. Only ever on a tap, never by itself.
bool TiTests::moveQueue(int from, int to) {
  if (from < 0 || from >= queueSize() || to < 0 || to >= queueSize() || from == to) {
    return false;
  }
  const std::string key = TESTS_QUEUE_KEY;
  if (tests_params.checkKey(key)) {
    tests_params.put(key, defs[to].id.toStdString());
  }
  const bool from_on = from < (int)views.size() && views[from].requested;
  if (from_on) {
    setTest(from, false);
    setTest(to, true);
    for (TiTestView &view : views) {
      view.mismatch_ticks = 0;
    }
    updateFrogPilotToggles();
  }
  return true;
}

// RUN ALL: the overview page, and run-all mode switched on (a single test that was on goes off: one injection at a
// time)
void TiTests::openOverview() {
  if (!hasStepTests()) {
    return;
  }
  if (tests_params.checkKey(TESTS_QUEUE_KEY)) {
    tests_params.put(TESTS_QUEUE_KEY, std::string(TESTS_ALL_ID));
  }
  startAll();
}

// Back to the test pages: run-all mode off (nothing else is switched on by it), the queue on test `to`
void TiTests::closeOverview(int to) {
  if (to >= 0 && to < (int)defs.size() && tests_params.checkKey(TESTS_QUEUE_KEY)) {
    tests_params.put(TESTS_QUEUE_KEY, defs[to].id.toStdString());
  }
  if (all_view.requested) {
    setAll(false);
    all_view.mismatch_ticks = 0;
    updateFrogPilotToggles();
  }
}

// The lock of the Tests page (tmpfs param: gone at reboot, kept across engaging, disengaging and a drive ending)
bool TiTests::locked() {
  return tests_params_memory.checkKey(TI_TEST_LOCK_KEY) && tests_params_memory.getBool(TI_TEST_LOCK_KEY);
}

void TiTests::setLocked(bool on) {
  if (tests_params_memory.checkKey(TI_TEST_LOCK_KEY)) {
    tests_params_memory.putBool(TI_TEST_LOCK_KEY, on);
  }
}

// Required seconds and required cells (steps) still to collect by test i in the window `label`; false when there is
// no such step test or window
bool TiTests::windowLeft(int i, const QString &label, double &secs, int &cells) const {
  secs = 0;
  cells = 0;
  if (i < 0 || i >= (int)defs.size() || i >= (int)views.size() || defs[i].metric != "step_seconds") {
    return false;
  }
  for (const TiTestBin &bin : defs[i].bins) {
    if (bin.label != label) {
      continue;
    }
    const std::vector<double> have = views[i].step_have.value(label);
    for (int k = 0; k < bin.steps; k++) {
      const double step_need = k < (int)bin.needs.size() ? bin.needs[k] : defs[i].target;
      const double got = k < (int)have.size() ? have[k] : 0.0;
      if (step_need > 0 && got < step_need - 1e-6) {
        secs += step_need - got;
        cells++;
      }
    }
    return true;
  }
  return false;
}

// Required items (steps with a need) done and in all, in window w, over the step tests `scope`; the seconds still needed there
void TiTests::windowShare(const std::vector<int> &scope, int w, int &done, int &total, double &secs_left) const {
  done = 0;
  total = 0;
  secs_left = 0;
  int open = 0;
  const QString label = QString(ti_logic::WINDOWS[w].label);
  for (int i : scope) {
    if (i < 0 || i >= (int)defs.size() || defs[i].metric != "step_seconds") {
      continue;
    }
    for (const TiTestBin &bin : defs[i].bins) {
      if (bin.label != label) {
        continue;
      }
      for (int k = 0; k < bin.steps; k++) {
        const double step_need = k < (int)bin.needs.size() ? bin.needs[k] : defs[i].target;
        if (step_need > 0) {
          total++;
        }
      }
    }
    double secs = 0;
    int cells = 0;
    if (windowLeft(i, label, secs, cells)) {
      secs_left += secs;
      open += cells;
    }
  }
  done = std::max(0, total - open);
}

// Seconds credited so far in window w over the step tests `scope` (each step's seconds, and what came beyond its need)
double TiTests::windowCredited(const std::vector<int> &scope, int w) const {
  const QString label = QString(ti_logic::WINDOWS[w].label);
  double sum = 0;
  for (int i : scope) {
    if (i >= 0 && i < (int)views.size()) {
      sum += views[i].step_seconds.value(label, 0.0) + views[i].step_extra.value(label, 0.0);
    }
  }
  return sum;
}

// The window to ask the driver for, among the step tests `scope` (indices into defs)
ti_logic::Target TiTests::target(const std::vector<int> &scope, double mph) const {
  double secs[ti_logic::N_WINDOWS] = {0, 0, 0, 0};
  int cells[ti_logic::N_WINDOWS] = {0, 0, 0, 0};
  for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
    for (int i : scope) {
      double s = 0;
      int c = 0;
      if (windowLeft(i, QString(ti_logic::WINDOWS[w].label), s, c)) {
        secs[w] += s;
        cells[w] += c;
      }
    }
  }
  return ti_logic::chooseTarget(mph, secs, cells);
}

// At the end of a drive (onroad -> offroad): switch off the tests that ask for it and the run-all mode, and copy the
// live progress to flash so the last minute of counting survives a power-down before the next drive
void TiTests::driveEnded() {
  // The comma stays powered across a stop: the drive plan, the alternation and its pin go OFF with the drive, as the run-all mode does
  // (and as controls does when it restarts), so the screens never show ON for a mode that is not running
  tests_params_memory.putBool(ti_pages::KEY_PLAN_ON, false);
  tests_params_memory.putBool(ti_pages::KEY_AB_ON, false);
  tests_params_memory.remove(ti_pages::KEY_AB_PIN);
  bool changed = tiTestSwitchOff(tests_params, QString(TESTS_ALL_KEY), true);
  for (const TiTestDef &def : defs) {
    if (!def.off_at_ignition_off) {
      continue;
    }
    for (const QPair<QString, bool> &s : def.settings) {
      changed = tiTestSwitchOff(tests_params, s.first, s.second) || changed;
    }
  }
  if (changed) {
    updateFrogPilotToggles();
  }

  const QJsonObject live_copy = QJsonDocument::fromJson(QByteArray::fromStdString(tests_params_memory.get(TI_TEST_PROGRESS_KEY))).object();
  QJsonObject kept;
  for (const QString &k : live_copy.keys()) {
    if (!k.startsWith("_")) {  // not the live block or a reset request
      kept[k] = live_copy.value(k);
    }
  }
  if (!kept.keys().isEmpty()) {
    tests_params.putNonBlocking(TI_TEST_PROGRESS_KEY, QJsonDocument(kept).toJson(QJsonDocument::Compact).toStdString());
  }
}

bool TiTests::done(int i) const {
  if (i < 0 || i >= (int)defs.size() || i >= (int)views.size()) {
    return false;
  }
  const TiTestDef &def = defs[i];
  if (def.bins.empty()) {
    return views[i].seconds.value(TI_TEST_ALL, 0.0) >= def.target;
  }
  for (const TiTestBin &bin : def.bins) {
    if (bin.required && have(i, bin) < need(def, bin)) {
      return false;
    }
  }
  return true;
}

// Seconds a bin has (step_seconds: its steps' clipped sum) and needs (step_seconds: the sum of its steps' needs)
double TiTests::have(int i, const TiTestBin &bin) const {
  if (i < 0 || i >= (int)defs.size() || i >= (int)views.size()) {
    return 0.0;
  }
  return defs[i].metric == "step_seconds" ? views[i].step_seconds.value(bin.label, 0.0) : views[i].seconds.value(bin.label, 0.0);
}

double TiTests::need(const TiTestDef &def, const TiTestBin &bin) {
  if (!bin.needs.empty()) {
    double sum = 0;
    for (double x : bin.needs) {
      sum += x;
    }
    return sum;
  }
  return bin.steps > 0 ? bin.steps * def.target : def.target;
}

int TiTests::countOn() const {
  int n = 0;
  for (const TiTestView &view : views) {
    n += view.any_on ? 1 : 0;
  }
  return n + (all_view.any_on ? 1 : 0);
}

// Collected share of the required time of the tests that are on, 0-100; -1 when none is on
int TiTests::overallPercent() const {
  double sum = 0;
  int n = 0;
  for (size_t i = 0; i < defs.size() && i < views.size(); i++) {
    if (!views[i].requested) {
      continue;
    }
    const TiTestDef &def = defs[i];
    if (def.bins.empty()) {
      sum += std::min(1.0, views[i].seconds.value(TI_TEST_ALL, 0.0) / def.target);
      n++;
    }
    for (const TiTestBin &bin : def.bins) {
      if (bin.required) {
        sum += std::min(1.0, have((int)i, bin) / need(def, bin));
        n++;
      }
    }
  }
  return n > 0 ? int(100.0 * sum / n) : -1;
}

// A bin in the device's units (the definitions are in mph), as data_settings.cc shows speeds
QString TiTests::binText(const TiTestBin &bin, bool metric) {
  const double conversion = metric ? MILE_TO_KM : 1.0;
  return QString("%1-%2 %3").arg(qRound(bin.low * conversion)).arg(qRound(bin.high * conversion))
                            .arg(metric ? QString("km/h") : QString("mph"));
}

// The bin a speed counts in, half-open [low, high) as in ti_tests.TestDef.bin_for
QString TiTests::binFor(const TiTestDef &def, double speed_mph) {
  if (def.bins.empty()) {
    return QString(TI_TEST_ALL);
  }
  for (const TiTestBin &bin : def.bins) {
    if (bin.low <= speed_mph && speed_mph < bin.high) {
      return bin.label;
    }
  }
  return QString(TI_TEST_OTHER);
}

// TestsWindow: the full-screen Tests page

TestsWindow::TestsWindow(QWidget *parent) : QWidget(parent) {
  idle.start();
  opened.start();
  locked = tests.locked();

  ticker = new QTimer(this);
  QObject::connect(ticker, &QTimer::timeout, [this]() {
    tick();
  });
}

// False (and the page stays closed) while the road view shows an alert
bool TestsWindow::openPage() {
  if (tiTestsAlertShowing(*uiState(), *frogpilotUIState())) {
    return false;
  }
  tests.load();
  locked = tests.locked();
  if (!locked) {
    goHome();             // the first page; a locked page that an alert took away comes back as it was
  }
  overview = tests.overviewQueued();
  abOpen();               // the stored set-up: whether an alternation is set up decides what START runs
  reopen.reset();
  idle.start();
  opened.start();
  pressed_control = 0;
  queue_pos = -1;
  tick_count = 0;
  refresh();
  return true;
}

// Called by the main window on every UI update: true when a LOCKED page that an alert (or the end of a drive) took
// off the screen should come back, once nothing has alerted for a couple of seconds (ti_tests_logic.h LockReopen)
bool TestsWindow::wantsReopen(bool home_showing) {
  const UIState &s = *uiState();
  const FrogPilotUIState &fs = *frogpilotUIState();
  return reopen.step(locked, home_showing, s.scene.started, tiTestsAllowed(s, fs), tiTestsAlertShowing(s, fs));
}

void TestsWindow::showEvent(QShowEvent *event) {
  ticker->start(250);
}

void TestsWindow::hideEvent(QHideEvent *event) {
  ticker->stop();
}

void TestsWindow::leave() {
  if (on_close) {
    on_close();
  }
}

void TestsWindow::tick() {
  if (!isVisible()) {
    return;
  }
  const UIState &s = *uiState();

  // Back to the driving screen at the end of the drive and at once for ANY alert the road view would show, locked or
  // not (an alert is never hidden behind this page); after 15 s without a touch unless the page is locked
  const bool alert = tiTestsAlertShowing(s, *frogpilotUIState());
  const bool idle_expired = idle.elapsed() > TI_TEST_IDLE_CLOSE_MS;
  if (ti_logic::pageDecision(locked, s.scene.started, alert, idle_expired) == ti_logic::PageDecision::Leave) {
    leave();
    return;
  }

  if (++tick_count % 4 == 0) {
    refresh();
  }
}

void TestsWindow::refresh() {
  const UIState &s = *uiState();
  auto &sm = *(s.sm);

  is_metric = s.scene.is_metric;
  have_speed = sm.rcv_frame("carState") > 0;
  speed_mph = have_speed ? sm["carState"].getCarState().getVEgo() * MS_TO_MPH : 0.0;
  tests.poll(frogpilotUIState()->frogpilot_toggles);
  abPoll();                  // the plan line is on both pages
  cruisePoll();              // the cruise check: its page, and whether START and the switches wait for it
  if (!ab_hold_why.isEmpty() && ti_ab::holdText(ab_hold_why.toStdString().c_str())[0] != 0) {
    // the tests are held for a reason of the plan or the alternation: say it, not "another test is on"
    const QString text = QString(ti_ab::holdText(ab_hold_why.toStdString().c_str()));
    for (TiTestView &view : tests.views) {
      view.status = text;
    }
    tests.all_view.status = text;
  }
  // The queue is read when the page opens; after that only PREV / NEXT move the shown test
  if (queue_pos < 0 || queue_pos >= tests.queueSize()) {
    queue_pos = tests.queued();
  }
  if (overview && !tests.hasStepTests()) {
    overview = false;
  }
  update();
}

namespace {
const char *const TI_AB_CONFIG_KEY = "TiAbConfig";
const char *const TI_DRIVE_PLAN_FILE = "../car/mazda/ti_drive_plan.json";   // the UI runs in selfdrive/ui
const char *const TI_AB_PROGRESS_KEY = "TiAbProgress";
const char *const TI_AB_ON_KEY = "TiAbOn";
const char *const TI_AB_PIN_KEY = "TiAbPin";
const char *const TI_AB_STATUS_KEY = "TiAbStatus";
const char *const TI_PLAN_ON_KEY = "TiPlanOn";
const char *const TI_HOLD_KEY = "TiTestHold";
}  // namespace

// Every rectangle comes from ti_pages_logic.h (layout), the one place the sizes and gaps are tested
static QRect qr(const ti_pages::Rect &r) {
  return QRect(r.x, r.y, r.w, r.h);
}

int TestsWindow::rowTop() const {
  return ti_pages::ROW_TOP;
}

int TestsWindow::rowHeight() const {
  return ti_pages::rowHeight((int)tests.errors.size());
}

ti_pages::Page TestsWindow::pageNow() const {
  if (cruise_page) {
    return ti_pages::Page::Cruise;
  }
  if (progress_page) {
    return ti_pages::Page::Progress;
  }
  if (setup_page) {
    return ti_pages::Page::Setup;
  }
  if (home_page) {
    return ti_pages::Page::Home;
  }
  if (ab_page) {
    return ti_pages::Page::Ab;
  }
  return overview ? ti_pages::Page::Overview : ti_pages::Page::Tests;
}

QRect TestsWindow::ctlRect(int id) const {
  ti_pages::Rect r{0, 0, 0, 0};
  return ti_pages::rectOf(pageNow(), id, (int)tests.errors.size(), r) ? qr(r) : QRect();
}

QRect TestsWindow::backRect() const {
  return ctlRect(ti_pages::CTL_BACK);
}

QRect TestsWindow::allOffRect() const {
  return ctlRect(ti_pages::CTL_ALLOFF);
}

QRect TestsWindow::prevRect() const {
  return ctlRect(ti_pages::CTL_PREV);
}

QRect TestsWindow::nextRect() const {
  return ctlRect(ti_pages::CTL_NEXT);
}

QRect TestsWindow::lockRect() const {
  return ctlRect(ti_pages::CTL_LOCK);
}

// The banner: under a one-line note of what the page is for (both in the box beside the second row of controls)
QRect TestsWindow::bannerRect() const {
  return QRect(ti_pages::BANNER_RECT.x, ti_pages::BANNER_RECT.y + 70, ti_pages::BANNER_RECT.w, ti_pages::BANNER_RECT.h - 70);
}

QRect TestsWindow::toggleRect() const {
  return ctlRect(ti_pages::CTL_TOGGLE);
}

QRect TestsWindow::resetRect() const {
  return ctlRect(ti_pages::CTL_RESET);
}

// Controls (ti_pages_logic.h Ctl): the first page and the set-up page use the ids as they are; on the test pages 10 + i is
// the on/off box of the shown test i (the overview: i = defs.size(), run-all mode) and 20 + i its Reset progress; 0 none
int TestsWindow::controlAt(const QPoint &pos) const {
  using namespace ti_pages;
  const Page pg = pageNow();
  const int id = ti_pages::controlAt(pg, (int)tests.errors.size(), pos.x(), pos.y());
  if (pg != Page::Tests && pg != Page::Overview) {
    return id;
  }
  if (id == CTL_BACK || id == CTL_ALLOFF || id == CTL_LOCK) {
    return id;
  }
  if (id == CTL_AB_PAGE) {
    return id;
  }
  if (queue_pos < 0 || queue_pos >= tests.queueSize()) {
    return 0;
  }
  if (pg == Page::Overview) {
    if (id == CTL_OVERVIEW_BACK) {
      return 7;
    }
    return id == CTL_TOGGLE ? 10 + (int)tests.defs.size() : 0;
  }
  switch (id) {
    case CTL_PREV: return queue_pos > 0 ? 3 : 0;
    case CTL_NEXT: return queue_pos + 1 < tests.queueSize() ? 4 : 0;
    case CTL_OVERVIEW: return tests.hasStepTests() ? 6 : 0;
    case CTL_TOGGLE: return 10 + queue_pos;
    case CTL_RESET: return queue_pos < (int)tests.defs.size() ? 20 + queue_pos : 0;
  }
  return 0;
}

// A press only records which control it is on, and none in the first moment after the page opens (the on/off box
// sits where the sidebar card was, so a second tap or a bounce of the opening tap must not switch the test)
void TestsWindow::mousePressEvent(QMouseEvent *event) {
  idle.start();
  pressed_control = opened.elapsed() < TI_TEST_INPUT_DELAY_MS ? 0 : controlAt(event->pos());
}

// The first page again (from the set-up page and the individual tests). Nothing is switched by going there.
void TestsWindow::goHome() {
  abOpen();                // the set-up page's unconfirmed draft is dropped: the first page shows, and START runs, what is saved
  cruise_page = false;
  home_page = true;
  setup_page = false;
  progress_page = false;
  ab_page = false;
}

// What is switched on, in the order of ti_pages::Run
bool TestsWindow::running() const {
  return ab_plan_on || ab_on || tests.all_view.requested || tests.singleTestOn();
}

// The first page's controls. START and STOP write exactly what ti_pages::startOps / stopOps list (tested off the device).
void TestsWindow::homeAct(int control) {
  using namespace ti_pages;
  if (control == CTL_NAV_SETUP) {
    setupOpen();
    home_page = false;
    setup_page = true;
  } else if (control == CTL_NAV_PROGRESS) {
    home_page = false;
    setup_page = false;
    ab_page = false;
    progress_page = true;
  } else if (control == CTL_NAV_INDIV) {   // (from the progress page)
    home_page = false;
    setup_page = false;
    ab_page = false;
    progress_page = false;
  } else if (control == CTL_STOP) {
    ab_pin_b_armed = -1.0;
    tests.runOps(stopOps());
    abPoll();
  } else if (control == CTL_START) {
    abOpen();   // the stored set-up as it is now (the plan runs the alternation from it)
    TiHomeStrings keep;
    const HomeState hs = homeState(keep);
    if (!plan_ok || !startLook(ab_configured, hs.run, carRunning(hs), plan_ok, plan.optional_win[0] && plan.optional_win[1] && plan.optional_win[2] && plan.optional_win[3], planComplete(hs), planDead(hs)).enabled) {
      return;                   // the car side reports it is running what the label says already
    }
    ab_pin_b_armed = -1.0;
    tests.runOps(startOps(ab_configured), [this]() { abSave(); });
    abPoll();
    // what the counters are now: the drive's own additions are shown against this ("since START")
    const std::vector<int> scope = stepScope();
    for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
      drive_base_win[w] = tests.windowCredited(scope, w);
    }
    bool known = false;
    double target = 0;
    abDisplay(drive_base_a, drive_base_b, known, target);
    drive_base_set = true;
  }
}

// The set-up page: reads the stored set-up, edits it and stores it at once (TiAbConfig, persistent). Only while parked and
// nothing is running. The switches (TiAbOn, TiPlanOn: tmpfs) are never touched here: they are off after every boot.
void TestsWindow::setupOpen() {
  abOpen();
}

void TestsWindow::setupAct(int control) {
  using namespace ti_pages;
  if (control != CTL_SETUP_RESET || running()) {
    return;
  }
  // a deliberate act: asks first, then the stored progress goes
  if (ConfirmationDialog::confirm(QString("Reset the A/B progress (minutes per side) to zero? This cannot be undone."), QString("Reset"), this)) {
    ab_params.remove(TI_AB_PROGRESS_KEY);
    ab_stored_known = false;
    ab_stored_a = ab_stored_b = 0;
  }
  idle.start();
  opened.start();                       // the dialog's own taps end here
}

// A control acts on release, and only when press and release are on the same control
void TestsWindow::mouseReleaseEvent(QMouseEvent *event) {
  using namespace ti_pages;
  idle.start();
  const int control = controlAt(event->pos());
  const int pressed = pressed_control;
  pressed_control = 0;
  if (control == 0 || control != pressed) {
    return;
  }

  if (control == CTL_BACK) {
    if (home_page) {
      if (ti_logic::backAllowed(locked)) {  // locked: unlock first (the LOCK button), so a stray tap cannot dismiss the page
        leave();
      }
      return;
    }
    goHome();                               // the other pages: back to the first page (allowed while locked: the page stays)
    refresh();
    return;
  }
  if (control == CTL_NAV_CRUISE || control == CTL_CRUISE_RUN || control == CTL_CRUISE_STOP) {
    cruiseAct(control);
    refresh();
    return;
  }
  // While the car reports the cruise check running, nothing else starts: START, the A/B switch and the test switches wait (the
  // car side also aborts the check if one comes on). STOP and the off controls still act, and STOP stops the check too.
  if (cruiseBusy(cruise_view) && (control == CTL_START || control == CTL_AB_SWITCH || (control >= 10 && control < 20))) {
    refresh();
    return;
  }
  if (control == CTL_STOP && cruiseBusy(cruise_view)) {
    cruiseWrite("stop");
  }
  if (control == CTL_START || control == CTL_STOP || control == CTL_NAV_SETUP || control == CTL_NAV_PROGRESS || control == CTL_NAV_INDIV) {
    homeAct(control);
    refresh();
    return;
  }
  if (control >= CTL_SETUP_USE && control <= CTL_SETUP_RESET) {
    setupAct(control);
    refresh();
    return;
  }
  if (control == 8 || control == 9 || control == CTL_AB_SWITCH || control == CTL_PIN_A || control == CTL_PIN_B || (control == 2 && ab_page)) {
    abAct(control);       // the A/B switch page: its red box turns A/B off, not the tests
    refresh();
    return;
  }
  if (control == 2) {
    tests.allOff();
    refresh();
    return;
  }
  if (control == 5) {
    locked = !locked;
    tests.setLocked(locked);
    reopen.reset();
    refresh();
    return;
  }
  if (control == 6) {
    tests.openOverview();
    overview = true;
    refresh();
    return;
  }
  if (control == 7) {
    tests.closeOverview(queue_pos);
    overview = false;
    refresh();
    return;
  }
  if (control == 3 || control == 4) {
    const int to = control == 3 ? queue_pos - 1 : queue_pos + 1;
    if (tests.moveQueue(queue_pos, to)) {
      queue_pos = to;
    }
    refresh();
    return;
  }
  const int i = control >= 20 ? control - 20 : control - 10;
  if (control < 20) {
    if (overview && i == (int)tests.defs.size()) {
      tests.toggleAll();
      refresh();
      return;
    }
    if (i < 0 || i >= tests.queueSize() || i != queue_pos || overview) {
      return;
    }
    tests.toggle(i);
    refresh();
    return;
  }
  if (i < 0 || i >= (int)tests.defs.size() || i != queue_pos) {
    return;
  }
  const QString prompt = QString("Reset the time collected for \"%1\"? This cannot be undone.").arg(tests.defs[i].title);
  if (ConfirmationDialog::confirm(prompt, QString("Reset"), this)) {
    tests.resetProgress(i);
  }
  idle.start();
  opened.start();  // the dialog's own taps end here: no stray input right after it closes
  refresh();
}

// ---- the A/B page (selfdrive/car/mazda/ti_ab.py; logic in ti_ab_logic.h) -----------------------------------------------
// A page of the same window, beside the test pages and under the same page lock. It writes the configuration (TiAbConfig,
// persistent) and the switch and pin (TiAbOn, TiAbPin: tmpfs, off at every boot) and reads what the car side publishes
// (TiAbStatus, tmpfs). It never writes a tuning setting.


// The owner's value of setting i as the controller reads it (frogpilot_toggles); false when absent or unreadable
bool TestsWindow::abStoredValue(int i, double &value) const {
  const QJsonObject &toggles = frogpilotUIState()->frogpilot_toggles;
  const ti_ab::Setting &s = ti_ab::SETTINGS[i];
  const QJsonValue v = toggles.value(s.toggle);
  if (s.is_bool) {
    if (!v.isBool()) {
      return false;
    }
    value = v.toBool() ? 1.0 : 0.0;
    return true;
  }
  const QJsonArray kp = v.toArray();
  if (kp.size() != 2 || kp[0].toArray().size() != 1 || kp[1].toArray().size() != 1 || !kp[1].toArray()[0].isDouble()) {
    return false;
  }
  value = kp[1].toArray()[0].toDouble();
  return true;
}

// Read the saved configuration (the setting, B and the block length) and fix B to something the car side accepts
// Reads and checks the shipped plan (the file names the experiment; the code and the driver do not)
void TestsWindow::loadPlan() {
  plan = ti_pages::PlanFields();
  plan_ok = false;
  plan_name = plan_date = QString();
  QFile f(TI_DRIVE_PLAN_FILE);
  QJsonObject o;
  if (f.open(QIODevice::ReadOnly)) {
    o = QJsonDocument::fromJson(f.readAll()).object();
  }
  if (o.isEmpty()) {
    plan_problem_text = QString("plan file missing or unreadable");
    return;
  }
  plan.named = o.value("name").isString() && !o.value("name").toString().trimmed().isEmpty();
  plan_name = o.value("name").toString();
  plan_date = o.value("date").toString();
  for (const QJsonValue &v : o.value("tests").toObject().value("ids").toArray()) {
    plan.test_ids.push_back(v.toString().toStdString());
  }
  plan.reasons_ok = true;
  plan_reasons = QStringList();
  const auto note = [&](const QString &what, const QJsonObject &e) {
    for (const char *key : {"reason", "minimum", "buffer"}) {
      if (!e.value(key).isString() || e.value(key).toString().trimmed().isEmpty()) {
        plan.reasons_ok = false;
      }
    }
    plan_reasons << QString("%1: %2 Minimum: %3 Buffer: %4").arg(what, e.value("reason").toString(), e.value("minimum").toString(), e.value("buffer").toString());
  };
  for (const QJsonValue &v : o.value("tests").toObject().value("windows").toArray()) {
    const QJsonObject e = v.toObject();
    plan.windows.push_back(e.value("window").toString().toStdString());
    if (e.contains("optional")) {
      if (!e.value("optional").isBool()) {
        plan.optional_bad = true;
      } else {
        const int wi = (int)plan.windows.size() - 1;
        if (wi >= 0 && wi < ti_logic::N_WINDOWS) {
          plan.optional_win[wi] = e.value("optional").toBool();
        }
      }
    }
    if (!e.value("reason").toString().startsWith("Same as")) {
      note(QString("Tests at %1 mph (%2)").arg(e.value("window").toString(), e.value("amount").toString()), e);
    } else {
      note(QString("Tests at %1 mph").arg(e.value("window").toString()), e);
      plan_reasons.removeLast();      // unchanged windows are one line below, not a paragraph each
    }
  }
  const QJsonValue alt = o.value("alternation");
  if (alt.isObject()) {
    const QJsonObject a = alt.toObject();
    plan.has_alt = true;
    plan.setting = a.value("setting").toString().toStdString();
    plan.a = a.value("a").toString().toStdString();
    plan.b_is_number = a.value("b").isDouble();
    plan.b = a.value("b").toDouble();
    plan.b_text = a.value("b").toString().toStdString();
    plan.block_min = a.value("block_min").toDouble();
    plan.target_min = a.value("target_min_per_side").toDouble();
    note(QString("Alternation, %1 min per side").arg((int)a.value("target_min_per_side").toDouble()), a);
    plan.min_mph = a.value("min_mph").toDouble();
    plan.max_mph = a.value("max_mph").toDouble();
  }
  const std::string why = ti_pages::planProblem(plan);
  plan_problem_text = QString::fromStdString(why);
  plan_ok = why.empty();
}

// What START will run, from the plan: the setting, B and the block length as the plan names them, checked against the owner's
// stored value A the way the car side checks them. A plan that does not pass is invalid and nothing runs.
void TestsWindow::abOpen() {
  loadPlan();
  ab_configured = false;
  ab_setting = 0;
  ab_block_min = ti_ab::DEFAULT_BLOCK_MIN;
  if (plan_ok && plan.has_alt) {
    ab_setting = ti_ab::settingIndex(plan.setting.c_str());
    ab_block_min = plan.block_min;
    const ti_ab::Setting &s = ti_ab::SETTINGS[ab_setting];
    ab_b = s.is_bool ? 0.0 : plan.b;
    abFixB(!s.is_bool);
    if (!ab_a_known) {
      plan_ok = false;
      plan_problem_text = QString("your value cannot be read");
    } else if (!s.is_bool && std::fabs(ab_b - plan.b) > 1e-9) {
      plan_ok = false;
      plan_problem_text = QString("B not allowed for your value");
    } else {
      ab_configured = true;
    }
  }
  abPoll();
}

// B for the shown setting: kept when the car side would accept it with the owner's stored value, else the first offer
void TestsWindow::abFixB(bool keep) {
  ab_a_known = abStoredValue(ab_setting, ab_a);
  const ti_ab::Setting &s = ti_ab::SETTINGS[ab_setting];
  if (!ab_a_known) {
    return;
  }
  if (!(keep && ti_ab::bAllowed(s, ab_a, ab_b))) {
    ab_b = ti_ab::defaultB(s, ab_a);
  }
}

void TestsWindow::abSave() {
  const ti_ab::Setting &s = ti_ab::SETTINGS[ab_setting];
  QJsonObject cfg;
  cfg["setting"] = QString(s.name);
  if (s.is_bool) {
    cfg["b"] = ab_b != 0.0;
  } else {
    cfg["b"] = ab_b;
  }
  cfg["block_min"] = ab_block_min;
  ab_params.put(TI_AB_CONFIG_KEY, QJsonDocument(cfg).toJson(QJsonDocument::Compact).toStdString());
}

// What the car side says now, and the switch and pin as they are in tmpfs
void TestsWindow::abPoll() {
  ab_on = ab_memory.getBool(TI_AB_ON_KEY);
  ab_plan_on = ab_memory.getBool(TI_PLAN_ON_KEY);
  const QJsonObject hold = QJsonDocument::fromJson(QByteArray::fromStdString(ab_memory.get(TI_HOLD_KEY))).object();
  ab_hold_fresh = ti_ab::statusFresh(QDateTime::currentMSecsSinceEpoch() / 1000.0, hold.value("t").toDouble(-1e9));
  ab_hold_why = (ab_hold_fresh && hold.value("held").toBool()) ? hold.value("why").toString() : QString();
  const std::string pin = ab_memory.get(TI_AB_PIN_KEY);
  ab_pin = pin == "A" ? ti_ab::Pin::A : (pin == "B" ? ti_ab::Pin::B : ti_ab::Pin::None);
  ab_status = QJsonDocument::fromJson(QByteArray::fromStdString(ab_memory.get(TI_AB_STATUS_KEY))).object();
  ab_fresh = ti_ab::statusFresh(QDateTime::currentMSecsSinceEpoch() / 1000.0, ab_status.value("t").toDouble(-1e9));
  if (!ab_on) {
    ab_a_known = abStoredValue(ab_setting, ab_a);   // follows what he stores while the page is open and A/B is off
  }
  // the persistent progress of the experiment set up now (setting, A and B must match: another experiment's record is not shown)
  const QJsonObject stored = QJsonDocument::fromJson(QByteArray::fromStdString(ab_params.get(TI_AB_PROGRESS_KEY))).object();
  const QJsonObject sc = stored.value("committed").toObject();
  ab_stored_known = ab_configured && ab_a_known && stored.value("setting").toString() == QString(ti_ab::SETTINGS[ab_setting].name) &&
                    std::fabs(stored.value("a").toDouble(-1e9) - ab_a) < 1e-3 && std::fabs(stored.value("b").toDouble(-1e9) - ab_b) < 1e-3 &&
                    sc.value("A").isDouble() && sc.value("B").isDouble();
  ab_stored_a = ab_stored_known ? sc.value("A").toDouble() : 0.0;
  ab_stored_b = ab_stored_known ? sc.value("B").toDouble() : 0.0;
}

// The alternation's scored seconds per side to show: what the car side reports now, or the stored record when that is more
// (stopped, or a new process that has not counted yet). known: either exists. target_min: the plan's, else the default.
void TestsWindow::abDisplay(double &a_s, double &b_s, bool &known, double &target_min) {
  a_s = ab_stored_known ? ab_stored_a : 0.0;
  b_s = ab_stored_known ? ab_stored_b : 0.0;
  known = ab_stored_known;
  target_min = ti_ab::PLAN_TARGET_MIN_DEFAULT;
  if (ab_fresh && ab_configured) {
    const QJsonObject scored = ab_status.value("scored").toObject();
    a_s = std::max(a_s, scored.value("A").toDouble());
    b_s = std::max(b_s, scored.value("B").toDouble());
    known = true;
    const double t = ab_status.value("target_s").toDouble(0.0);
    const QJsonObject car_plan = ab_status.value("plan").toObject();
    target_min = car_plan.value("target_min").toDouble(t > 0 ? t / 60.0 : ti_ab::PLAN_TARGET_MIN_DEFAULT);
  }
}

// The step tests (sweep, pulses, ladder), whose windows the run-all mode fills
std::vector<int> TestsWindow::stepScope() const {
  std::vector<int> scope;
  for (int i = 0; i < (int)tests.defs.size(); i++) {
    if (tests.defs[i].metric == "step_seconds") {
      scope.push_back(i);
    }
  }
  return scope;
}

void TestsWindow::abSetOn(bool on) {
  if (on && !ab_configured) {
    return;                                  // nothing is set up: SET UP A/B on the first page
  }
  ab_memory.putBool(TI_PLAN_ON_KEY, false);   // the plain A/B switch and the drive plan exclude each other
  if (on) {
    tests.allOff();                          // one injection at a time: the tests go off before A/B starts
    abSave();
    ab_memory.remove(TI_AB_PIN_KEY);
  }
  ab_memory.putBool(TI_AB_ON_KEY, on);
  if (!on) {
    ab_memory.remove(TI_AB_PIN_KEY);
  }
  abPoll();
}

// The A/B switch page (a diagnostic page): 8 opens it, 9 leaves it, 2 / 30 switch A/B off / on, 31 / 32 the pins.
// The set-up (which setting, B, block length) is made on the set-up page.
void TestsWindow::abAct(int control) {
  if (control == 8) {
    abOpen();
    ab_page = true;
  } else if (control == 9) {
    ab_page = false;
  } else if (control == 2 || control == 30) {   // A/B OFF (the red box), or the on/off box
    abSetOn(control == 30 && !ab_on);
  } else if (control == 31 || control == 32) {
    if (ab_on) {                                // a pin means nothing while A/B is off
      const ti_ab::Pin next = ti_ab::tapPin(ab_pin, control == 31 ? ti_ab::Pin::A : ti_ab::Pin::B);
      const double now = QDateTime::currentMSecsSinceEpoch() / 1000.0;
      if (next == ti_ab::Pin::B && !ti_ab::pinBConfirmed(now, ab_pin_b_armed)) {
        ab_pin_b_armed = now;                       // the first tap on PIN B only arms it
      } else if (next == ti_ab::Pin::None) {
        ab_pin_b_armed = -1.0;
        ab_memory.remove(TI_AB_PIN_KEY);
      } else {
        ab_pin_b_armed = -1.0;
        ab_memory.put(TI_AB_PIN_KEY, ti_ab::pinText(next));
      }
    }
  }
  abPoll();
}

// One line at the bottom of both pages: what the drive plan runs now, what is next, the alternation's minutes against target
void TestsWindow::drawPlanLine(QPainter &p) {
  const QJsonObject car_plan = ab_fresh ? ab_status.value("plan").toObject() : QJsonObject();
  const QString mode = (ab_plan_on && !car_plan.isEmpty()) ? car_plan.value("mode").toString() : QString("off");
  const QJsonObject scored = ab_status.value("scored").toObject();
  const std::string line = ti_ab::planLine(mode.toStdString().c_str(), scored.value("A").toDouble(), scored.value("B").toDouble(),
                                           car_plan.value("target_min").toDouble(ti_ab::PLAN_TARGET_MIN_DEFAULT),
                                           ab_status.value("complete").toBool());
  p.setPen(QColor(0xc9, 0xc9, 0xc9));
  p.setFont(InterFont(34));
  p.drawText(QRect(30, height() - ti_pages::BOTTOM_STRIP, width() - 580, 64), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(line));
  if (!ab_hold_why.isEmpty() && ti_ab::holdText(ab_hold_why.toStdString().c_str())[0] != 0) {
    p.setPen(QColor(218, 202, 37));                         // why the tests are not running now, on both pages
    p.drawText(QRect(30, height() - ti_pages::BOTTOM_STRIP - 50, width() - 580, 50), Qt::AlignLeft | Qt::AlignVCenter,
               QString(ti_ab::holdText(ab_hold_why.toStdString().c_str())));
  }
}

// A box with a label and, under it, a smaller line (the first page's controls)
static void drawBox2(QPainter &p, const QRect &r, const QColor &fill, const QColor &ink, const QString &label, int label_px,
                     const QString &sub, int sub_px) {
  p.setPen(Qt::NoPen);
  p.setBrush(fill);
  p.drawRoundedRect(r, 24, 24);
  p.setPen(ink);
  if (sub.isEmpty()) {
    p.setFont(InterFont(label_px, QFont::Bold));
    p.drawText(r, Qt::AlignCenter, label);
    return;
  }
  const int sub_h = sub_px + 30;
  p.setFont(InterFont(label_px, QFont::Bold));
  p.drawText(QRect(r.x(), r.y(), r.width(), r.height() - sub_h), Qt::AlignCenter, label);
  p.setFont(InterFont(sub_px, QFont::DemiBold));
  p.drawText(QRect(r.x() + 10, r.bottom() - sub_h - 20, r.width() - 20, sub_h), Qt::AlignCenter, sub);
}

static QColor toneQColor(ti_pages::Tone t) {
  const ti_pages::ToneColor c = ti_pages::toneColor(t);
  return QColor(c.r, c.g, c.b);
}

// What the first page is told: from the params, the car side's status and the tests' own progress (nothing computed here that
// the Tests page does not already show)
ti_pages::HomeState TestsWindow::homeState(TiHomeStrings &keep) {
  using namespace ti_pages;
  HomeState h;
  h.run = ab_plan_on ? Run::Plan : (ab_on ? Run::AbPlain : (tests.all_view.requested ? Run::All : (tests.singleTestOn() ? Run::Single : Run::Stopped)));
  h.configured = ab_configured;
  const UIStatus st = uiState()->status;
  h.engaged = st != STATUS_DISENGAGED && st != STATUS_OVERRIDE;
  for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
    h.tests_optional[w] = plan_ok && plan.optional_win[w];
  }
  h.metric = is_metric;
  h.have_speed = have_speed;
  h.mph = speed_mph;

  const QJsonObject car_plan = ab_fresh ? ab_status.value("plan").toObject() : QJsonObject();
  keep.mode = (ab_plan_on && !car_plan.isEmpty()) ? car_plan.value("mode").toString().toStdString() : std::string();
  keep.why = ab_hold_why.toStdString();
  keep.state = ab_fresh ? ab_status.value("state").toString().toStdString() : std::string();
  keep.arm = ab_fresh ? ab_status.value("arm").toString().toStdString() : std::string();
  h.plan_mode = keep.mode.c_str();
  h.hold_why = keep.why.c_str();
  h.ab_fresh = ab_fresh;
  h.ab_state = keep.state.c_str();
  h.ab_arm = keep.arm.c_str();
  const QJsonObject block = ab_fresh ? ab_status.value("block").toObject() : QJsonObject();
  h.ab_block = !block.isEmpty();
  h.ab_block_left_s = block.value("left_s").toDouble();
  const QJsonObject wait = ab_fresh ? ab_status.value("wait").toObject() : QJsonObject();
  h.ab_wait = !wait.isEmpty();
  keep.reason = wait.value("reason").toString().toStdString();
  h.ab_wait_reason = keep.reason.c_str();
  h.ab_wait_s = wait.value("s").toDouble();
  h.ab_wait_max_s = wait.value("max_s").toDouble();
  h.ab_complete = ab_fresh && ab_status.value("complete").toBool();
  h.car_plan_on = ab_fresh && ab_status.value("plan").toObject().value("on").toBool();
  h.live_fresh = tests.live_fresh;
  h.live_all_on = tests.live_all_on;
  h.live_tests_on = tests.live_tests_on;

  std::vector<int> scope;
  for (int i = 0; i < (int)tests.defs.size(); i++) {
    if (tests.defs[i].metric == "step_seconds") {
      scope.push_back(i);
    }
  }
  if (have_speed && !scope.empty()) {
    const ti_logic::Target t = tests.target(scope, speed_mph);
    h.target = t.window >= 0;
    h.target_window = t.window;
    h.target_secs = t.secs;
  }
  bool active = tests.all_view.active;
  for (const TiTestView &view : tests.views) {
    active = active || view.active;
  }
  h.tests_active = active;
  keep.gate = tests.gate.valid ? tests.gate.reason.toStdString() : std::string();
  h.gate_reason = keep.gate.c_str();
  for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
    int done_n = 0, total_n = 0;
    tests.windowShare(scope, w, done_n, total_n, h.tests_left_s[w]);
  }
  bool known = false;
  abDisplay(h.ab_stored_a_s, h.ab_stored_b_s, known, h.target_min);
  h.ab_stored_known = known;
  return h;
}

// Colour of the engaged badge: the same status the road view colours its border by (green engaged, amber override, grey not)
void TestsWindow::drawEngagedBadge(QPainter &p, const QRect &pill) {
  const QColor amber(218, 202, 37);
  const UIStatus st = uiState()->status;
  const bool engaged = st != STATUS_DISENGAGED && st != STATUS_OVERRIDE;
  drawBox(p, pill, engaged ? QColor(0x17, 0x86, 0x44) : st == STATUS_OVERRIDE ? amber : QColor(0x54, 0x54, 0x54),
          (engaged || st != STATUS_OVERRIDE) ? QColor(0xff, 0xff, 0xff) : QColor(0, 0, 0),
          engaged ? QString("● ENGAGED") : st == STATUS_OVERRIDE ? QString("OVERRIDE") : QString("○ NOT ENGAGED"), 40);
}

void TestsWindow::drawTitle(QPainter &p, const QRect &r, const QString &text, int px) {
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setFont(InterFont(px, QFont::Bold));
  p.drawText(r, Qt::AlignCenter, text);
}

// The first page: one status line, one number, what will run, START and STOP, and (small, not needed while driving) the detail
void TestsWindow::drawHome(QPainter &p) {
  using namespace ti_pages;
  const QColor white(0xff, 0xff, 0xff);
  const QColor grey(0x33, 0x33, 0x33);
  TiHomeStrings keep;
  const HomeState hs = homeState(keep);
  const HomeLines lines = home(hs);

  // The status band: its colour is the state (the same colours on every page), its words say the same
  const ToneColor tc = toneColor(lines.primary.tone);
  const QColor ink = tc.light ? QColor(0, 0, 0) : white;
  const QRect band(HOME_PRIMARY.x, HOME_PRIMARY.y, HOME_PRIMARY.w, HOME_SECOND.bottom() - HOME_PRIMARY.y);
  p.setPen(Qt::NoPen);
  p.setBrush(toneQColor(lines.primary.tone));
  p.drawRoundedRect(band, 24, 24);
  p.setPen(ink);
  p.setFont(InterFont(PRIMARY_PX, QFont::Bold));
  p.drawText(qr(HOME_PRIMARY).adjusted(24, 0, -24, 0), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(lines.primary.text));
  p.setFont(InterFont(SECOND_PX, QFont::DemiBold));
  p.drawText(qr(HOME_SECOND).adjusted(24, 0, -24, 0), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(lines.second));

  // What will run, in plain words
  const std::string other_mode = (carRunning(hs) && otherModeOn(hs)) ? otherModeLine(hs) : std::string();
  p.setPen((plan_ok && other_mode.empty()) ? white : QColor(218, 202, 37));
  p.setFont(InterFont(CHOSEN_PX, QFont::DemiBold));
  p.drawText(qr(HOME_EXPERIMENT).adjusted(24, 0, 0, 0), Qt::AlignLeft | Qt::AlignVCenter,
             !other_mode.empty() ? QString::fromStdString(other_mode) :
             QString::fromStdString(planLine(plan_ok, plan_problem_text.toStdString(), plan.has_alt, plan.has_alt ? plan.setting.c_str() : "", !(plan.optional_win[0] && plan.optional_win[1] && plan.optional_win[2] && plan.optional_win[3]))));

  // The bands still needed: one speed and the minutes left, tests and alternation merged. The same model as the status line above.
  Band bands[MAX_BANDS];
  const int n_bands = bandsFromState(hs, bands);
  const int counting = countingBand(hs, bands, n_bands);
  const StartLook look = startLook(ab_configured, hs.run, carRunning(hs), plan_ok, plan.optional_win[0] && plan.optional_win[1] && plan.optional_win[2] && plan.optional_win[3], planComplete(hs), planDead(hs));
  // The middle of the page: START while nothing runs, while the plan has stopped itself, or while another mode is on (START replaces
  // it); the plan's complete message when it is done; otherwise the bands still needed. A stopped plan never lists bands.
  const bool show_box = !carRunning(hs) || planDead(hs) || otherModeOn(hs);
  const bool show_complete = !show_box && planComplete(hs);
  const bool show_bands = !show_box && !show_complete;
  if (show_complete) {
    const QRect area = ctlRect(CTL_START);
    const std::vector<std::string> msg = completeLines(hs);
    p.setPen(QColor(0x33, 0xcc, 0x66));
    p.setFont(InterFont(BAND_PX + 16, QFont::Bold));
    p.drawText(QRect(area.x(), area.y(), area.width(), 90), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(msg[0]));
    p.setPen(white);
    p.setFont(InterFont(BAND_PX, QFont::Bold));
    p.drawText(QRect(area.x(), area.y() + 96, area.width(), 76), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(msg[1]));
    p.setPen(QColor(0xc9, 0xc9, 0xc9));
    p.setFont(InterFont(EXPERIMENT_PX));
    p.drawText(QRect(area.x(), area.y() + 180, area.width(), 56), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(msg[2]));
    p.drawText(QRect(area.x(), area.y() + 236, area.width(), 56), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(msg[3]));
  }
  if (show_bands) {
    const QRect area = ctlRect(CTL_START);
    for (int i = 0; i < n_bands; i++) {
      const QRect row(area.x(), area.y() + i * 70, area.width(), 66);
      const bool inside = bandIndexAt(bands, n_bands, hs.mph) == i && hs.have_speed;
      p.setPen(Qt::NoPen);
      // green: time is counting in this band now; amber: the car is in it but nothing counts yet; dim green: done
      p.setBrush(i == counting ? QColor(0x17, 0x86, 0x44) : (inside ? QColor(0x6b, 0x5e, 0x10) : (bands[i].done ? QColor(0x1f, 0x3a, 0x2a) : QColor(0x22, 0x22, 0x22))));
      p.drawRoundedRect(row, 18, 18);
      p.setPen(white);
      p.setFont(InterFont(BAND_PX, QFont::Bold));
      p.drawText(row.adjusted(24, 0, -420, 0), Qt::AlignLeft | Qt::AlignVCenter,
                 (bands[i].done ? QString("✓ ") : QString()) + QString::fromStdString(bandText(bands[i], is_metric)));
      p.setPen(QColor(0xc9, 0xc9, 0xc9));
      p.setFont(InterFont(DETAIL_PX));
      p.drawText(row.adjusted(0, 0, -24, 0), Qt::AlignRight | Qt::AlignVCenter, QString::fromStdString(bandRange(bands[i], is_metric)));
    }
    p.setPen(QColor(0xc9, 0xc9, 0xc9));
    p.setFont(InterFont(DETAIL_PX));
    if (bandsTotalMinutes(bands, n_bands) > 0) {
      p.drawText(QRect(area.x(), area.y() + n_bands * 70 + 4, area.width(), 44), Qt::AlignLeft | Qt::AlignVCenter,
                 QString::fromStdString(std::string("About ") + std::to_string((int)bandsTotalMinutes(bands, n_bands)) + " min of driving left in all"));
    }
  }

  // START (what it will run, and what it replaces), STOP
  if (show_box) {
    drawBox2(p, ctlRect(CTL_START), look.enabled ? toneQColor(Tone::Done) : (plan_ok && look.tone != Tone::Held ? QColor(0x0f, 0x4d, 0x28) : QColor(0x6b, 0x4a, 0x1d)), white,
             QString::fromStdString(look.label), START_PX, QString::fromStdString(look.sub), START_SUB_PX);
  }
  const bool any = hs.run != Run::Stopped;
  drawBox2(p, ctlRect(CTL_STOP), any ? QColor(201, 34, 49) : QColor(0x6b, 0x1d, 0x25), any ? white : QColor(0xc9, 0xc9, 0xc9),
           QString("STOP"), STOP_PX, QString("stops every test and the A/B relay"), START_SUB_PX);

  // Navigation at the screen edge
  drawBox(p, ctlRect(CTL_BACK), locked ? QColor(0x1a, 0x1a, 0x1a) : grey, locked ? QColor(0x55, 0x55, 0x55) : white, QString("< EXIT"), NAV_PX);
  drawBox(p, ctlRect(CTL_NAV_SETUP), grey, white, QString("PLAN\nDETAILS >"), NAV_PX);
  drawBox(p, ctlRect(CTL_NAV_PROGRESS), grey, white, QString("PROGRESS >"), NAV_PX);
  drawEngagedBadge(p, qr(HOME_BADGE));

  // Detail (not needed while driving). Line 1: what this drive has added since START. Line 2: why the tests are held, or the
  // alternation's minutes against its target.
  QString detail1;
  if (hs.run == Run::Stopped && plan_ok) {
    detail1 = QString::fromStdString(budgetLine(bands, n_bands, is_metric));          // what the whole plan needs of one drive
  }
  if (drive_base_set && hs.run != Run::Stopped) {
    const std::vector<int> scope = stepScope();
    double win[ti_logic::N_WINDOWS];
    for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
      win[w] = std::max(0.0, tests.windowCredited(scope, w) - drive_base_win[w]);
    }
    double a_s = 0, b_s = 0, target = 0;
    bool known = false;
    abDisplay(a_s, b_s, known, target);
    detail1 = QString::fromStdString(thisDriveLine(win, std::max(0.0, a_s - drive_base_a), std::max(0.0, b_s - drive_base_b), ab_configured, is_metric));
  }
  p.setPen(QColor(0xc9, 0xc9, 0xc9));
  p.setFont(InterFont(DETAIL_PX));
  p.drawText(qr(HOME_DETAIL), Qt::AlignLeft | Qt::AlignVCenter, detail1);
  QString detail2;
  QColor detail_color(0xc9, 0xc9, 0xc9);
  if (!ab_hold_why.isEmpty() && ti_ab::holdText(ab_hold_why.toStdString().c_str())[0] != 0) {
    detail2 = QString(ti_ab::holdText(ab_hold_why.toStdString().c_str()));
    detail_color = QColor(218, 202, 37);
  } else if (ab_configured) {
    double a_s = 0, b_s = 0, target = 0;
    bool known = false;
    abDisplay(a_s, b_s, known, target);
    if (known) {
      detail2 = QString::fromStdString(detailScored(a_s, b_s, target, ab_fresh && ab_status.value("complete").toBool()));
    }
  }
  p.setPen(detail_color);
  p.drawText(qr(HOME_DETAIL2), Qt::AlignLeft | Qt::AlignVCenter, detail2);
}

// The plan's details, read-only: what START runs and what it needs. Nothing here is chosen on the screen.
void TestsWindow::drawSetup(QPainter &p) {
  using namespace ti_pages;
  const QColor white(0xff, 0xff, 0xff);
  const QColor grey(0x33, 0x33, 0x33);
  const QColor dim(0xc9, 0xc9, 0xc9);
  drawBox(p, ctlRect(CTL_BACK), grey, white, QString("< BACK"), NAV_PX);
  drawTitle(p, qr(TITLE_SETUP), QString("PLAN DETAILS"), 90);
  const bool can_reset = !running();
  drawBox(p, ctlRect(CTL_SETUP_RESET), can_reset ? grey : QColor(0x1a, 0x1a, 0x1a), can_reset ? white : QColor(0x55, 0x55, 0x55), QString("RESET\nPROGRESS"), NAV_PX);

  QStringList lines;
  lines << (plan_ok ? plan_name : QString("PLAN INVALID: %1").arg(plan_problem_text));
  if (plan_ok) {
    lines << QString("Date %1. Read from the build; nothing here is chosen on the screen.").arg(plan_date);
    lines << QString("Tests: sweep, pulses, ladder in the windows 21-27, 35-41, 49-56, 64-80 mph");
    if (plan.has_alt) {
      const ti_ab::Setting &s = ti_ab::SETTINGS[ab_setting];
      lines << QString::fromStdString(experimentLine(ab_setting, ab_a_known, ab_a, ab_b));
      lines << QString("Blocks of %1 min, %2 min per side to finish, at 36-55 mph (35-56 mph range)").arg((int)ab_block_min).arg((int)plan.target_min);
      if (ti_ab::bendsOnly(s.name)) {
        lines << QString("%1 acts only in bends: minutes count only there").arg(plainName(s.name));
      }
    } else {
      lines << QString("No alternation in this plan");
    }
    double a_s = 0, b_s = 0, target = 0;
    bool known = false;
    abDisplay(a_s, b_s, known, target);
    if (plan.has_alt && known) {
      lines << QString::fromStdString(abProgressText(a_s, b_s, target)) + QString("  (kept across restarts)");
    }
  }
  p.setFont(InterFont(EXPERIMENT_PX));
  for (int i = 0; i < lines.size(); i++) {
    p.setPen(i == 0 ? (plan_ok ? white : QColor(218, 202, 37)) : dim);
    p.drawText(QRect(480, 215 + i * 56, 1660, 52), Qt::AlignLeft | Qt::AlignVCenter, lines[i]);
  }
  // why each amount is what it is: reason, minimum, buffer (from the plan file), in plain words
  p.setPen(dim);
  p.setFont(InterFont(PLAN_NOTE_PX));
  p.drawText(QRect(480, 215 + lines.size() * 56 + 8, 1660, SCREEN_H - (215 + lines.size() * 56 + 8) - 10), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
             plan_reasons.join(QString("\n\n")));
}

void TestsWindow::drawBar(QPainter &p, const QRect &r, double fraction, const QColor &fill) {
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0x33, 0x33, 0x33));
  p.drawRoundedRect(r, 12, 12);
  if (fraction > 0) {
    p.setBrush(fill);
    p.drawRoundedRect(QRect(r.x(), r.y(), std::max(24, int(r.width() * std::min(1.0, fraction))), r.height()), 12, 12);
  }
}

// The progress page: the alternation's two sides against their target with the driving time left, and every test window with
// its share of required items done and the next thing wanted. Read while stopped; nothing here is needed while driving.
void TestsWindow::drawProgress(QPainter &p) {
  using namespace ti_pages;
  const QColor white(0xff, 0xff, 0xff);
  const QColor grey(0x33, 0x33, 0x33);
  const QColor blue(0x46, 0x5B, 0xEA);
  const QColor amber(218, 202, 37);
  const QColor green(0x17, 0x86, 0x44);
  drawBox(p, ctlRect(CTL_BACK), grey, white, QString("< START\nPAGE"), NAV_PX);
  drawBox(p, ctlRect(CTL_NAV_INDIV), grey, white, QString("INDIVIDUAL\nTESTS >"), NAV_PX);
  drawTitle(p, qr(PROG_TITLE), QString("PROGRESS"), 90);

  double a_s = 0, b_s = 0, target = 0;
  bool known = false;
  abDisplay(a_s, b_s, known, target);
  p.setPen(white);
  p.setFont(InterFont(PROGRESS_PX, QFont::Bold));
  if (ab_configured) {
    p.drawText(qr(PROG_AB_HEAD), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(abProgressText(a_s, b_s, target)));
    drawBar(p, qr(PROG_AB_BAR_A), fraction(a_s, 60.0 * target), blue);
    drawBar(p, qr(PROG_AB_BAR_B), fraction(b_s, 60.0 * target), amber);
    p.setPen(QColor(0xc9, 0xc9, 0xc9));
    p.setFont(InterFont(EXPERIMENT_PX));
    p.drawText(qr(PROG_AB_LEFT), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(abLeftText(a_s, b_s, target, OVERHEAD_ALT)));
  } else {
    p.drawText(qr(PROG_AB_HEAD), Qt::AlignLeft | Qt::AlignVCenter, QString("A/B relay: not used"));
  }

  const bool alternation_open = ab_configured && abMinutesLeft(a_s, b_s, target) > 0.0;
  p.setPen(amber);
  p.setFont(InterFont(DETAIL_PX, QFont::DemiBold));
  p.drawText(qr(PROG_SPLIT), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(splitNote(alternation_open, is_metric)));

  const std::vector<int> scope = stepScope();
  for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
    int done = 0, total = 0;
    double secs_left = 0;
    tests.windowShare(scope, w, done, total, secs_left);
    const int y = PROG_ROWS_TOP + w * PROG_ROW_H;
    p.setPen(white);
    p.setFont(InterFont(PROGRESS_PX, QFont::Bold));
    p.drawText(QRect(480, y, 400, 50), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(windowLabel(w, is_metric)));   // short: the bar starts at x=900 on the same row; "optional" is said in the line below
    drawBar(p, QRect(900, y + 8, 1240, 36), total > 0 ? double(done) / total : 1.0, done >= total ? green : blue);
    p.setPen(QColor(0xc9, 0xc9, 0xc9));
    p.setFont(InterFont(EXPERIMENT_PX));
    p.drawText(QRect(480, y + 54, 1660, 50), Qt::AlignLeft | Qt::AlignVCenter,
               QString::fromStdString(windowRowText(w, done, total, secs_left, is_metric, plan_ok && plan.optional_win[w])));
  }
}

// Draws the plain A/B switch page below the shared header; returns the banner line
QString TestsWindow::drawAb(QPainter &p) {
  using namespace ti_pages;
  const QColor white(0xff, 0xff, 0xff);
  const QColor dim_fill(0x1a, 0x1a, 0x1a);
  const QColor dim_ink(0x55, 0x55, 0x55);
  const QColor amber(218, 202, 37);
  const QColor blue(0x46, 0x5B, 0xEA);
  const QColor grey(0x33, 0x33, 0x33);
  const ti_ab::Setting &s = ti_ab::SETTINGS[ab_setting];
  const QString state = ab_fresh ? ab_status.value("state").toString() : QString("off");
  const bool running_now = ab_on && ab_fresh && (state == "running" || state == "pinned");
  const QString arm = ab_status.value("arm").toString();

  // the switch and the pins
  drawBox(p, ctlRect(CTL_AB_SWITCH), ab_on ? QColor(0x17, 0x86, 0x44) : QColor(201, 34, 49), white,
          ab_on ? QString("A/B ON\ntap to stop") : QString("A/B OFF\ntap to start"), ACTION_PX);
  drawBox(p, ctlRect(CTL_PIN_A), ab_pin == ti_ab::Pin::A ? blue : (ab_on ? grey : dim_fill), ab_on ? white : dim_ink,
          ab_pin == ti_ab::Pin::A ? QString("A PINNED\ntap to release") : QString("PIN A"), ACTION_PX);
  drawBox(p, ctlRect(CTL_PIN_B), ab_pin == ti_ab::Pin::B ? amber : (ab_on ? grey : dim_fill),
          ab_pin == ti_ab::Pin::B ? QColor(0, 0, 0) : (ab_on ? white : dim_ink),
          ab_pin == ti_ab::Pin::B ? QString("B PINNED\ntap to release")
                                  : (ti_ab::pinBConfirmed(QDateTime::currentMSecsSinceEpoch() / 1000.0, ab_pin_b_armed)
                                         ? QString("TAP AGAIN\nto pin B") : QString("PIN B")), ACTION_PX);

  // which value is in force, and its number
  const QRect panel = qr(AB_PANEL);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0x11, 0x11, 0x11));
  p.drawRoundedRect(panel, 24, 24);
  if (running_now && (arm == "A" || arm == "B")) {
    p.setPen(arm == "B" ? amber : blue);
    p.setFont(InterFont(130, QFont::Bold));
    p.drawText(QRect(panel.left() + 10, panel.top(), 130, panel.height()), Qt::AlignCenter, arm);
    p.setPen(white);
    p.setFont(InterFont(56, QFont::Bold));
    const QJsonValue value = ab_status.value("value");
    const QString number = value.isBool() ? (value.toBool() ? QString("ON") : QString("OFF")) : QString::number(value.toDouble(), 'f', 2);
    const double blend = ab_status.value("blend").toDouble();
    p.drawText(QRect(panel.left() + 150, panel.top() + 5, panel.width() - 160, 80), Qt::AlignLeft | Qt::AlignVCenter,
               number + (blend > 0.001 && blend < 0.999 ? QString(" (blending)") : QString()));
    p.setFont(InterFont(30));
    const QJsonObject block = ab_status.value("block").toObject();
    QString line;
    if (state == "pinned") {
      line = QString("Pinned: no alternation");
    } else if (!block.isEmpty()) {
      const int left = (int)std::ceil(block.value("left_s").toDouble());
      line = QString("Block %1: %2:%3 left").arg(block.value("idx").toInt() + 1).arg(left / 60).arg(left % 60, 2, 10, QChar('0'));
    } else {
      const QJsonObject wait = ab_status.value("wait").toObject();
      line = wait.isEmpty() ? QString("Starting the next block")
                            : QString("Switch to %1 waiting: %2 (%3 s of %4 s)").arg(wait.value("arm").toString())
                                  .arg(QString(ti_ab::reasonText(wait.value("reason").toString().toStdString().c_str())))
                                  .arg((int)wait.value("s").toDouble()).arg((int)wait.value("max_s").toDouble());
    }
    if (ab_status.value("wash_in").toBool()) {
      line += QString("  settling after a switch");
    }
    p.setPen(amber);
    p.drawText(QRect(panel.left() + 150, panel.top() + 85, panel.width() - 160, 110), Qt::AlignLeft | Qt::AlignVCenter | Qt::TextWordWrap, line);
  } else {
    p.setPen(QColor(0xc9, 0xc9, 0xc9));
    p.setFont(InterFont(34, QFont::DemiBold));
    const QString msg = ab_status.value("msg").toString();
    p.drawText(panel.adjusted(16, 0, -16, 0), Qt::AlignLeft | Qt::AlignVCenter | Qt::TextWordWrap,
               !ab_on ? QString("A/B is off: your own value is in force")
                      : (msg.isEmpty() ? QString("Waiting for engagement") : msg));
  }

  // engaged minutes per side and per speed window (the wash-in after each switch is not counted)
  const int table_top = AB_TABLE_TOP;
  const int row_h = std::max(32, std::min(60, (height() - 150 - table_top) / 6));
  p.setFont(InterFont(std::min(38, row_h * 7 / 10), QFont::Bold));
  p.setPen(white);
  p.drawText(QRect(30, table_top, 600, row_h), Qt::AlignLeft | Qt::AlignVCenter, QString("Engaged minutes"));
  p.setPen(blue);
  p.drawText(QRect(640, table_top, 220, row_h), Qt::AlignRight | Qt::AlignVCenter, QString("A"));
  p.setPen(amber);
  p.drawText(QRect(900, table_top, 220, row_h), Qt::AlignRight | Qt::AlignVCenter, QString("B"));
  const QJsonObject progress = ab_status.value("progress").toObject();
  p.setFont(InterFont(std::min(38, row_h * 7 / 10)));
  for (int k = 0; k <= ti_logic::N_WINDOWS; k++) {
    const QString label = k < ti_logic::N_WINDOWS ? QString(ti_logic::WINDOWS[k].label) : QString("gap");
    const int y = table_top + (k + 1) * row_h;
    p.setPen(white);
    p.drawText(QRect(30, y, 600, row_h), Qt::AlignLeft | Qt::AlignVCenter,
               k < ti_logic::N_WINDOWS ? label + QString(" mph") : QString("between windows"));
    for (int j = 0; j < 2; j++) {
      const double secs = progress.value(j == 0 ? "A" : "B").toObject().value(label).toDouble(0.0);
      p.setPen(j == 0 ? blue : amber);
      p.drawText(QRect(j == 0 ? 640 : 900, y, 220, row_h), Qt::AlignRight | Qt::AlignVCenter, QString::number(secs / 60.0, 'f', 1));
    }
  }

  // refusals and advice (the set-up is made on the first page's SET UP A/B)
  if (!ab_configured) {
    return QString("Not set up. Choose the setting on SET UP A/B (START page).");
  }
  if (!ab_on && !ab_a_known) {
    return QString("Your stored value of %1 cannot be read: A/B cannot start").arg(s.name);
  }
  if (ab_on && ab_status.value("msg").toString().size() > 0 && ab_fresh) {
    return ab_status.value("msg").toString();
  }
  if (ti_ab::bendsOnly(s.name)) {
    return QString("%1 acts only in bends: minutes count only there").arg(s.name);
  }
  return QString::fromStdString(experimentLine(ab_setting, ab_a_known, ab_a, ab_b));
}

// ---- the cruise set-speed check's page ---------------------------------------------------------------------------------------
// RUN CHECK writes one trigger (TiCruiseCheck, tmpfs) that the car controller consumes; STOP CHECK writes a stop. Everything shown
// comes from the car controller's report (TiCruiseCheckStatus); nothing here decides that the check runs.
const char *const TI_CRUISE_TRIGGER_KEY = "TiCruiseCheck";
const char *const TI_CRUISE_STATUS_KEY = "TiCruiseCheckStatus";

void TestsWindow::cruiseOpen() {
  home_page = false;
  setup_page = false;
  progress_page = false;
  ab_page = false;
  cruise_page = true;
}

void TestsWindow::cruisePoll() {
  ti_pages::CruiseView v;
  const QJsonObject st = QJsonDocument::fromJson(QByteArray::fromStdString(ab_memory.get(TI_CRUISE_STATUS_KEY))).object();
  v.have = st.value("state").isString();
  if (v.have) {
    const double age = QDateTime::currentMSecsSinceEpoch() / 1000.0 - st.value("t").toDouble(-1e9);
    v.fresh = age > -ti_pages::CRUISE_STALE_S && age < ti_pages::CRUISE_STALE_S;
    v.state = st.value("state").toString().toStdString();
    v.why = st.value("why").toString().toStdString();
    v.phase = st.value("phase").toString("A").toStdString();
    v.round = st.value("round").toInt();
    v.orig = st.value("orig").toInt();
    v.set = st.value("set").toInt();
    v.result = st.value("result").toString().toStdString();
    v.restore = st.value("restore").toString().toStdString();
    for (const char *key : {"a", "b"}) {
      std::vector<ti_pages::CruiseRound> &rows = key[0] == 'a' ? v.a : v.b;
      for (const QJsonValue &rv : st.value(key).toArray()) {
        const QJsonArray r = rv.toArray();
        rows.push_back(ti_pages::CruiseRound{r.at(0).toInt() != 0, r.at(1).toInt() != 0, r.at(2).toInt(), r.at(3).toInt()});
      }
    }
  }
  cruise_view = v;
}

// Why another mode keeps the check from starting, as the screen knows it ("" when nothing else is on); the car side checks again
std::string TestsWindow::cruiseOthers() const {
  if (ab_plan_on) {
    return "the drive plan is on";
  }
  if (ab_on) {
    return "the A/B switch is on";
  }
  if (tests.all_view.requested || tests.singleTestOn()) {
    return "a test is on";
  }
  return "";
}

void TestsWindow::cruiseWrite(const char *cmd) {
  QJsonObject t;
  const qint64 ms = QDateTime::currentMSecsSinceEpoch();
  t["cmd"] = QString(cmd);
  t["id"] = QString("%1-%2").arg(cmd).arg(ms);
  t["t"] = ms / 1000.0;
  ab_memory.put(TI_CRUISE_TRIGGER_KEY, QJsonDocument(t).toJson(QJsonDocument::Compact).toStdString());
}

void TestsWindow::cruiseAct(int control) {
  using namespace ti_pages;
  if (control == CTL_NAV_CRUISE) {
    cruiseOpen();
    return;
  }
  cruisePoll();
  const CruiseLook look = cruiseLook(cruise_view, cruiseOthers());
  if (control == CTL_CRUISE_RUN && look.run_enabled) {
    cruiseWrite("run");          // one press, one run: the car controller consumes the trigger
  } else if (control == CTL_CRUISE_STOP && look.stop_enabled) {
    cruiseWrite("stop");
  }
}

void TestsWindow::drawCruise(QPainter &p) {
  using namespace ti_pages;
  const QColor white(0xff, 0xff, 0xff);
  const QColor grey(0x33, 0x33, 0x33);
  const QColor dim(0xc9, 0xc9, 0xc9);
  const CruiseLook look = cruiseLook(cruise_view, cruiseOthers());
  drawBox(p, ctlRect(CTL_BACK), grey, white, QString("< START\nPAGE"), NAV_PX);
  drawTitle(p, qr(CRUISE_TITLE), QString("CRUISE SET-SPEED\nCHECK"), 64);
  drawBox2(p, ctlRect(CTL_CRUISE_RUN), look.run_enabled ? toneQColor(Tone::Done) : QColor(0x1a, 0x1a, 0x1a),
           look.run_enabled ? white : QColor(0x77, 0x77, 0x77), QString("RUN CHECK"), START_PX, QString::fromStdString(look.run_sub), START_SUB_PX);
  drawBox2(p, ctlRect(CTL_CRUISE_STOP), look.stop_enabled ? QColor(201, 34, 49) : QColor(0x6b, 0x1d, 0x25),
           look.stop_enabled ? white : QColor(0xc9, 0xc9, 0xc9), QString("STOP CHECK"), START_PX, QString::fromStdString(look.stop_sub), START_SUB_PX);

  // the status band: its colour is the state, its words say the same
  const ToneColor tc = toneColor(look.tone);
  const QColor ink = tc.light ? QColor(0, 0, 0) : white;
  p.setPen(Qt::NoPen);
  p.setBrush(toneQColor(look.tone));
  p.drawRoundedRect(qr(CRUISE_BAND), 24, 24);
  p.setPen(ink);
  p.setFont(InterFont(CRUISE_PRIMARY_PX, QFont::Bold));
  p.drawText(qr(CRUISE_PRIMARY).adjusted(24, 0, -24, 0), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(look.primary));
  p.setFont(InterFont(CRUISE_SECOND_PX, QFont::DemiBold));
  p.drawText(qr(CRUISE_SECOND).adjusted(24, 0, -24, 0), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(look.second));

  // the answer, large and plain
  if (!look.result.empty()) {
    const std::string &r = cruise_view.result;
    p.setPen(r == "YES" ? QColor(0x33, 0xcc, 0x66) : (r == "NO" ? QColor(0xff, 0x6b, 0x6b) : QColor(218, 202, 37)));
    p.setFont(InterFont(CRUISE_RESULT_PX, QFont::Bold));
    p.drawText(qr(CRUISE_RESULT).adjusted(24, 0, -24, 0), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(look.result));
  }
  p.setPen(dim);
  p.setFont(InterFont(DETAIL_PX));
  p.drawText(qr(CRUISE_DETAIL1), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(look.detail1));
  p.drawText(qr(CRUISE_DETAIL2), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(look.detail2));
  p.drawText(qr(CRUISE_DETAIL3), Qt::AlignLeft | Qt::AlignVCenter, QString::fromStdString(look.detail3));
  drawEngagedBadge(p, qr(HOME_BADGE));
}

// The first page while the check runs: START is covered by what it waits for (the car's report decides, never this page)
void TestsWindow::drawCruiseBusy(QPainter &p) {
  using namespace ti_pages;
  if (!cruiseBusy(cruise_view)) {
    return;
  }
  drawBox2(p, ctlRect(CTL_START), QColor(0x6b, 0x5e, 0x10), QColor(0xff, 0xff, 0xff), QString("CRUISE CHECK RUNNING"), START_PX,
           QString("START waits until it ends (PROGRESS > CRUISE CHECK)"), START_SUB_PX);
}

void TestsWindow::paintEvent(QPaintEvent *event) {
  using namespace ti_pages;
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.fillRect(rect(), QColor(0, 0, 0));

  const QColor white(0xff, 0xff, 0xff);
  const QColor dim_fill(0x1a, 0x1a, 0x1a);
  const QColor dim_ink(0x55, 0x55, 0x55);
  const QColor amber(218, 202, 37);
  const QColor grey(0x33, 0x33, 0x33);

  // The lock, always visible and at the same place on the same row of every page: amber and solid when locked (the page stays
  // until it is unlocked), plain when not
  drawBox(p, lockRect(), locked ? amber : grey, locked ? QColor(0, 0, 0) : white,
          locked ? QString("LOCKED\ntap to unlock") : QString("LOCK PAGE"), NAV_PX);
  const auto lockBorder = [&]() {
    if (locked) {
      QPen pen(amber);
      pen.setWidth(10);
      p.setPen(pen);
      p.setBrush(Qt::NoBrush);
      p.drawRect(rect().adjusted(5, 5, -5, -5));
    }
  };

  if (cruise_page) {
    drawCruise(p);
    lockBorder();
    return;
  }
  if (home_page) {
    drawHome(p);
    drawCruiseBusy(p);         // START waits while the cruise check runs: said where START is
    lockBorder();
    return;
  }
  if (setup_page) {
    drawSetup(p);
    lockBorder();
    return;
  }
  if (progress_page) {
    drawProgress(p);
    drawBox(p, ctlRect(CTL_NAV_CRUISE), grey, white, QString("CRUISE\nCHECK >"), NAV_PX);   // the cruise check's own page
    lockBorder();
    return;
  }

  // The individual tests: diagnostic pages
  drawBox(p, backRect(), grey, white, QString("< START\nPAGE"), NAV_PX);
  drawBox(p, allOffRect(), QColor(201, 34, 49), white, ab_page ? QString("A/B OFF") : QString("ALL TESTS OFF"), 60);
  lockBorder();

  // Whether the system is engaged, always visible here: this page hides the road view and its engaged border.
  drawEngagedBadge(p, QRect(width() - 520, height() - ti_pages::BOTTOM_STRIP, 490, 64));

  drawPlanLine(p);
  const int n = tests.queueSize();
  const int n_defs = (int)tests.defs.size();
  QString banner;
  QColor banner_color(0x33, 0xcc, 0x66);
  const ti_pages::Run run = ab_plan_on ? Run::Plan : (ab_on ? Run::AbPlain : (tests.all_view.requested ? Run::All : (tests.singleTestOn() ? Run::Single : Run::Stopped)));
  p.setPen(QColor(0xa0, 0xa0, 0xa0));
  p.setFont(InterFont(30));
  p.drawText(QRect(ti_pages::BANNER_RECT.x, ti_pages::BANNER_RECT.y, ti_pages::BANNER_RECT.w, 70), Qt::AlignLeft | Qt::AlignVCenter | Qt::TextWordWrap,
             QString::fromStdString(individualNote(run)));
  if (ab_page) {
    drawBox(p, ctlRect(CTL_AB_BACK), grey, white, QString("< TEST PAGES"), NAV_PX);
    drawTitle(p, qr(TITLE_WIDE), QString("A/B SWITCH"), 56);
    banner = drawAb(p);
    banner_color = amber;
  } else if (queue_pos < 0 || queue_pos >= n) {
    drawTitle(p, qr(TITLE_WIDE), QString("TESTS"), 80);
    p.setPen(white);
    p.setFont(InterFont(56, QFont::DemiBold));
    p.drawText(QRect(30, rowTop(), width() - 60, rowHeight()), Qt::AlignCenter, QString("No valid test definitions"));
    drawBox(p, ctlRect(CTL_AB_PAGE), grey, white, QString("A/B\nSWITCH >"), NAV_PX);
  } else if (overview) {
    // The overview of run all: no PREV / NEXT; TEST PAGES goes back to the individual tests (run all off)
    drawBox(p, ctlRect(CTL_OVERVIEW_BACK), grey, white, QString("< TEST PAGES"), NAV_PX);
    drawTitle(p, qr(TITLE_WIDE), QString("RUN ALL TESTS"), 56);
    drawAll(p);
    banner = overview_banner;
  } else {
    // The queue: PREV / TEST k OF n / NEXT; an end of the queue greyed out, NEXT green once the shown test is done
    const QColor green(0x17, 0x86, 0x44);
    const bool finished = tests.done(queue_pos);
    const bool has_prev = queue_pos > 0;
    const bool has_next = queue_pos + 1 < n;
    drawBox(p, prevRect(), has_prev ? grey : dim_fill, has_prev ? white : dim_ink, QString("< PREV"), 56);
    drawBox(p, nextRect(), has_next ? (finished ? green : grey) : dim_fill, has_next ? white : dim_ink, QString("NEXT >"), 56);
    drawTitle(p, qr(TITLE_PREV_NEXT), QString("TEST %1\nOF %2").arg(queue_pos + 1).arg(n), 48);
    drawBox(p, ctlRect(CTL_OVERVIEW), tests.hasStepTests() ? QColor(0x46, 0x5B, 0xEA) : dim_fill, tests.hasStepTests() ? white : dim_ink,
            QString("RUN-ALL\nPAGE >"), NAV_PX);
    drawBox(p, ctlRect(CTL_AB_PAGE), ab_on ? QColor(0x17, 0x86, 0x44) : grey, white, ab_on ? QString("A/B ON >") : QString("A/B\nSWITCH >"), NAV_PX);

    if (finished) {
      bool all_done = true;
      for (int i = 0; i < n_defs; i++) {
        all_done = all_done && tests.done(i);
      }
      banner = has_next ? QString("Complete. Tap NEXT to switch to %1").arg(tests.defs[queue_pos + 1].title)
                        : (all_done ? QString("All queued tests are complete.") : QString("Complete. This is the last queued test."));
    }
    drawTest(p, queue_pos);
  }
  if (cruiseBusy(cruise_view)) {
    banner = QString("Cruise check running: the switches wait for it");
    banner_color = amber;
  }
  if (!banner.isEmpty()) {
    p.setPen(banner_color);
    p.setFont(InterFont(32, QFont::Bold));
    p.drawText(bannerRect(), Qt::AlignLeft | Qt::AlignVCenter | Qt::TextWordWrap, banner);
  }

  // Definitions that were ignored, and why (at most two lines)
  const int shown = std::min(2, (int)tests.errors.size());
  p.setPen(QColor(218, 202, 37));
  p.setFont(InterFont(34));
  for (int k = 0; k < shown; k++) {
    QString line = tests.errors[k];
    if (k == 1 && tests.errors.size() > 2) {
      line = QString("... and %1 more ignored definitions (see the log)").arg(tests.errors.size() - 1);
    }
    p.drawText(QRect(30, height() - ti_pages::BOTTOM_STRIP - 50 * (shown - k), width() - 580, 50), Qt::AlignLeft | Qt::AlignVCenter, line);
  }
}

void TestsWindow::drawBox(QPainter &p, const QRect &r, const QColor &fill, const QColor &ink, const QString &text, int px) const {
  p.setPen(Qt::NoPen);
  p.setBrush(fill);
  p.drawRoundedRect(r, 24, 24);
  p.setPen(ink);
  p.setFont(InterFont(px, QFont::Bold));
  p.drawText(r, Qt::AlignCenter, text);
}

// The on/off control: state in colour and words
void TestsWindow::drawToggle(QPainter &p, const TiTestView &view, bool off_at_ignition_off) const {
  const QRect toggle_r = toggleRect();
  const QColor ink = view.light ? QColor(0, 0, 0) : QColor(0xff, 0xff, 0xff);
  p.setPen(Qt::NoPen);
  p.setBrush(view.color);
  p.drawRoundedRect(toggle_r, 30, 30);
  p.setPen(ink);
  p.setFont(InterFont(64, QFont::Bold));
  p.drawText(toggle_r.adjusted(10, 10, -10, -toggle_r.height() / 3), Qt::AlignCenter, view.state);
  p.setFont(InterFont(36));
  p.drawText(toggle_r.adjusted(10, toggle_r.height() * 2 / 3 - 20, -10, off_at_ignition_off ? -70 : -10), Qt::AlignCenter,
             view.requested ? QString("tap to switch off") : QString("tap to switch on"));
  if (off_at_ignition_off) {
    p.setFont(InterFont(28));
    p.drawText(QRect(toggle_r.x() + 10, toggle_r.bottom() - 60, toggle_r.width() - 20, 50), Qt::AlignCenter,
               QString("turns off when the car is switched off"));
  }
}

// What the screen asks of the driver (two lines): the speed to hold for the nearest window still needing something of
// the tests in `scope` (indices into defs), what is left there, and where the car is now. Returns the next y.
int TestsWindow::drawGuidance(QPainter &p, int x, int y, int w, const std::vector<int> &scope) {
  const QColor white(0xff, 0xff, 0xff);
  const QColor green(0x33, 0xcc, 0x66);
  const QColor amber(218, 202, 37);
  const double conv = is_metric ? MILE_TO_KM : 1.0;
  const QString unit = is_metric ? QString("km/h") : QString("mph");

  QString zone_text = QString("Speed not known yet");
  ti_logic::Zone zone;
  if (have_speed) {
    zone = ti_logic::zoneFor(speed_mph);
    const QString now = QString("Now %1 %2: ").arg(qRound(speed_mph * conv)).arg(unit);
    switch (zone.state) {
      case ti_logic::ZoneState::Inside:
        zone_text = now + QString("inside a window, progress counts");
        break;
      case ti_logic::ZoneState::Between:
        zone_text = now + QString("between windows, tests run and are logged but count nothing");
        if (zone.window == 0 && speed_mph < ti_logic::WINDOWS[0].low) {
          zone_text = now + QString("%1-%2 %3 runs but counts nothing, the lowest window starts at %2").arg(qRound(ti_logic::MIN_TEST_MPH * conv))
                                .arg(qRound(ti_logic::WINDOWS[0].low * conv)).arg(unit);
        }
        break;
      case ti_logic::ZoneState::Transition:
        zone_text = now + QString("steering mode changes here (%1-%2 %3), nothing runs").arg(qRound(ti_logic::EPS_LOW_MPH * conv))
                              .arg(qRound(ti_logic::EPS_HIGH_MPH * conv)).arg(unit);
        break;
      case ti_logic::ZoneState::Below:
        zone_text = now + QString("below %1 %2, nothing runs").arg(qRound(ti_logic::MIN_TEST_MPH * conv)).arg(unit);
        break;
      case ti_logic::ZoneState::Above:
        zone_text = now + QString("above %1 %2, nothing runs").arg(qRound(ti_logic::MAX_TEST_MPH * conv)).arg(unit);
        break;
    }
  }

  const ti_logic::Target t = tests.target(scope, speed_mph);
  QString head;
  QColor color = white;
  if (t.window < 0) {
    head = QString("Every window is complete here. Extra data still helps.");
    color = green;
  } else {
    const ti_logic::Window &win = ti_logic::WINDOWS[t.window];
    const QString window_text = QString("%1-%2 %3").arg(qRound(win.low * conv)).arg(qRound(win.high * conv)).arg(unit);
    const QString left = tiTestClock(int(std::ceil(t.secs))) + (t.cells == 1 ? QString(" and 1 cell") : QString(" and %1 cells").arg(t.cells));
    const bool inside = have_speed && zone.state == ti_logic::ZoneState::Inside && zone.window == t.window;
    if (inside) {
      head = QString("IN THE WINDOW %1 · %2 to go").arg(window_text).arg(left);
      color = green;
    } else {
      head = QString("HOLD %1 %2 · window %3 · %4 to go").arg(qRound(ti_logic::windowTargetMph(t.window) * conv)).arg(unit.toUpper())
                 .arg(window_text).arg(left);
      color = amber;
    }
  }
  p.setPen(color);
  p.setFont(InterFont(42, QFont::Bold));
  p.drawText(QRect(x, y, w, 52), Qt::AlignLeft | Qt::AlignVCenter, head);
  p.setPen(QColor(0xc9, 0xc9, 0xc9));
  p.setFont(InterFont(32));
  p.drawText(QRect(x, y + 52, w, 40), Qt::AlignLeft | Qt::AlignVCenter, zone_text);
  return y + 96;
}

// While a limit (command, bend, lateral demand) blocks the test: which one, its value now and its limit, so a limit
// that is too tight shows up in numbers. `key`: the test's setting key, or empty for any test.
int TestsWindow::drawGate(QPainter &p, int x, int y, int w, const QString &key) {
  const TiTestGate &g = tests.gate;
  if (!g.valid || (!key.isEmpty() && g.test != key)) {
    return y;
  }
  int decimals = 0;
  QString unit = g.unit;
  if (g.unit == "1/m") {
    decimals = 4;
  } else if (g.unit == "m/s2") {
    decimals = 2;
    unit = QString("m/s²");
  }
  const QString value = QString::number(g.value, 'f', decimals);
  const QString limit = QString::number(g.limit, 'f', decimals);
  QString text;
  if (g.running) {
    text = QString("The step still to capture waits: %1 %2 against the limit %3 (%4 window). Other steps run meanwhile.")
               .arg(g.what).arg(value + " " + unit).arg(limit).arg(g.window);
  } else if (g.rearm) {
    text = QString("%1 is back under its limit (%2, now %3); starting again in a moment").arg(g.what).arg(limit).arg(value);
  } else {
    text = QString("Blocked: %1 %2 against the limit %3 (%4 window)").arg(g.what).arg(value + " " + unit).arg(limit).arg(g.window);
  }
  p.setPen(QColor(218, 202, 37));
  p.setFont(InterFont(32, QFont::DemiBold));
  p.drawText(QRect(x, y, w, 44), Qt::AlignLeft | Qt::AlignVCenter, text);
  return y + 46;
}

void TestsWindow::drawTest(QPainter &p, int i) {
  if (i >= (int)tests.views.size()) {
    return;
  }
  const TiTestDef &def = tests.defs[i];
  const TiTestView &view = tests.views[i];
  const QColor white(0xff, 0xff, 0xff);
  const QColor grey(0xa0, 0xa0, 0xa0);
  const int top = rowTop();
  const int row_h = rowHeight();

  const QRect toggle_r = toggleRect();
  drawToggle(p, view, def.off_at_ignition_off);
  drawBox(p, resetRect(), QColor(0x33, 0x33, 0x33), white, QString("Reset progress"), 40);

  // Title (DONE when every required bin has its time), the car side's arm and status lines when it publishes them
  const int x0 = toggle_r.right() + 40;
  const int x1 = width() - 30;
  const int head_w = resetRect().left() - 20 - x0;
  const bool finished = tests.done(i);
  QString heading = def.title;
  if (!view.arm.isEmpty()) {
    heading += "  ·  arm " + view.arm;
  }
  p.setPen(finished ? QColor(0x33, 0xcc, 0x66) : white);
  p.setFont(InterFont(50, QFont::Bold));
  p.drawText(QRect(x0, top + 10, head_w, 60), Qt::AlignLeft | Qt::AlignVCenter, finished ? "DONE · " + heading : heading);
  if (!view.status.isEmpty()) {
    p.setPen(QColor(0xc9, 0xc9, 0xc9));
    p.setFont(InterFont(38));
    p.drawText(QRect(x0, top + 70, head_w, 45), Qt::AlignLeft | Qt::AlignVCenter, view.status);
  }
  // The speed to hold (step tests only: the dither has its own bins), then any limit that is blocking this test
  int y = top + 122;
  if (def.metric == "step_seconds") {
    y = drawGuidance(p, x0, y, x1 - x0, std::vector<int>{i});
    y = drawGate(p, x0, y, x1 - x0, def.settings.empty() ? QString() : def.settings[0].first);
  }

  // One bar per bin (or one for a test without bins), then "other speeds"; the current bin outlined
  const int lines = def.bins.empty() ? 1 : (int)def.bins.size() + 1;
  const int area_top = y + 10;                                  // under the guidance and any limit line
  const int lh = std::max(30, std::min(90, (top + row_h - area_top) / lines));
  const int px = std::max(24, std::min(46, lh * 55 / 100));
  const QString current = have_speed ? TiTests::binFor(def, speed_mph) : QString();
  const int label_w = 360;
  const int value_w = 400;
  const int bar_x = x0 + label_w + 20;
  const int bar_w = std::max(100, x1 - value_w - 20 - bar_x);

  for (int k = 0; k < lines; k++) {
    const int line_y = area_top + k * lh;
    QString key = QString(TI_TEST_ALL);
    QString label = QString("all speeds");
    bool required = true;
    bool has_bar = true;
    double collected = view.seconds.value(key, 0.0);
    double needed = def.target;
    int steps = 0;
    if (!def.bins.empty() && k < (int)def.bins.size()) {
      key = def.bins[k].label;
      label = TiTests::binText(def.bins[k], is_metric);
      required = def.bins[k].required;
      collected = tests.have(i, def.bins[k]);
      needed = TiTests::need(def, def.bins[k]);
      steps = def.bins[k].steps;
    } else if (!def.bins.empty()) {
      key = QString(TI_TEST_OTHER);
      label = def.metric == "step_seconds" ? QString("between windows") : QString("other speeds");
      has_bar = false;
      collected = view.seconds.value(key, 0.0);
    }
    const bool here = key == current;

    if (here) {
      QPen pen = QPen(white);
      pen.setWidth(4);
      p.setPen(pen);
      p.setBrush(Qt::NoBrush);
      p.drawRoundedRect(QRect(x0 - 12, line_y + 2, x1 - x0 + 24, lh - 4), 12, 12);
    }

    p.setPen(required ? white : grey);
    p.setFont(InterFont(px, here ? QFont::Bold : QFont::Normal));
    p.drawText(QRect(x0, line_y, label_w, lh), Qt::AlignLeft | Qt::AlignVCenter, label);

    QString value = QString("%1 s").arg(int(collected));
    if (has_bar) {
      const QRect bar = QRect(bar_x, line_y + lh / 4, bar_w, lh / 2);
      const double fraction = std::min(1.0, std::max(0.0, collected / needed));
      p.setPen(Qt::NoPen);
      p.setBrush(QColor(0x33, 0x33, 0x33));
      p.drawRoundedRect(bar, 8, 8);
      if (fraction > 0) {
        p.setBrush(fraction >= 1.0 ? QColor(0x17, 0x86, 0x44) : QColor(0x46, 0x5B, 0xEA));
        p.drawRoundedRect(QRect(bar.x(), bar.y(), std::max(16, int(bar.width() * fraction)), bar.height()), 8, 8);
      }
      if (!required) {
        p.setPen(white);
        p.setFont(InterFont(std::max(22, px - 14)));
        p.drawText(bar, Qt::AlignCenter, QString("optional"));
      }
      value = QString("%1 / %2 s").arg(int(collected)).arg(int(def.target));
      if (steps > 0) {  // step_seconds: the time still to collect in this window, or the time beyond the targets
        const int left = int(std::ceil(needed - collected));
        const int extra = int(view.step_extra.value(key, 0.0));
        value = collected < needed ? tiTestClock(left) + " left" : (extra > 0 ? "done +" + tiTestClock(extra) : QString("done"));
      }
    }
    p.setPen(required ? white : grey);
    p.setFont(InterFont(px, QFont::DemiBold));
    p.drawText(QRect(x1 - value_w, line_y, value_w, lh), Qt::AlignRight | Qt::AlignVCenter, value);
  }
}

// The run-all overview: the run-all switch, the speed to hold, then every test against every window with what is left
void TestsWindow::drawAll(QPainter &p) {
  const TiTestView &view = tests.all_view;
  const QColor white(0xff, 0xff, 0xff);
  const QColor green(0x33, 0xcc, 0x66);
  const int top = rowTop();
  const int row_h = rowHeight();

  drawToggle(p, view, true);

  const int x0 = toggleRect().right() + 40;
  const int x1 = width() - 30;
  p.setPen(white);
  p.setFont(InterFont(50, QFont::Bold));
  p.drawText(QRect(x0, top + 10, x1 - x0, 60), Qt::AlignLeft | Qt::AlignVCenter, QString("Run all tests by speed window"));
  if (!view.status.isEmpty()) {
    p.setPen(QColor(0xc9, 0xc9, 0xc9));
    p.setFont(InterFont(38));
    p.drawText(QRect(x0, top + 70, x1 - x0, 45), Qt::AlignLeft | Qt::AlignVCenter, view.status);
  }

  // The step tests in the order the car side runs them (windows are shared by all of them)
  const QStringList ids = {"ti_sweep", "ti_pulses", "ti_ladder"};
  const QStringList names = {"Sweep", "Pulses", "Ladder"};
  std::vector<int> shown;
  QStringList shown_names;
  for (int j = 0; j < (int)ids.size(); j++) {
    for (int i = 0; i < (int)tests.defs.size(); i++) {
      if (tests.defs[i].id == ids[j] && tests.defs[i].metric == "step_seconds") {
        shown.push_back(i);
        shown_names.append(names[j]);
      }
    }
  }
  const TiTestDef *bands = nullptr;
  for (const TiTestDef &def : tests.defs) {
    if (bands == nullptr && def.metric == "step_seconds") {
      bands = &def;
    }
  }
  if (bands == nullptr || bands->bins.empty()) {
    return;
  }

  int y = drawGuidance(p, x0, top + 122, x1 - x0, shown);
  y = drawGate(p, x0, y, x1 - x0, QString());

  // One row per window, the one the car is inside outlined, the target marked; the test running now in bold with a bullet
  const int lines = (int)bands->bins.size();
  const int area_top = y + 10;                                  // under the guidance and any limit line
  const int lh = std::max(30, std::min(90, (top + row_h - area_top) / lines));
  const int px = std::max(24, std::min(46, lh * 55 / 100));
  const int item_px = std::min(36, px);
  const QString current = (have_speed && ti_logic::zoneFor(speed_mph).state == ti_logic::ZoneState::Inside) ? TiTests::binFor(*bands, speed_mph) : QString();
  const ti_logic::Target t = tests.target(shown, speed_mph);
  const QString target_label = t.window >= 0 ? QString(ti_logic::WINDOWS[t.window].label) : QString();
  const int label_w = 360;
  const int item_w = (x1 - x0 - label_w - 20) / std::max(1, (int)shown.size());
  bool every_band = !shown.empty();
  bool this_band = false;
  for (int k = 0; k < lines; k++) {
    const TiTestBin &band = bands->bins[k];
    const int line_y = area_top + k * lh;
    const bool here = band.label == current;

    if (here) {
      QPen pen = QPen(white);
      pen.setWidth(4);
      p.setPen(pen);
      p.setBrush(Qt::NoBrush);
      p.drawRoundedRect(QRect(x0 - 12, line_y + 2, x1 - x0 + 24, lh - 4), 12, 12);
    }
    p.setPen(band.label == target_label ? QColor(218, 202, 37) : white);
    p.setFont(InterFont(px, here ? QFont::Bold : QFont::Normal));
    p.drawText(QRect(x0, line_y, label_w, lh), Qt::AlignLeft | Qt::AlignVCenter,
               (band.label == target_label ? QString("▶ ") : QString()) + TiTests::binText(band, is_metric));

    bool band_done = !shown.empty();
    for (int j = 0; j < (int)shown.size(); j++) {
      const TiTestDef &def = tests.defs[shown[j]];
      const TiTestBin *bin = nullptr;
      for (const TiTestBin &b : def.bins) {
        if (b.label == band.label) {
          bin = &b;
        }
      }
      if (bin == nullptr) {
        continue;
      }
      const double collected = tests.have(shown[j], *bin);
      const double needed = TiTests::need(def, *bin);
      const bool complete = collected >= needed;
      const bool running = here && view.active && tests.all_test == def.id;
      band_done = band_done && complete;
      QString item = shown_names[j] + (complete ? QString(" done") : " " + tiTestClock(int(std::ceil(needed - collected))) + " left");
      if (running) {
        item = "• " + item;
      }
      p.setPen(complete ? green : white);
      p.setFont(InterFont(item_px, running ? QFont::Bold : QFont::Normal));
      p.drawText(QRect(x0 + label_w + 20 + j * item_w, line_y, item_w, lh), Qt::AlignLeft | Qt::AlignVCenter, item);
    }
    every_band = every_band && band_done;
    this_band = this_band || (here && band_done);
  }

  overview_banner = QString();
  if (every_band) {
    overview_banner = QString("All windows complete. Run all goes on, choosing tests at random.");
  } else if (this_band) {
    overview_banner = QString("This window is complete: run all picks tests at random here. Other speeds still need data");
  }
}

void Sidebar::drawMetric(QPainter &p, const QPair<QString, QString> &label, QColor c, int y) {
  const QRect rect = {30, y, 240, 126};

  p.setPen(Qt::NoPen);
  p.setBrush(QBrush(c));
  p.setClipRect(rect.x() + 4, rect.y(), 18, rect.height(), Qt::ClipOperation::ReplaceClip);
  p.drawRoundedRect(QRect(rect.x() + 4, rect.y() + 4, 100, 118), 18, 18);
  p.setClipping(false);

  QPen pen = QPen(QColor(0xff, 0xff, 0xff, 0x55));
  pen.setWidth(2);
  p.setPen(pen);
  p.setBrush(Qt::NoBrush);
  p.drawRoundedRect(rect, 20, 20);

  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setFont(InterFont(35, QFont::DemiBold));
  p.drawText(rect.adjusted(22, 0, 0, 0), Qt::AlignCenter, label.first + "\n" + label.second);
}

// drawMetric, but in the taller test_rect and a size smaller so "NOT APPLIED" fits on one line
void Sidebar::drawTestCard(QPainter &p) {
  p.setPen(Qt::NoPen);
  p.setBrush(QBrush(test_status.second));
  p.setClipRect(test_rect.x() + 4, test_rect.y(), 18, test_rect.height(), Qt::ClipOperation::ReplaceClip);
  p.drawRoundedRect(QRect(test_rect.x() + 4, test_rect.y() + 4, 100, test_rect.height() - 8), 18, 18);
  p.setClipping(false);

  QPen pen = QPen(QColor(0xff, 0xff, 0xff, 0x55));
  pen.setWidth(2);
  p.setPen(pen);
  p.setBrush(Qt::NoBrush);
  p.drawRoundedRect(test_rect, 20, 20);

  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setFont(InterFont(30, QFont::DemiBold));
  p.drawText(test_rect.adjusted(22, 0, 0, 0), Qt::AlignCenter, test_status.first.first + "\n" + test_status.first.second);
}

Sidebar::Sidebar(QWidget *parent) : QFrame(parent), onroad(false), flag_pressed(false), settings_pressed(false) {
  connect(this, &Sidebar::valueChanged, [=] { update(); });

  setAttribute(Qt::WA_OpaquePaintEvent);
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
  setFixedWidth(300);

  QObject::connect(uiState(), &UIState::uiUpdate, this, &Sidebar::updateState);

  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"userFlag"});

  // FrogPilot variables
  QObject::connect(frogpilotUIState(), &FrogPilotUIState::themeUpdated, this, &Sidebar::updateTheme);
}

void Sidebar::updateTheme() {
  FrogPilotUIState &fs = *frogpilotUIState();
  FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;
  QJsonObject &frogpilot_toggles = fs.frogpilot_toggles;

  isCPU = frogpilot_toggles.value("cpu_metrics").toBool();
  isDeveloperUI = frogpilot_toggles.value("developer_ui").toBool();
  isFahrenheit = frogpilot_toggles.value("fahrenheit").toBool();
  isGPU = frogpilot_toggles.value("gpu_metrics").toBool();
  isIP = frogpilot_toggles.value("ip_metrics").toBool();
  isMemoryUsage = frogpilot_toggles.value("memory_metrics").toBool();
  isNumericalTemp = frogpilot_toggles.value("numerical_temp").toBool();
  isStorageLeft = frogpilot_toggles.value("storage_left_metrics").toBool();
  isStorageUsed = frogpilot_toggles.value("storage_used_metrics").toBool();
  sidebar_color1 = frogpilot_scene.use_stock_colors ? good_color : frogpilot_scene.sidebar_color1;
  sidebar_color2 = frogpilot_scene.use_stock_colors ? good_color : frogpilot_scene.sidebar_color2;
  sidebar_color3 = frogpilot_scene.use_stock_colors ? good_color : frogpilot_scene.sidebar_color3;

  if (util::random_int(0, 100) == 69 && frogpilot_toggles.value("random_events").toBool()) {
    loadImage("../../frogpilot/assets/random_events/icons/button_home", home_img, home_gif, home_btn.size(), this);
  } else {
    loadImage("../../frogpilot/assets/active_theme/icons/button_home", home_img, home_gif, home_btn.size(), this);
  }
  loadImage("../../frogpilot/assets/active_theme/icons/button_flag", flag_img, flag_gif, home_btn.size(), this);
  loadImage("../../frogpilot/assets/active_theme/icons/button_settings", settings_img, settings_gif, settings_btn.size(), this);
}

void Sidebar::showEvent(QShowEvent *event) {
  updateTheme();

  // The tests card, when it is up, reads the definitions again each time the sidebar opens (a small file) and
  // shows the state at once
  testTick = UI_FREQ - 1;
  if (showTest) {
    tiTests.load();
    tiTests.poll(frogpilotUIState()->frogpilot_toggles);
    updateTestStatus();
  }
}

void Sidebar::mousePressEvent(QMouseEvent *event) {
  QPoint pos = event->pos();

  QRect cpuRect = {30, 496, 240, 126};
  QRect memoryRect = {30, 654, 240, 126};
  QRect tempRect = {30, 338, 240, 126};

  static int showChip = 0;
  static int showMemory = 0;
  static int showTemp = 0;

  if (onroad && showTest && test_rect.contains(pos)) {
    // Opens the Tests page on release, the way the settings button opens Settings
    tests_pressed = true;
  } else if (cpuRect.contains(pos) && isDeveloperUI) {
    showChip = (showChip + 1) % 3;

    isCPU = showChip == 1;
    isGPU = showChip == 2;

    params.putBool("ShowCPU", isCPU);
    params.putBool("ShowGPU", isGPU);
  } else if (memoryRect.contains(pos) && isDeveloperUI) {
    showMemory = (showMemory + 1) % 4;

    isMemoryUsage = showMemory == 1;
    isStorageLeft = showMemory == 2;
    isStorageUsed = showMemory == 3;

    params.putBool("ShowMemoryUsage", isMemoryUsage);
    params.putBool("ShowStorageLeft", isStorageLeft);
    params.putBool("ShowStorageUsed", isStorageUsed);
  } else if (tempRect.contains(pos) && isDeveloperUI) {
    showTemp = (showTemp + 1) % 3;

    isFahrenheit = showTemp == 2;
    isNumericalTemp = showTemp != 0;

    params.putBool("Fahrenheit", showTemp == 2);
    params.putBool("NumericalTemp", showTemp != 0);
  } else if (onroad && home_btn.contains(pos)) {
    flag_pressed = true;
  } else if (settings_btn.contains(pos)) {
    settings_pressed = true;
  }

  update();
  updateFrogPilotToggles();
}

void Sidebar::mouseReleaseEvent(QMouseEvent *event) {
  const bool open_tests = tests_pressed && onroad && showTest && test_rect.contains(event->pos());
  tests_pressed = false;

  if (flag_pressed || settings_pressed) {
    flag_pressed = settings_pressed = false;
    update();
  }
  if (onroad && home_btn.contains(event->pos())) {
    MessageBuilder msg;
    msg.initEvent().initUserFlag();
    pm->send("userFlag", msg);
  } else if (settings_btn.contains(event->pos())) {
    emit openSettings();
  } else if (open_tests && !tiTestsAlertShowing(*uiState(), *frogpilotUIState())) {
    emit openTests();
  }
}

void Sidebar::offroadTransition(bool offroad) {
  const bool drive_ended = onroad && offroad;
  onroad = !offroad;

  // Mazda TI tests: the end of a drive (not the offroad state at boot)
  if (drive_ended && tiTestsAllowed(*uiState(), *frogpilotUIState())) {
    tiTests.load();
    tiTests.driveEnded();
  }

  // FrogPilot variables
  if (onroad) {
    updateTheme();
  }

  update();
}

void Sidebar::updateState(const UIState &s, const FrogPilotUIState &fs) {
  if (!isVisible()) return;

  auto &sm = *(s.sm);

  auto deviceState = sm["deviceState"].getDeviceState();
  setProperty("netType", network_type[deviceState.getNetworkType()]);
  int strength = (int)deviceState.getNetworkStrength();
  setProperty("netStrength", strength > 0 ? strength + 1 : 0);

  ItemStatus connectStatus;
  auto last_ping = deviceState.getLastAthenaPingTime();
  if (last_ping == 0) {
    connectStatus = ItemStatus{{tr("CONNECT"), tr("OFFLINE")}, warning_color};
  } else {
    connectStatus = nanos_since_boot() - last_ping < 80e9
                        ? ItemStatus{{tr("CONNECT"), tr("ONLINE")}, sidebar_color3}
                        : ItemStatus{{tr("CONNECT"), tr("ERROR")}, danger_color};
  }
  setProperty("connectStatus", QVariant::fromValue(connectStatus));

  int maxTempC = deviceState.getMaxTempC();
  QString max_temp = isFahrenheit ? QString::number(maxTempC * 9 / 5 + 32) + "°F" : QString::number(maxTempC) + "°C";
  ItemStatus tempStatus = {{tr("TEMP"), isNumericalTemp ? max_temp : tr("HIGH")}, danger_color};
  auto ts = deviceState.getThermalStatus();
  if (ts == cereal::DeviceState::ThermalStatus::GREEN) {
    tempStatus = {{tr("TEMP"), isNumericalTemp ? max_temp : tr("GOOD")}, sidebar_color1};
  } else if (ts == cereal::DeviceState::ThermalStatus::YELLOW) {
    tempStatus = {{tr("TEMP"), isNumericalTemp ? max_temp : tr("OK")}, warning_color};
  }
  setProperty("tempStatus", QVariant::fromValue(tempStatus));

  ItemStatus pandaStatus = {{tr("VEHICLE"), tr("ONLINE")}, sidebar_color2};
  if (s.scene.pandaType == cereal::PandaState::PandaType::UNKNOWN) {
    pandaStatus = {{tr("NO"), tr("PANDA")}, danger_color};
  } else if (s.scene.started && !sm["liveLocationKalman"].getLiveLocationKalman().getGpsOK()) {
    pandaStatus = {{tr("GPS"), tr("SEARCH")}, warning_color};
  }
  setProperty("pandaStatus", QVariant::fromValue(pandaStatus));

  // FrogPilot variables
  const SubMaster &frog_sm = *(fs.sm);
  const cereal::FrogPilotDeviceState::Reader &frogpilotDeviceState = frog_sm["frogpilotDeviceState"].getFrogpilotDeviceState();

  if (isCPU || isGPU) {
    capnp::List<int8_t>::Reader cpu_loads = deviceState.getCpuUsagePercent();
    int cpu_usage = cpu_loads.size() != 0 ? std::accumulate(cpu_loads.begin(), cpu_loads.end(), 0) / cpu_loads.size() : 0;
    int gpu_usage = deviceState.getGpuUsagePercent();
    int usage = isCPU ? cpu_usage : gpu_usage;

    QString chip_usage = QString::number(usage) + "%";

    ItemStatus chipStatus = {{isGPU ? tr("GPU") : tr("CPU"), chip_usage}, sidebar_color2};
    if (usage >= 85) {
      chipStatus = {{isGPU ? tr("GPU") : tr("CPU"), chip_usage}, danger_color};
    } else if (usage >= 70) {
      chipStatus = {{isGPU ? tr("GPU") : tr("CPU"), chip_usage}, warning_color};
    }
    setProperty("chipStatus", QVariant::fromValue(chipStatus));
  }

  if (isMemoryUsage || isStorageLeft || isStorageUsed) {
    int free_space = deviceState.getFreeSpacePercent();
    int memory_usage = deviceState.getMemoryUsagePercent();
    int storage_left = frogpilotDeviceState.getFreeSpace();
    int storage_used = frogpilotDeviceState.getUsedSpace();

    QString memory = QString::number(memory_usage) + "%";
    QString storage = QString::number(isStorageLeft ? storage_left : storage_used) + tr(" GB");

    if (isMemoryUsage) {
      ItemStatus memoryStatus = {{tr("MEMORY"), memory}, sidebar_color3};
      if (memory_usage >= 85) {
        memoryStatus = {{tr("MEMORY"), memory}, danger_color};
      } else if (memory_usage >= 70) {
        memoryStatus = {{tr("MEMORY"), memory}, warning_color};
      }
      setProperty("memoryStatus", QVariant::fromValue(memoryStatus));
    } else {
      ItemStatus storageStatus = {{isStorageLeft ? tr("LEFT") : tr("USED"), storage}, sidebar_color3};
      if (free_space < 25 && free_space >= 10) {
        storageStatus = {{isStorageLeft ? tr("LEFT") : tr("USED"), storage}, warning_color};
      } else if (10 > free_space) {
        storageStatus = {{isStorageLeft ? tr("LEFT") : tr("USED"), storage}, danger_color};
      }
      setProperty("storageStatus", QVariant::fromValue(storageStatus));
    }
  }

  // Mazda TI tests card, refreshed once a second
  if (++testTick >= UI_FREQ) {
    testTick = 0;

    // Driving screen only: nothing is read or shown offroad or on any other car
    const bool testVisible = onroad && tiTestsAllowed(s, fs);
    if (testVisible != showTest) {
      showTest = testVisible;
      if (showTest) {
        tiTests.load();
      }
      update();
    }
    if (showTest) {
      tiTests.poll(fs.frogpilot_toggles);
      updateTestStatus();
    }
  }
}

// TESTS / the most urgent state of any test / how many are on and the share of their required time collected
void Sidebar::updateTestStatus() {
  bool not_applied = false;
  bool active = false;
  bool waiting = false;
  for (const TiTestView &view : tiTests.views) {
    not_applied = not_applied || view.state == "NOT APPLIED";
    active = active || view.state == "ON · ACTIVE";
    waiting = waiting || view.any_on;
  }
  // ... and the run-all mode, which injects with every single test off
  not_applied = not_applied || tiTests.all_view.state == "NOT APPLIED";
  active = active || tiTests.all_view.state == "ON · ACTIVE";
  waiting = waiting || tiTests.all_view.any_on;

  ItemStatus testStatus = {{tr("TESTS"), tr("ALL OFF")}, QColor(0x54, 0x54, 0x54)};
  if (tiTests.defs.empty()) {
    testStatus = {{tr("TESTS"), tr("NONE VALID")}, QColor(0x54, 0x54, 0x54)};
  } else if (not_applied) {
    testStatus = {{tr("TESTS"), tr("NOT APPLIED")}, danger_color};
  } else if (active) {
    testStatus = {{tr("TESTS"), tr("ON · ACTIVE")}, QColor(0x17, 0x86, 0x44)};
  } else if (waiting) {
    testStatus = {{tr("TESTS"), tr("ON · WAIT")}, warning_color};
  }

  const int on = tiTests.countOn();
  const int percent = tiTests.overallPercent();
  QString summary = tr("tap to open");
  if (on > 0) {
    summary = percent >= 0 ? QString("%1 ON · %2%").arg(on).arg(percent) : QString("%1 ON").arg(on);
  }
  testStatus.first.second += "\n" + summary;
  setProperty("testStatus", QVariant::fromValue(testStatus));
}

void Sidebar::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setPen(Qt::NoPen);
  p.setRenderHint(QPainter::Antialiasing);

  p.fillRect(rect(), QColor(57, 57, 57));

  // buttons
  p.setOpacity(settings_pressed ? 0.65 : 1.0);
  if (settings_gif) {
    p.drawPixmap(settings_btn.x(), settings_btn.y(), settings_gif->currentPixmap());
  } else {
    p.drawPixmap(settings_btn.x(), settings_btn.y(), settings_img);
  }
  p.setOpacity(onroad && flag_pressed ? 0.65 : 1.0);
  if (onroad) {
    if (flag_gif) {
      p.drawPixmap(home_btn.x(), home_btn.y(), flag_gif->currentPixmap());
    } else {
      p.drawPixmap(home_btn.x(), home_btn.y(), flag_img);
    }
  } else {
    if (home_gif) {
      p.drawPixmap(home_btn.x(), home_btn.y(), home_gif->currentPixmap());
    } else {
      p.drawPixmap(home_btn.x(), home_btn.y(), home_img);
    }
  }
  p.setOpacity(1.0);

  // network
  int x = 58;
  const QColor gray(0x54, 0x54, 0x54);
  p.setFont(InterFont(35));

  if (isIP) {
    p.setPen(QColor(0xff, 0xff, 0xff));
    p.save();
    p.setFont(InterFont(30));
    p.drawText(QRect(50, 196, 225, 27), Qt::AlignLeft | Qt::AlignVCenter, frogpilotUIState()->wifi->getIp4Address());
    p.restore();
  } else {
    for (int i = 0; i < 5; ++i) {
      p.setBrush(i < net_strength ? Qt::white : gray);
      p.drawEllipse(x, 196, 27, 27);
      x += 37;
    }
    p.setPen(QColor(0xff, 0xff, 0xff));
  }

  const QRect r = QRect(50, 247, 100, 50);
  p.drawText(r, Qt::AlignCenter, net_type);

  // metrics
  drawMetric(p, temp_status.first, temp_status.second, 338);
  if (isCPU || isGPU) {
    drawMetric(p, chip_status.first, chip_status.second, 496);
  } else {
    drawMetric(p, panda_status.first, panda_status.second, 496);
  }
  if (onroad && showTest) {
    drawTestCard(p);
  } else if (isMemoryUsage) {
    drawMetric(p, memory_status.first, memory_status.second, 654);
  } else if (isStorageLeft || isStorageUsed) {
    drawMetric(p, storage_status.first, storage_status.second, 654);
  } else {
    drawMetric(p, connect_status.first, connect_status.second, 654);
  }
}
