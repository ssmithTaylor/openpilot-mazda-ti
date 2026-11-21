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
  tests_params_memory.putBool("TiAbOn", false);     // A/B and the tests exclude each other
  tests_params_memory.remove("TiAbPin");
  setAll(true);
  all_view.mismatch_ticks = 0;
  for (size_t i = 0; i < defs.size() && i < views.size(); i++) {
    if (defs[i].metric == "step_seconds" && views[i].any_on) {
      setTest((int)i, false);
      views[i].mismatch_ticks = 0;
    }
  }
  updateFrogPilotToggles();
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

// Every allow-listed key to "off", whatever the definitions say (works with a broken file too)
void TiTests::allOff() {
  for (const TiTestAllowed &a : TI_TEST_ALLOWED) {
    tiTestSwitchOff(tests_params, QString(a.key), a.on);
  }
  for (TiTestView &view : views) {
    view.mismatch_ticks = 0;
  }
  all_view.mismatch_ticks = 0;
  updateFrogPilotToggles();
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
  overview = tests.overviewQueued();
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
  if (ab_page) {
    abPoll();
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

// One test at a time: the queued one, below the header and its banner line
int TestsWindow::rowTop() const {
  return 260;
}

int TestsWindow::rowHeight() const {
  const int footer = tests.errors.isEmpty() ? 0 : 30 + 50 * std::min(2, (int)tests.errors.size());
  return height() - 20 - footer - rowTop();
}

QRect TestsWindow::backRect() const {
  return QRect(30, 24, 440, 150);
}

QRect TestsWindow::allOffRect() const {
  return QRect(width() - 670, 24, 640, 150);
}

// PREV and NEXT share the header between BACK and ALL TESTS OFF with the "TEST k OF n" line
QRect TestsWindow::prevRect() const {
  const int left = backRect().right() + 20;
  const int w = std::min(300, (allOffRect().left() - 20 - left) / 3);
  return QRect(left, 24, w, 150);
}

QRect TestsWindow::nextRect() const {
  const int right = allOffRect().left() - 20;
  const int w = std::min(300, (right - backRect().right() - 20) / 3);
  return QRect(right - w, 24, w, 150);
}

// The row under the header: RUN ALL (on the overview: TEST PAGES) on the left, the LOCK on the right, the banner between
QRect TestsWindow::runAllRect() const {
  return QRect(30, 184, 420, 66);
}

QRect TestsWindow::lockRect() const {
  return QRect(width() - 450, 184, 420, 66);
}

QRect TestsWindow::bannerRect() const {
  const int right = ab_page ? lockRect().left() : abRect().left();
  return QRect(runAllRect().right() + 20, 184, right - runAllRect().right() - 40, 66);
}

QRect TestsWindow::toggleRect() const {
  return QRect(30, rowTop() + 10, 600, rowHeight() - 30);
}

QRect TestsWindow::resetRect() const {
  return QRect(width() - 430, rowTop() + 10, 400, 110);
}

// Controls: 1 BACK (ignored while locked), 2 ALL TESTS OFF, 3 PREV, 4 NEXT (none at the ends of the queue), 5 LOCK,
// 6 RUN ALL (opens the overview), 7 TEST PAGES (leaves it), 8 A/B (opens the A/B page), 9 TEST PAGES (leaves the A/B page),
// 10 + i the on/off box of the shown test i (the overview: i = defs.size(), run-all mode), 20 + i its Reset progress,
// 30-38 the A/B page's boxes (abBox); 0 none
int TestsWindow::controlAt(const QPoint &pos) const {
  if (backRect().contains(pos)) {
    return 1;
  }
  if (allOffRect().contains(pos)) {
    return 2;
  }
  if (lockRect().contains(pos)) {
    return 5;
  }
  if (ab_page) {
    return abControlAt(pos);
  }
  if (abRect().contains(pos)) {
    return 8;
  }
  if (queue_pos < 0 || queue_pos >= tests.queueSize()) {
    return 0;
  }
  if (overview) {
    if (runAllRect().contains(pos)) {
      return 7;
    }
    if (toggleRect().contains(pos)) {
      return 10 + (int)tests.defs.size();
    }
    return 0;
  }
  if (prevRect().contains(pos)) {
    return queue_pos > 0 ? 3 : 0;
  }
  if (nextRect().contains(pos)) {
    return queue_pos + 1 < tests.queueSize() ? 4 : 0;
  }
  if (runAllRect().contains(pos)) {
    return tests.hasStepTests() ? 6 : 0;
  }
  if (toggleRect().contains(pos)) {
    return 10 + queue_pos;
  }
  if (resetRect().contains(pos)) {
    return queue_pos < (int)tests.defs.size() ? 20 + queue_pos : 0;
  }
  return 0;
}

// A press only records which control it is on, and none in the first moment after the page opens (the on/off box
// sits where the sidebar card was, so a second tap or a bounce of the opening tap must not switch the test)
void TestsWindow::mousePressEvent(QMouseEvent *event) {
  idle.start();
  pressed_control = opened.elapsed() < TI_TEST_INPUT_DELAY_MS ? 0 : controlAt(event->pos());
}

// A control acts on release, and only when press and release are on the same control
void TestsWindow::mouseReleaseEvent(QMouseEvent *event) {
  idle.start();
  const int control = controlAt(event->pos());
  const int pressed = pressed_control;
  pressed_control = 0;
  if (control == 0 || control != pressed) {
    return;
  }

  if (control == 1) {
    if (ti_logic::backAllowed(locked)) {  // locked: unlock first (the LOCK button), so a stray tap cannot dismiss the page
      ab_page = false;                    // the next opening from the sidebar card starts on the test pages
      leave();
    }
    return;
  }
  if (control == 8 || control == 9 || control >= 30 || (control == 2 && ab_page)) {
    abAct(control);       // the A/B page: its red box turns A/B off, not the tests
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

namespace {
const char *const TI_AB_CONFIG_KEY = "TiAbConfig";
const char *const TI_AB_ON_KEY = "TiAbOn";
const char *const TI_AB_PIN_KEY = "TiAbPin";
const char *const TI_AB_STATUS_KEY = "TiAbStatus";
}  // namespace

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
void TestsWindow::abOpen() {
  ab_setting = 0;
  ab_block_min = ti_ab::DEFAULT_BLOCK_MIN;
  bool have_b = false;
  const QJsonObject cfg = QJsonDocument::fromJson(QByteArray::fromStdString(ab_params.get(TI_AB_CONFIG_KEY))).object();
  const int i = ti_ab::settingIndex(cfg.value("setting").toString().toStdString().c_str());
  if (i >= 0) {
    ab_setting = i;
    const QJsonValue b = cfg.value("b");
    ab_b = b.isBool() ? (b.toBool() ? 1.0 : 0.0) : b.toDouble(0.0);
    have_b = b.isBool() || b.isDouble();
  }
  const double minutes = cfg.value("block_min").toDouble(ti_ab::DEFAULT_BLOCK_MIN);
  ab_block_min = std::min(ti_ab::MAX_BLOCK_MIN, std::max(ti_ab::MIN_BLOCK_MIN, std::round(minutes)));
  abFixB(have_b);
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
  const std::string pin = ab_memory.get(TI_AB_PIN_KEY);
  ab_pin = pin == "A" ? ti_ab::Pin::A : (pin == "B" ? ti_ab::Pin::B : ti_ab::Pin::None);
  ab_status = QJsonDocument::fromJson(QByteArray::fromStdString(ab_memory.get(TI_AB_STATUS_KEY))).object();
  ab_fresh = ti_ab::statusFresh(QDateTime::currentMSecsSinceEpoch() / 1000.0, ab_status.value("t").toDouble(-1e9));
  if (!ab_on) {
    ab_a_known = abStoredValue(ab_setting, ab_a);   // follows what he stores while the page is open and A/B is off
  }
}

QRect TestsWindow::abRect() const {
  return QRect(lockRect().left() - 250, 184, 230, 66);
}

// Boxes of the A/B page by control id: 30 on/off, 31 pin A, 32 pin B, 33 / 34 previous / next setting, 35 / 36 B - / B +,
// 37 / 38 block - / +
QRect TestsWindow::abBox(int id) const {
  const int top = rowTop() + 10;
  const int row2 = top + 210;
  switch (id) {
    case 30: return QRect(30, top, 560, 190);
    case 31: return QRect(620, top, 330, 190);
    case 32: return QRect(980, top, 330, 190);
    case 33: return QRect(30, row2, 100, 120);
    case 34: return QRect(30 + 100 + 560 + 16, row2, 100, 120);
    case 35: return QRect(30 + 100 + 560 + 16 + 100 + 30 + 250 + 20, row2, 100, 120);
    case 36: return QRect(abBox(35).right() + 1 + 16 + 200, row2, 100, 120);
    case 37: return QRect(abBox(36).right() + 1 + 40, row2, 100, 120);
    case 38: return QRect(abBox(37).right() + 1 + 16 + 200, row2, 100, 120);
  }
  return QRect();
}

int TestsWindow::abControlAt(const QPoint &pos) const {
  if (runAllRect().contains(pos)) {
    return 9;
  }
  for (int id = 30; id <= 38; id++) {
    if (abBox(id).contains(pos)) {
      return id;
    }
  }
  return 0;
}

void TestsWindow::abSetOn(bool on) {
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

void TestsWindow::abAct(int control) {
  const ti_ab::Setting &s = ti_ab::SETTINGS[ab_setting];
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
  } else if (ti_ab::canEdit(ab_on) && control >= 33 && control <= 38) {
    if (control == 33 || control == 34) {
      ab_setting = ti_ab::cycleSetting(ab_setting, control == 34 ? 1 : -1);
      abFixB(false);
    } else if (control == 35 || control == 36) {
      if (ab_a_known) {
        ab_b = ti_ab::stepB(s, ab_a, ab_b, control == 36 ? 1 : -1);
      }
    } else {
      ab_block_min = ti_ab::stepBlock(ab_block_min, control == 38 ? 1 : -1);
    }
    abSave();
  }
  abPoll();
}

static QString abValueText(const ti_ab::Setting &s, double v) {
  return s.is_bool ? (v != 0.0 ? QString("ON") : QString("OFF")) : QString::number(v, 'f', 2);
}

// Draws the A/B page below the shared header; returns the banner line
QString TestsWindow::drawAb(QPainter &p) {
  const QColor white(0xff, 0xff, 0xff);
  const QColor dim_fill(0x1a, 0x1a, 0x1a);
  const QColor dim_ink(0x55, 0x55, 0x55);
  const QColor amber(218, 202, 37);
  const QColor blue(0x46, 0x5B, 0xEA);
  const QColor grey(0x33, 0x33, 0x33);
  const ti_ab::Setting &s = ti_ab::SETTINGS[ab_setting];
  const QString state = ab_fresh ? ab_status.value("state").toString() : QString("off");
  const bool running = ab_on && ab_fresh && (state == "running" || state == "pinned");
  const QString arm = ab_status.value("arm").toString();

  // the switch and the pins
  drawBox(p, abBox(30), ab_on ? QColor(0x17, 0x86, 0x44) : QColor(201, 34, 49), white,
          ab_on ? QString("A/B ON\ntap to stop") : QString("A/B OFF\ntap to start"), 52);
  drawBox(p, abBox(31), ab_pin == ti_ab::Pin::A ? blue : (ab_on ? grey : dim_fill), ab_on ? white : dim_ink,
          ab_pin == ti_ab::Pin::A ? QString("A PINNED\ntap to release") : QString("PIN A"), 44);
  drawBox(p, abBox(32), ab_pin == ti_ab::Pin::B ? amber : (ab_on ? grey : dim_fill),
          ab_pin == ti_ab::Pin::B ? QColor(0, 0, 0) : (ab_on ? white : dim_ink),
          ab_pin == ti_ab::Pin::B ? QString("B PINNED\ntap to release")
                                  : (ti_ab::pinBConfirmed(QDateTime::currentMSecsSinceEpoch() / 1000.0, ab_pin_b_armed)
                                         ? QString("TAP AGAIN\nto pin B") : QString("PIN B")), 44);

  // which value is in force, and its number
  const QRect panel(1340, rowTop() + 10, width() - 1340 - 30, 190);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0x11, 0x11, 0x11));
  p.drawRoundedRect(panel, 24, 24);
  if (running && (arm == "A" || arm == "B")) {
    p.setPen(arm == "B" ? amber : blue);
    p.setFont(InterFont(130, QFont::Bold));
    p.drawText(QRect(panel.left() + 20, panel.top(), 150, panel.height()), Qt::AlignCenter, arm);
    p.setPen(white);
    p.setFont(InterFont(70, QFont::Bold));
    const QJsonValue value = ab_status.value("value");
    const QString number = value.isBool() ? (value.toBool() ? QString("ON") : QString("OFF")) : QString::number(value.toDouble(), 'f', 2);
    const double blend = ab_status.value("blend").toDouble();
    p.drawText(QRect(panel.left() + 180, panel.top() + 10, panel.width() - 190, 90), Qt::AlignLeft | Qt::AlignVCenter,
               number + (blend > 0.001 && blend < 0.999 ? QString("  (blending)") : QString()));
    p.setFont(InterFont(34));
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
      line += QString("   settling after a switch");
    }
    p.setPen(amber);
    p.drawText(QRect(panel.left() + 180, panel.top() + 100, panel.width() - 190, 80), Qt::AlignLeft | Qt::AlignVCenter, line);
  } else {
    p.setPen(QColor(0xc9, 0xc9, 0xc9));
    p.setFont(InterFont(44, QFont::DemiBold));
    const QString msg = ab_status.value("msg").toString();
    p.drawText(panel.adjusted(20, 0, -20, 0), Qt::AlignLeft | Qt::AlignVCenter | Qt::TextWordWrap,
               !ab_on ? QString("A/B is off: your own value is in force")
                      : (msg.isEmpty() ? QString("Waiting for engagement") : msg));
  }

  // the configuration: only while A/B is off
  const bool edit = ti_ab::canEdit(ab_on);
  const QColor box = edit ? grey : dim_fill;
  const QColor ink = edit ? white : dim_ink;
  drawBox(p, abBox(33), box, ink, QString("<"), 56);
  drawBox(p, abBox(34), box, ink, QString(">"), 56);
  drawBox(p, abBox(35), (edit && ab_a_known && !s.is_bool) ? grey : dim_fill, (edit && ab_a_known && !s.is_bool) ? white : dim_ink, QString("-"), 56);
  drawBox(p, abBox(36), (edit && ab_a_known && !s.is_bool) ? grey : dim_fill, (edit && ab_a_known && !s.is_bool) ? white : dim_ink, QString("+"), 56);
  drawBox(p, abBox(37), box, ink, QString("-"), 56);
  drawBox(p, abBox(38), box, ink, QString("+"), 56);
  p.setPen(white);
  p.setFont(InterFont(46, QFont::Bold));
  p.drawText(QRect(abBox(33).right() + 8, abBox(33).top(), abBox(34).left() - abBox(33).right() - 16, 120), Qt::AlignCenter, QString(s.name));
  p.setFont(InterFont(40));
  p.drawText(QRect(abBox(34).right() + 30, abBox(34).top(), 250, 120), Qt::AlignCenter,
             QString("A (yours)\n") + (ab_a_known ? abValueText(s, ab_a) : QString("unreadable")));
  p.drawText(QRect(abBox(35).right() + 8, abBox(35).top(), abBox(36).left() - abBox(35).right() - 16, 120), Qt::AlignCenter,
             QString("B\n") + abValueText(s, ab_b));
  p.drawText(QRect(abBox(37).right() + 8, abBox(37).top(), abBox(38).left() - abBox(37).right() - 16, 120), Qt::AlignCenter,
             QString("Block\n%1 min").arg((int)ab_block_min));

  // engaged minutes per side and per speed window (the wash-in after each switch is not counted)
  const int table_top = abBox(33).bottom() + 40;
  const int row_h = std::max(36, std::min(60, (height() - 110 - table_top) / 6));
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

  // refusals and advice
  if (!ab_on && !ab_a_known) {
    return QString("Your stored value of %1 cannot be read: A/B cannot start").arg(s.name);
  }
  if (ab_on && ab_status.value("msg").toString().size() > 0 && ab_fresh) {
    return ab_status.value("msg").toString();
  }
  if (ti_ab::bendsOnly(s.name)) {
    return QString("%1 acts only in bends: minutes count only there").arg(s.name);
  }
  return edit ? QString("Changes apply when you start A/B") : QString();
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
    drawGate(p, x0, y, x1 - x0, def.settings.empty() ? QString() : def.settings[0].first);
  }

  // One bar per bin (or one for a test without bins), then "other speeds"; the current bin outlined
  const int lines = def.bins.empty() ? 1 : (int)def.bins.size() + 1;
  const int area_top = top + 270;
  const int lh = std::max(30, std::min(90, (top + row_h - 20 - area_top) / lines));
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
  drawGate(p, x0, y, x1 - x0, QString());

  // One row per window, the one the car is inside outlined, the target marked; the test running now in bold with a bullet
  const int lines = (int)bands->bins.size();
  const int area_top = top + 270;
  const int lh = std::max(30, std::min(90, (top + row_h - 20 - area_top) / lines));
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

void TestsWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.fillRect(rect(), QColor(0, 0, 0));

  const QColor white(0xff, 0xff, 0xff);
  const QColor dim_fill(0x1a, 0x1a, 0x1a);
  const QColor dim_ink(0x55, 0x55, 0x55);
  const QColor amber(218, 202, 37);
  drawBox(p, backRect(), locked ? dim_fill : QColor(0x33, 0x33, 0x33), locked ? dim_ink : white, QString("BACK"), 70);
  drawBox(p, allOffRect(), QColor(201, 34, 49), white, ab_page ? QString("A/B OFF") : QString("ALL TESTS OFF"), 60);
  if (!ab_page) {
    drawBox(p, abRect(), ab_memory.getBool("TiAbOn") ? QColor(0x17, 0x86, 0x44) : QColor(0x33, 0x33, 0x33), white,
            ab_memory.getBool("TiAbOn") ? QString("A/B ON >") : QString("A/B >"), 40);
  }
  // The lock, always visible: amber and solid when locked (the page stays until it is unlocked), plain when not
  drawBox(p, lockRect(), locked ? amber : QColor(0x33, 0x33, 0x33), locked ? QColor(0, 0, 0) : white,
          locked ? QString("LOCKED · tap to unlock") : QString("LOCK PAGE"), 36);
  if (locked) {
    QPen pen(amber);
    pen.setWidth(10);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    p.drawRect(rect().adjusted(5, 5, -5, -5));
  }

  // Whether the system is engaged, always visible here: this page hides the road view and its engaged border. The same
  // status the road view colours its border by (UIState::status): green engaged, amber override, grey not engaged.
  {
    const UIStatus st = uiState()->status;
    const bool engaged = st != STATUS_DISENGAGED && st != STATUS_OVERRIDE;
    const QRect pill(width() - 520, height() - 84, 490, 64);
    drawBox(p, pill, engaged ? QColor(0x17, 0x86, 0x44) : st == STATUS_OVERRIDE ? amber : QColor(0x54, 0x54, 0x54),
            (engaged || st != STATUS_OVERRIDE) ? white : QColor(0, 0, 0),
            engaged ? QString("● ENGAGED") : st == STATUS_OVERRIDE ? QString("OVERRIDE") : QString("○ NOT ENGAGED"), 40);
  }

  const int n = tests.queueSize();
  const int n_defs = (int)tests.defs.size();
  QString banner;
  QColor banner_color(0x33, 0xcc, 0x66);
  if (ab_page) {
    drawBox(p, runAllRect(), QColor(0x33, 0x33, 0x33), white, QString("< TEST PAGES"), 40);
    p.setPen(white);
    p.setFont(InterFont(56, QFont::Bold));
    p.drawText(QRect(backRect().right() + 20, 24, allOffRect().left() - backRect().right() - 40, 150), Qt::AlignCenter, QString("A/B ALTERNATION"));
    banner = drawAb(p);
    banner_color = QColor(218, 202, 37);
  } else if (queue_pos < 0 || queue_pos >= n) {
    p.setPen(white);
    p.setFont(InterFont(80, QFont::Bold));
    p.drawText(QRect(backRect().right() + 20, 24, allOffRect().left() - backRect().right() - 40, 150), Qt::AlignCenter, QString("TESTS"));
    p.setFont(InterFont(56, QFont::DemiBold));
    p.drawText(QRect(30, rowTop(), width() - 60, rowHeight()), Qt::AlignCenter, QString("No valid test definitions"));
  } else if (overview) {
    // The overview of run all: no PREV / NEXT; TEST PAGES goes back to the individual tests (run all off)
    drawBox(p, runAllRect(), QColor(0x33, 0x33, 0x33), white, QString("< TEST PAGES"), 40);
    p.setPen(white);
    p.setFont(InterFont(56, QFont::Bold));
    p.drawText(QRect(backRect().right() + 20, 24, allOffRect().left() - backRect().right() - 40, 150), Qt::AlignCenter, QString("RUN ALL TESTS"));
    drawAll(p);
    banner = overview_banner;
  } else {
    // The queue: PREV / TEST k OF n / NEXT; an end of the queue greyed out, NEXT green once the shown test is done
    const QColor green(0x17, 0x86, 0x44);
    const bool finished = tests.done(queue_pos);
    const bool has_prev = queue_pos > 0;
    const bool has_next = queue_pos + 1 < n;
    drawBox(p, prevRect(), has_prev ? QColor(0x33, 0x33, 0x33) : dim_fill, has_prev ? white : dim_ink, QString("< PREV"), 56);
    drawBox(p, nextRect(), has_next ? (finished ? green : QColor(0x33, 0x33, 0x33)) : dim_fill, has_next ? white : dim_ink,
            QString("NEXT >"), 56);
    p.setPen(white);
    p.setFont(InterFont(48, QFont::Bold));
    p.drawText(QRect(prevRect().right() + 10, 24, nextRect().left() - prevRect().right() - 20, 150), Qt::AlignCenter,
               QString("TEST %1 OF %2").arg(queue_pos + 1).arg(n));
    if (tests.hasStepTests()) {
      drawBox(p, runAllRect(), QColor(0x46, 0x5B, 0xEA), white, QString("RUN ALL >"), 40);
    }

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
  if (!banner.isEmpty()) {
    p.setPen(banner_color);
    p.setFont(InterFont(34, QFont::Bold));
    p.drawText(bannerRect(), Qt::AlignCenter, banner);
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
    p.drawText(QRect(30, height() - 20 - 50 * (shown - k), width() - 580, 50), Qt::AlignLeft | Qt::AlignVCenter, line);
  }
}

// Sidebar

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
