#include "selfdrive/ui/qt/sidebar.h"

#include <algorithm>

#include <QDateTime>
#include <QDebug>
#include <QJsonArray>
#include <QMouseEvent>
#include <QPainter>

#include "common/util.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/input.h"

// Mazda TI on-car tests. Mirrors selfdrive/car/mazda/ti_tests.py; selfdrive/car/tests/test_mazda_ti_tests.py checks
// that these limits, key sets and the allow-list agree with the Python copy.
namespace {

constexpr int TI_TEST_MAX_TESTS = 2;
constexpr int TI_TEST_MAX_SETTINGS = 3;
constexpr int TI_TEST_MAX_BINS = 5;
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
constexpr int TI_TEST_IDLE_CLOSE_MS = 15000;  // the Tests page closes itself after this long without a touch
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

struct TiTestAllowed {
  const char *key;     // the param a definition may switch
  const char *toggle;  // the frogpilot_toggles value that shows what the car applies
  bool on;             // the value that means "on"; "off" is the other one
};
const TiTestAllowed TI_TEST_ALLOWED[] = {
  {"TiDither", "ti_dither", true},
};

const QStringList TI_TEST_ACTIVE_SOURCES = {"TiDither"};
const QStringList TI_TEST_METRICS = {"active_seconds", "engaged_seconds"};
const QStringList TI_TEST_BIN_BY = {"speed_mph"};
const QStringList TI_TEST_SWITCH_WHEN = {"straight_only", "between_engagements"};
const QStringList TI_TEST_FILE_KEYS = {"version", "tests"};
const QStringList TI_TEST_TEST_KEYS = {"id", "title", "description", "settings", "progress", "auto_off", "alternate"};
const QStringList TI_TEST_TEST_REQUIRED = {"id", "title", "description", "settings", "progress"};
const QStringList TI_TEST_SETTING_KEYS = {"key", "on"};
const QStringList TI_TEST_PROGRESS_KEYS = {"metric", "target_seconds", "bin_by", "bins"};
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

QString tiTestBins(const QJsonValue &raw, std::vector<TiTestBin> &out) {
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
  if (p.contains("bins")) {
    if (!TI_TEST_BIN_BY.contains(p.value("bin_by").toString())) {
      return QString("unknown bin_by");
    }
    why = tiTestBins(p.value("bins"), def.bins);
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
  setTest(i, !views[i].requested);
  views[i].mismatch_ticks = 0;
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
  }
}

// At the end of a drive (onroad -> offroad): switch off the tests that ask for it, and copy the live progress to
// flash so the last minute of counting survives a power-down before the next drive
void TiTests::driveEnded() {
  bool changed = false;
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
    if (bin.required && views[i].seconds.value(bin.label, 0.0) < def.target) {
      return false;
    }
  }
  return true;
}

int TiTests::countOn() const {
  int n = 0;
  for (const TiTestView &view : views) {
    n += view.any_on ? 1 : 0;
  }
  return n;
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
        sum += std::min(1.0, views[i].seconds.value(bin.label, 0.0) / def.target);
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
  idle.start();
  opened.start();
  pressed_control = 0;
  tick_count = 0;
  refresh();
  return true;
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

  // Back to the driving screen at the end of the drive, after 15 s without a touch, and at once for ANY alert the
  // road view would show
  if (!s.scene.started || idle.elapsed() > TI_TEST_IDLE_CLOSE_MS || tiTestsAlertShowing(s, *frogpilotUIState())) {
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
  update();
}

int TestsWindow::rowTop(int i) const {
  return 210 + i * rowHeight();
}

int TestsWindow::rowHeight() const {
  const int footer = tests.errors.isEmpty() ? 0 : 30 + 50 * std::min(2, (int)tests.errors.size());
  const int n = std::max(1, (int)tests.defs.size());
  return (height() - 20 - footer - 210) / n;
}

QRect TestsWindow::backRect() const {
  return QRect(30, 24, 440, 150);
}

QRect TestsWindow::allOffRect() const {
  return QRect(width() - 670, 24, 640, 150);
}

QRect TestsWindow::toggleRect(int i) const {
  return QRect(30, rowTop(i) + 10, 600, rowHeight() - 30);
}

QRect TestsWindow::resetRect(int i) const {
  return QRect(width() - 430, rowTop(i) + 10, 400, 110);
}

// Controls: 1 BACK, 2 ALL TESTS OFF, 10 + i the on/off box of test i, 20 + i its Reset progress; 0 none
int TestsWindow::controlAt(const QPoint &pos) const {
  if (backRect().contains(pos)) {
    return 1;
  }
  if (allOffRect().contains(pos)) {
    return 2;
  }
  for (int i = 0; i < (int)tests.defs.size(); i++) {
    if (toggleRect(i).contains(pos)) {
      return 10 + i;
    }
    if (resetRect(i).contains(pos)) {
      return 20 + i;
    }
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
    leave();
    return;
  }
  if (control == 2) {
    tests.allOff();
    refresh();
    return;
  }
  const int i = control >= 20 ? control - 20 : control - 10;
  if (i < 0 || i >= (int)tests.defs.size()) {
    return;
  }
  if (control < 20) {
    tests.toggle(i);
    refresh();
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

void TestsWindow::drawBox(QPainter &p, const QRect &r, const QColor &fill, const QColor &ink, const QString &text, int px) const {
  p.setPen(Qt::NoPen);
  p.setBrush(fill);
  p.drawRoundedRect(r, 24, 24);
  p.setPen(ink);
  p.setFont(InterFont(px, QFont::Bold));
  p.drawText(r, Qt::AlignCenter, text);
}

void TestsWindow::drawTest(QPainter &p, int i) {
  if (i >= (int)tests.views.size()) {
    return;
  }
  const TiTestDef &def = tests.defs[i];
  const TiTestView &view = tests.views[i];
  const QColor white(0xff, 0xff, 0xff);
  const QColor grey(0xa0, 0xa0, 0xa0);
  const int top = rowTop(i);
  const int row_h = rowHeight();

  // The on/off control: state in colour and words
  const QRect toggle_r = toggleRect(i);
  const QColor ink = view.light ? QColor(0, 0, 0) : white;
  p.setPen(Qt::NoPen);
  p.setBrush(view.color);
  p.drawRoundedRect(toggle_r, 30, 30);
  p.setPen(ink);
  p.setFont(InterFont(64, QFont::Bold));
  p.drawText(toggle_r.adjusted(10, 10, -10, -toggle_r.height() / 3), Qt::AlignCenter, view.state);
  p.setFont(InterFont(36));
  p.drawText(toggle_r.adjusted(10, toggle_r.height() * 2 / 3 - 20, -10, def.off_at_ignition_off ? -70 : -10), Qt::AlignCenter,
             view.requested ? QString("tap to switch off") : QString("tap to switch on"));
  if (def.off_at_ignition_off) {
    p.setFont(InterFont(28));
    p.drawText(QRect(toggle_r.x() + 10, toggle_r.bottom() - 60, toggle_r.width() - 20, 50), Qt::AlignCenter,
               QString("turns off when the car is switched off"));
  }

  drawBox(p, resetRect(i), QColor(0x33, 0x33, 0x33), white, QString("Reset progress"), 40);

  // Title (DONE when every required bin has its time), the car side's arm and status lines when it publishes them
  const int x0 = toggle_r.right() + 40;
  const int x1 = width() - 30;
  const int head_w = resetRect(i).left() - 20 - x0;
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
    p.drawText(QRect(x0, top + 75, head_w, 45), Qt::AlignLeft | Qt::AlignVCenter, view.status);
  }

  // One bar per bin (or one for a test without bins), then "other speeds"; the current bin outlined
  const int lines = def.bins.empty() ? 1 : (int)def.bins.size() + 1;
  const int area_top = top + 135;
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
    if (!def.bins.empty() && k < (int)def.bins.size()) {
      key = def.bins[k].label;
      label = TiTests::binText(def.bins[k], is_metric);
      required = def.bins[k].required;
    } else if (!def.bins.empty()) {
      key = QString(TI_TEST_OTHER);
      label = QString("other speeds");
      has_bar = false;
    }
    const double collected = view.seconds.value(key, 0.0);
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
      const double fraction = std::min(1.0, std::max(0.0, collected / def.target));
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
    }
    p.setPen(required ? white : grey);
    p.setFont(InterFont(px, QFont::DemiBold));
    p.drawText(QRect(x1 - value_w, line_y, value_w, lh), Qt::AlignRight | Qt::AlignVCenter, value);
  }
}

void TestsWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.fillRect(rect(), QColor(0, 0, 0));

  const QColor white(0xff, 0xff, 0xff);
  drawBox(p, backRect(), QColor(0x33, 0x33, 0x33), white, QString("BACK"), 70);
  drawBox(p, allOffRect(), QColor(201, 34, 49), white, QString("ALL TESTS OFF"), 60);
  p.setPen(white);
  p.setFont(InterFont(80, QFont::Bold));
  p.drawText(QRect(backRect().right() + 20, 24, allOffRect().left() - backRect().right() - 40, 150), Qt::AlignCenter, QString("TESTS"));

  if (tests.defs.empty()) {
    p.setFont(InterFont(56, QFont::DemiBold));
    p.drawText(QRect(30, rowTop(0), width() - 60, rowHeight()), Qt::AlignCenter, QString("No valid test definitions"));
  }
  for (int i = 0; i < (int)tests.defs.size(); i++) {
    drawTest(p, i);
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
    p.drawText(QRect(30, height() - 20 - 50 * (shown - k), width() - 60, 50), Qt::AlignLeft | Qt::AlignVCenter, line);
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
