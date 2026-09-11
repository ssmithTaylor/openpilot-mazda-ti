from tools.mazda_ti.feature_intersection_audit import SETTINGS, settings_from


def test_settings_from_keeps_unrecorded_controller_fields_unknown():
    recorded = {"settings": {"LatDamping": "1", "NNFF": "0"}}

    settings = settings_from(recorded)

    assert settings["LatDamping"] == "1"
    assert settings["LatCommitSetpoint"] is None
    assert settings["LatFrictionComp"] is None
    assert settings["TorqueInterceptorEnabled"] is None
    assert set(settings) == set(SETTINGS)
