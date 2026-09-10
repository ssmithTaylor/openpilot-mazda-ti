from .controller_replay import instrument
from .runtime import controller_source


def test_optional_prompt_unload_trace_fields_preserve_baseline_source():
  source = instrument(controller_source("7c3819b92b63e8cd0f88dafc3de6cac415185080"))
  assert 'prompt_unload=getattr(self,"prompt_unload",0.0)' in source
  assert 'prompt_unload_active=getattr(self,"prompt_unload_active",False)' in source
  compile(source, "<instrumented-controller>", "exec")
