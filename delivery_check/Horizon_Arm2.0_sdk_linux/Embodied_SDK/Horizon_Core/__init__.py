#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lazy entry points for Horizon_Core.

ROS2 and headless Linux deployments often only need `core`, `Control_SDK`, and
`gateway`. Importing `AI_SDK` eagerly pulls in optional Python dependencies such
as `python-dotenv`, which should not block motor-control use cases.
"""

from __future__ import annotations

import importlib


_EXPORTS = {
    "core": (".core", "module"),
    "Control_SDK": (".Control_SDK", "module"),
    "AI_SDK": (".AI_SDK", "module"),
    "gateway": (".gateway", "module"),
}


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

    module_name, kind = _EXPORTS[name]
    module = importlib.import_module(module_name, __name__)
    value = module if kind == "module" else getattr(module, kind)
    globals()[name] = value
    return value


__all__ = [
    "core",
    "Control_SDK",
    "AI_SDK",
    "gateway",
]
