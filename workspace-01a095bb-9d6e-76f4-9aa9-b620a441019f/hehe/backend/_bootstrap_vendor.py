#!/usr/bin/env python3
"""
Bootstrap script to make vendor shims available as system packages.
Must be imported before any backend module that needs pydantic/pymavlink/fastapi.

Usage:
    import _bootstrap_vendor  # adds _vendor/ to sys.path
    # Now regular imports like 'from pymavlink import mavutil' work
"""

import os
import sys

# Add the _vendor directory to the beginning of sys.path
# so our shim packages are found before (non-existent) system packages
_vendor_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_vendor")
if _vendor_dir not in sys.path:
    sys.path.insert(0, _vendor_dir)

# Also add the backend directory itself
_backend_dir = os.path.dirname(os.path.abspath(__file__))
if _backend_dir not in sys.path:
    sys.path.insert(0, _backend_dir)
