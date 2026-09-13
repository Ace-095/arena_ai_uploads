"""
Vendored minimal pydantic shim.
Provides just enough of the pydantic v2 API for models.py to import and work.
This is a DEVELOPMENT-ONLY shim for testing on air-gapped machines.
Replace with real pydantic when internet is available.
"""

from dataclasses import dataclass, field as _dc_field, fields as _dc_fields
from typing import Any, Callable, List, Optional, get_type_hints
import json
import copy


class _FieldInfo:
    def __init__(self, default, ge=None, le=None):
        self.default = default
        self.ge = ge
        self.le = le

def Field(default=..., *, ge=None, le=None, **kwargs):
    """Minimal pydantic.Field replacement."""
    if default is ...:
        default = None
    return _FieldInfo(default, ge, le)

def field_validator(*field_names, mode='before'):
    """Decorator shim for pydantic v2 field_validator."""
    def decorator(func):
        underlying = func.__func__ if isinstance(func, classmethod) else func
        underlying._field_validator_names = field_names
        return func
    return decorator


class _ModelMeta(type):
    """Metaclass that converts class annotations into a dataclass-like init."""
    def __new__(mcs, name, bases, namespace):
        cls = super().__new__(mcs, name, bases, namespace)
        if name == 'BaseModel':
            return cls

        # Gather annotations
        annotations = {}
        for base in reversed(cls.__mro__):
            annotations.update(getattr(base, '__annotations__', {}))

        cls.__annotations__ = annotations
        return cls


class BaseModel(metaclass=_ModelMeta):
    """Minimal pydantic BaseModel replacement using plain __init__."""

    def __init__(self, **kwargs):
        hints = {}
        for klass in reversed(type(self).__mro__):
            hints.update(getattr(klass, '__annotations__', {}))

        # Run field validators
        validators = {}
        for klass in type(self).__mro__:
            for attr_name, attr in klass.__dict__.items():
                func = attr.__func__ if isinstance(attr, classmethod) else attr
                if hasattr(func, '_field_validator_names'):
                    for fn in func._field_validator_names:
                        validators[fn] = getattr(type(self), attr_name)

        for field_name, field_type in hints.items():
            value = None
            field_info = None
            if hasattr(type(self), field_name):
                default_attr = getattr(type(self), field_name)
                if isinstance(default_attr, _FieldInfo):
                    field_info = default_attr
                    default = field_info.default
                else:
                    default = default_attr
            else:
                default = None

            if field_name in kwargs:
                value = kwargs[field_name]
            elif default is not None:
                if isinstance(default, list):
                    value = list(default)
                elif isinstance(default, dict):
                    value = dict(default)
                else:
                    value = default
            else:
                origin = getattr(field_type, '__origin__', None)
                if origin is type(None) or (hasattr(field_type, '__args__') and type(None) in getattr(field_type, '__args__', ())):
                    value = None
                else:
                    if field_type in (int, float):
                        value = field_type()
                    elif field_type is str:
                        value = ""
                    elif field_type is bool:
                        value = False
                    elif field_type is list or (hasattr(field_type, '__origin__') and getattr(field_type, '__origin__', None) is list):
                        value = []
                    else:
                        value = None
            
            # Apply validators
            if field_info:
                if field_info.ge is not None and value is not None and value < field_info.ge:
                    raise ValueError(f"{field_name} must be >= {field_info.ge}")
                if field_info.le is not None and value is not None and value > field_info.le:
                    raise ValueError(f"{field_name} must be <= {field_info.le}")
            
            if field_name in validators:
                value = validators[field_name](value)
                
            setattr(self, field_name, value)

    def model_dump(self) -> dict:
        result = {}
        hints = {}
        for klass in reversed(type(self).__mro__):
            hints.update(getattr(klass, '__annotations__', {}))
        for field_name in hints:
            val = getattr(self, field_name, None)
            if isinstance(val, BaseModel):
                result[field_name] = val.model_dump()
            elif isinstance(val, list):
                result[field_name] = [
                    item.model_dump() if isinstance(item, BaseModel) else item
                    for item in val
                ]
            else:
                result[field_name] = val
        return result

    def model_dump_json(self) -> str:
        return json.dumps(self.model_dump(), default=str)

    def model_copy(self, **kwargs):
        data = self.model_dump()
        data.update(kwargs)
        return type(self)(**data)

    def __repr__(self):
        hints = {}
        for klass in reversed(type(self).__mro__):
            hints.update(getattr(klass, '__annotations__', {}))
        fields = ", ".join(f"{k}={getattr(self, k, None)!r}" for k in hints)
        return f"{type(self).__name__}({fields})"


# Type alias shims
from typing import Literal
