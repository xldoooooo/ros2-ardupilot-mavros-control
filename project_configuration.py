"""Resolve repository-owned aircraft parameters for source and installed entry points."""

from pathlib import Path
import re


# The selector is host-local; simulation is the default for an unconfigured ground host.
SELECTOR_NAME = "aircraft.local.env"


def project_root(start=None) -> Path:
    """Find the checkout containing this helper or a source/installed launch file."""
    location = Path(start or __file__).resolve()
    for parent in (location, *location.parents):
        if (parent / "project_configuration.py").is_file():
            return parent
    raise FileNotFoundError(f"Project configuration root not found from {location}")


def config_dir(root=None) -> Path:
    """Return the selected aircraft directory without applying fallback parameter layers."""
    base = Path(root).resolve() if root else project_root()
    selector = base / "config" / SELECTOR_NAME
    aircraft = "simulation"
    if selector.exists():
        for line in selector.read_text().splitlines():
            key, separator, value = line.partition("=")
            if separator and key.strip() == "AIRCRAFT_ID":
                aircraft = value.split("#", 1)[0].strip().strip("\"'")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", aircraft):
        raise ValueError(f"Invalid AIRCRAFT_ID in {selector}: {aircraft!r}")
    directory = base / "config" / "aircraft" / aircraft
    if not directory.is_dir():
        raise FileNotFoundError(f"Aircraft configuration does not exist: {directory}")
    return directory


def config_file(relative, root=None) -> Path:
    """Resolve one parameter file below the selected aircraft directory."""
    return config_dir(root) / relative
