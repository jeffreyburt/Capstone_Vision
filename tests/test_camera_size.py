"""Robot camera size option for head tracking (head_tracker/camera_feed.py)."""

import pytest

from head_tracker.camera_feed import DEFAULT_SIZE, parse_size


def test_default_is_320x240():
    assert DEFAULT_SIZE == (320, 240)


@pytest.mark.parametrize("text,size", [("320x240", (320, 240)), ("640X480", (640, 480)), ("160x120", (160, 120))])
def test_parse_size_accepts(text, size):
    assert parse_size(text) == size


@pytest.mark.parametrize("text", ["320", "abc", "321x240", "100x80", "2000x1500", "320x241"])
def test_parse_size_rejects(text):
    with pytest.raises(ValueError):
        parse_size(text)
