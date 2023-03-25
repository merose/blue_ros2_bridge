from flake8.api import legacy as flake8

import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """
    Tests source code using the Flake8 plugin.

    This replaces the old flake8 test added automatically when creating
    the Python ROS package using ros2. Unfortunately, the old unit test
    code added by "ros2 pkg create" uses a wrapper class in ament_python
    that does not work with more recent flake8 releases. Instead, we
    call flake8 directly.

    Uses the Google Python standards, as implemented by ament_lint
    ignoring certain flake8 rules. The configuration is from:
    https://github.com/ament/ament_lint/blob/rolling/ament_flake8/ament_flake8/configuration/ament_flake8.ini
    """
    style_guide = flake8.get_style_guide(
        extend_ignore=['B902', 'C816', 'D100', 'D101', 'D102', 'D103', 'D104',
                       'D105', 'D106', 'D107', 'D203', 'D212', 'D404', 'I202'],
        import_order_style='google',
        max_line_length=110,
        show_source=True,
        statistics=True)
    report = style_guide.check_files()
    assert report.get_statistics('E') == [], 'Flake8 found violations'
