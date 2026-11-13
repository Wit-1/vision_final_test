"""Run our plugin in a dedicated rqt configuration, independent of other rqt windows."""

import sys

from python_qt_binding.QtCore import QLocale
from rqt_gui.main import Main


def main():
    QLocale.setDefault(QLocale(QLocale.English, QLocale.UnitedStates))
    return Main(settings_filename='vision_final_test_gui').main(
        argv=[sys.argv[0], '--force-discover', *sys.argv[1:]],
        standalone='ft_rqt_gui/ControlPanel',
    )


if __name__ == '__main__':
    raise SystemExit(main())
