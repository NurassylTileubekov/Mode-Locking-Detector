"""Entry point for the MLD Configuration Tool desktop application."""

import ctypes
import os
import sys
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication

from main_window import MLDConfigWindow


def resource_path(relative_path: str) -> str:
    """Resolve resource path for both dev environment and PyInstaller bundle."""
    base_path = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)


def main() -> int:
    if sys.platform == 'win32':
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID('MLD.MLDConfigTool.1')

    app = QApplication(sys.argv)
    icon_path = resource_path('logo.ico')
    if os.path.exists(icon_path):
        app_icon = QIcon(icon_path)
        app.setWindowIcon(app_icon)

    window = MLDConfigWindow()
    window.show()
    return app.exec()


if __name__ == '__main__':
    sys.exit(main())