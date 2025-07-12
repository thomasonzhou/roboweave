"""A PySide6 game widget with movable red square, live preview, and Gemini navigation oracle."""

import asyncio, sys
from pathlib import Path
from PySide6.QtCore import Qt, QRect, QTimer, QThread, pyqtSignal, QBuffer, QIODevice
from PySide6.QtGui import QPainter, QColor, QKeyEvent
from PySide6.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPlainTextEdit

sys.path.insert(0, str(Path(__file__).parent / 'capture_demo'))
try:
    from live_api_client import next_move
except ImportError:
    next_move = None


class NavigationWorker(QThread):
    move_suggested = pyqtSignal(str)
    analysis_error = pyqtSignal(str)
    
    def __init__(self, pixmap_bytes):
        super().__init__()
        self.pixmap_bytes = pixmap_bytes
        
    def run(self):
        try:
            if next_move is None:
                self.analysis_error.emit("Navigation oracle not available. Check capture_demo setup.")
                return
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            direction = loop.run_until_complete(next_move(self.pixmap_bytes))
            self.move_suggested.emit(direction)
        except Exception as e:
            self.analysis_error.emit(f"Navigation oracle failed: {str(e)}")
        finally:
            if 'loop' in locals():
                loop.close()


class GameWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(400, 400)
        self.setFocusPolicy(Qt.StrongFocus)
        self.square_size = 30
        self.step_size = 10
        self.blue_x = (400 - self.square_size) // 2
        self.blue_y = (400 - self.square_size) // 2
        self.red_x = 10
        self.red_y = 10
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(255, 255, 255))
        painter.fillRect(QRect(self.blue_x, self.blue_y, self.square_size, self.square_size), QColor(0, 0, 255))
        painter.fillRect(QRect(self.red_x, self.red_y, self.square_size, self.square_size), QColor(255, 0, 0))
        
    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        old_x, old_y = self.red_x, self.red_y
        if key == Qt.Key_W or key == Qt.Key_Up:
            self.red_y -= self.step_size
        elif key == Qt.Key_S or key == Qt.Key_Down:
            self.red_y += self.step_size
        elif key == Qt.Key_A or key == Qt.Key_Left:
            self.red_x -= self.step_size
        elif key == Qt.Key_D or key == Qt.Key_Right:
            self.red_x += self.step_size
        else:
            super().keyPressEvent(event)
            return
        self.red_x = max(0, min(self.red_x, 400 - self.square_size))
        self.red_y = max(0, min(self.red_y, 400 - self.square_size))
        if (self.red_x, self.red_y) != (old_x, old_y):
            self.update()
    
    def auto_move(self, direction):
        old_x, old_y = self.red_x, self.red_y
        if direction == "Up":
            self.red_y -= self.step_size
        elif direction == "Down":
            self.red_y += self.step_size
        elif direction == "Left":
            self.red_x -= self.step_size
        elif direction == "Right":
            self.red_x += self.step_size
        self.red_x = max(0, min(self.red_x, 400 - self.square_size))
        self.red_y = max(0, min(self.red_y, 400 - self.square_size))
        if (self.red_x, self.red_y) != (old_x, old_y):
            self.update()


class Main(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GameWidget with Navigation Oracle")
        self.setFixedSize(800, 600)
        
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        self.game_widget = GameWidget()
        
        right_layout = QVBoxLayout()
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(0)
        
        self.preview_label = QLabel()
        self.preview_label.setFixedSize(400, 400)
        self.preview_label.setStyleSheet("border: 1px solid black;")
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setScaledContents(True)
        
        self.reply_box = QPlainTextEdit()
        self.reply_box.setReadOnly(True)
        self.reply_box.setFixedHeight(200)
        self.reply_box.setPlaceholderText("Press SPACE for AI navigation guidance, ENTER for auto-move...")
        
        right_layout.addWidget(self.preview_label)
        right_layout.addWidget(self.reply_box)
        
        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        
        main_layout.addWidget(self.game_widget)
        main_layout.addWidget(right_widget)
        self.setLayout(main_layout)
        
        self.latest_pix = None
        self.latest_direction = None
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_preview)
        self.timer.start(100)
        
        self.nav_worker = None
        self.game_widget.setFocus()
    
    def update_preview(self):
        self.latest_pix = self.game_widget.grab()
        self.preview_label.setPixmap(self.latest_pix)
    
    def pixmap_to_bytes(self, pixmap):
        buffer = QBuffer()
        buffer.open(QIODevice.WriteOnly)
        pixmap.save(buffer, "PNG")
        return buffer.data().data()
    
    def get_navigation_guidance(self):
        if self.latest_pix is None:
            self.reply_box.appendPlainText("No image to analyze yet...")
            return
        if self.nav_worker and self.nav_worker.isRunning():
            self.reply_box.appendPlainText("Navigation analysis already in progress...")
            return
        pixmap_bytes = self.pixmap_to_bytes(self.latest_pix)
        self.nav_worker = NavigationWorker(pixmap_bytes)
        self.nav_worker.move_suggested.connect(self.on_move_suggested)
        self.nav_worker.analysis_error.connect(self.on_navigation_error)
        self.nav_worker.start()
        self.reply_box.appendPlainText("🧭 Consulting navigation oracle...")
    
    def on_move_suggested(self, direction):
        self.latest_direction = direction
        self.reply_box.appendPlainText(f"\n🎯 Oracle suggests: {direction}")
        self.reply_box.appendPlainText("Press ENTER to auto-move or continue manually with WASD")
        self.reply_box.appendPlainText("="*50)
        self.reply_box.ensureCursorVisible()
    
    def on_navigation_error(self, error):
        self.reply_box.appendPlainText(f"\n❌ Navigation error: {error}\n" + "="*50)
        self.reply_box.ensureCursorVisible()
    
    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key_Space:
            self.get_navigation_guidance()
        elif event.key() == Qt.Key_Return and self.latest_direction:
            self.game_widget.auto_move(self.latest_direction)
            self.reply_box.appendPlainText(f"🤖 Auto-moved: {self.latest_direction}")
            self.latest_direction = None
        else:
            self.game_widget.keyPressEvent(event)


if __name__ == "__main__":
    app = QApplication([])
    main_widget = Main()
    main_widget.show()
    app.exec()


"""
Run:
pip install PySide6 google-genai
export GEMINI_API_KEY="YOUR-KEY"
python wasd_stream.py

Controls: WASD/arrows=Manual, SPACE=AI guidance, ENTER=Auto-move
""" 