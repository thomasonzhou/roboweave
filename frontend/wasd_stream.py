"""A PySide6 game widget with movable red square, live preview, and Gemini Vision analysis."""

import asyncio
import sys
from pathlib import Path
from PySide6.QtCore import Qt, QRect, QTimer, QThread, pyqtSignal, QBuffer, QIODevice
from PySide6.QtGui import QPainter, QColor, QKeyEvent
from PySide6.QtWidgets import (QApplication, QWidget, QHBoxLayout, QVBoxLayout, 
                               QLabel, QPlainTextEdit)

sys.path.insert(0, str(Path(__file__).parent / 'capture_demo'))
try:
    from live_api_client import analyze
except ImportError:
    analyze = None


class AnalysisWorker(QThread):
    analysis_complete = pyqtSignal(str)
    analysis_error = pyqtSignal(str)
    
    def __init__(self, pixmap_bytes):
        super().__init__()
        self.pixmap_bytes = pixmap_bytes
        
    def run(self):
        try:
            if analyze is None:
                self.analysis_error.emit("Live API client not available. Check capture_demo setup.")
                return
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            result = loop.run_until_complete(
                analyze(self.pixmap_bytes, "What is depicted and what's the next step?")
            )
            self.analysis_complete.emit(result)
        except Exception as e:
            self.analysis_error.emit(f"Analysis failed: {str(e)}")
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


class Main(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GameWidget with Live Preview & Gemini Analysis")
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
        self.reply_box.setPlaceholderText("Press SPACE to analyze current view with Gemini...")
        
        right_layout.addWidget(self.preview_label)
        right_layout.addWidget(self.reply_box)
        
        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        
        main_layout.addWidget(self.game_widget)
        main_layout.addWidget(right_widget)
        self.setLayout(main_layout)
        
        self.latest_pix = None
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_preview)
        self.timer.start(100)
        
        self.analysis_worker = None
        self.game_widget.setFocus()
    
    def update_preview(self):
        self.latest_pix = self.game_widget.grab()
        self.preview_label.setPixmap(self.latest_pix)
    
    def pixmap_to_bytes(self, pixmap):
        buffer = QBuffer()
        buffer.open(QIODevice.WriteOnly)
        pixmap.save(buffer, "PNG")
        return buffer.data().data()
    
    def analyze_current_view(self):
        if self.latest_pix is None:
            self.reply_box.appendPlainText("No image to analyze yet...")
            return
            
        if self.analysis_worker and self.analysis_worker.isRunning():
            self.reply_box.appendPlainText("Analysis already in progress...")
            return
            
        pixmap_bytes = self.pixmap_to_bytes(self.latest_pix)
        self.analysis_worker = AnalysisWorker(pixmap_bytes)
        self.analysis_worker.analysis_complete.connect(self.on_analysis_complete)
        self.analysis_worker.analysis_error.connect(self.on_analysis_error)
        self.analysis_worker.start()
        self.reply_box.appendPlainText("🤔 Analyzing with Gemini...")
    
    def on_analysis_complete(self, result):
        self.reply_box.appendPlainText(f"\n🤖 Gemini says:\n{result}\n" + "="*50)
        self.reply_box.ensureCursorVisible()
    
    def on_analysis_error(self, error):
        self.reply_box.appendPlainText(f"\n❌ Error: {error}\n" + "="*50)
        self.reply_box.ensureCursorVisible()
    
    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key_Space:
            self.analyze_current_view()
        else:
            self.game_widget.keyPressEvent(event)


if __name__ == "__main__":
    app = QApplication([])
    main_widget = Main()
    main_widget.show()
    app.exec()


"""
Run:
pip install PySide6 google-generativeai
export GEMINI_API_KEY="YOUR-KEY"
python wasd_stream.py

Controls:
- WASD/arrows: Move red square
- SPACE: Analyze current view with Gemini
""" 