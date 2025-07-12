"""A PySide6 game widget with movable red square and live preview."""

from PySide6.QtCore import Qt, QRect, QTimer
from PySide6.QtGui import QPainter, QColor, QKeyEvent
from PySide6.QtWidgets import QApplication, QWidget, QHBoxLayout, QLabel


class GameWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(400, 400)
        self.setFocusPolicy(Qt.StrongFocus)
        
        self.square_size = 30
        self.step_size = 10
        
        # Blue square fixed at center
        self.blue_x = (400 - self.square_size) // 2
        self.blue_y = (400 - self.square_size) // 2
        
        # Red square starts at top-left
        self.red_x = 10
        self.red_y = 10
        
    def paintEvent(self, event):
        painter = QPainter(self)
        
        # White background
        painter.fillRect(self.rect(), QColor(255, 255, 255))
        
        # Blue square at center
        blue_rect = QRect(self.blue_x, self.blue_y, self.square_size, self.square_size)
        painter.fillRect(blue_rect, QColor(0, 0, 255))
        
        # Red square at current position
        red_rect = QRect(self.red_x, self.red_y, self.square_size, self.square_size)
        painter.fillRect(red_rect, QColor(255, 0, 0))
        
    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        old_x, old_y = self.red_x, self.red_y
        
        # Handle WASD and arrow keys
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
        
        # Clamp to boundaries
        self.red_x = max(0, min(self.red_x, 400 - self.square_size))
        self.red_y = max(0, min(self.red_y, 400 - self.square_size))
        
        # Update display if position changed
        if (self.red_x, self.red_y) != (old_x, old_y):
            self.update()


class Main(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GameWidget with Live Preview")
        self.setFixedSize(800, 400)
        
        # Layout for side-by-side arrangement
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # Game widget on left
        self.game_widget = GameWidget()
        
        # Preview label on right
        self.preview_label = QLabel()
        self.preview_label.setFixedSize(400, 400)
        self.preview_label.setStyleSheet("border: 1px solid black;")
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setScaledContents(True)
        
        layout.addWidget(self.game_widget)
        layout.addWidget(self.preview_label)
        self.setLayout(layout)
        
        # Timer for live preview updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_preview)
        self.timer.start(100)
        
        self.game_widget.setFocus()
    
    def update_preview(self):
        """Capture game widget and update preview."""
        pixmap = self.game_widget.grab()
        self.preview_label.setPixmap(pixmap)
    
    def keyPressEvent(self, event: QKeyEvent):
        """Forward key events to game widget."""
        self.game_widget.keyPressEvent(event)


if __name__ == "__main__":
    app = QApplication([])
    main_widget = Main()
    main_widget.show()
    app.exec()


"""
Run:
pip install PySide6
python wasd_stream.py

Use WASD or arrow keys to move the red square.
The left pane shows the game, the right pane shows a live preview.
""" 