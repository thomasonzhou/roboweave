#!/usr/bin/env python3

from PySide6.QtCore import Qt, QRect, QTimer
from PySide6.QtGui import QPainter, QColor, QKeyEvent
from PySide6.QtWidgets import QApplication, QWidget, QHBoxLayout, QLabel


class GameWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Game Widget - Move Red Square with WASD/Arrow Keys")
        self.setFixedSize(400, 400)
        self.setFocusPolicy(Qt.StrongFocus)  # Allow widget to receive keyboard events
        
        # Square dimensions
        self.square_size = 30
        
        # Blue square position (fixed at center)
        self.blue_x = (400 - self.square_size) // 2
        self.blue_y = (400 - self.square_size) // 2
        
        # Red square initial position (top-left corner)
        self.red_x = 10
        self.red_y = 10
        
        # Movement step size
        self.step_size = 10
        
    def paintEvent(self, event):
        """Handle painting of the widget"""
        painter = QPainter(self)
        
        # Draw white background
        painter.fillRect(self.rect(), QColor(255, 255, 255))
        
        # Draw blue square at center
        blue_rect = QRect(self.blue_x, self.blue_y, self.square_size, self.square_size)
        painter.fillRect(blue_rect, QColor(0, 0, 255))
        
        # Draw red square at current position
        red_rect = QRect(self.red_x, self.red_y, self.square_size, self.square_size)
        painter.fillRect(red_rect, QColor(255, 0, 0))
        
    def keyPressEvent(self, event: QKeyEvent):
        """Handle keyboard input for moving the red square"""
        key = event.key()
        
        # Store current position
        old_x, old_y = self.red_x, self.red_y
        
        # Handle movement keys
        if key == Qt.Key_W or key == Qt.Key_Up:
            self.red_y -= self.step_size
        elif key == Qt.Key_S or key == Qt.Key_Down:
            self.red_y += self.step_size
        elif key == Qt.Key_A or key == Qt.Key_Left:
            self.red_x -= self.step_size
        elif key == Qt.Key_D or key == Qt.Key_Right:
            self.red_x += self.step_size
        else:
            # If key is not handled, pass to parent
            super().keyPressEvent(event)
            return
        
        # Clamp red square to widget boundaries
        self.red_x = max(0, min(self.red_x, 400 - self.square_size))
        self.red_y = max(0, min(self.red_y, 400 - self.square_size))
        
        # Only update if position changed
        if (self.red_x, self.red_y) != (old_x, old_y):
            self.update()  # Trigger repaint


class Main(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GameWidget with Live Preview")
        self.setFixedSize(800, 400)  # Two 400x400 panes side by side
        
        # Create layout
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # Create GameWidget instance
        self.game_widget = GameWidget()
        
        # Create preview label
        self.preview_label = QLabel()
        self.preview_label.setFixedSize(400, 400)
        self.preview_label.setStyleSheet("border: 1px solid black;")
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setScaledContents(True)  # Scale pixmap to fit label
        
        # Add widgets to layout
        layout.addWidget(self.game_widget)
        layout.addWidget(self.preview_label)
        
        # Set layout
        self.setLayout(layout)
        
        # Set up timer for live preview
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_preview)
        self.timer.start(100)  # Update every 100ms
        
        # Make sure the game widget has focus for keyboard input
        self.game_widget.setFocus()
    
    def update_preview(self):
        """Capture the game widget and update the preview label"""
        # Grab the current contents of the game widget
        pixmap = self.game_widget.grab()
        
        # Set the pixmap on the preview label
        self.preview_label.setPixmap(pixmap)
    
    def keyPressEvent(self, event: QKeyEvent):
        """Forward key events to the game widget"""
        self.game_widget.keyPressEvent(event)


if __name__ == "__main__":
    app = QApplication([])
    
    # Create and show the main widget
    main_widget = Main()
    main_widget.show()
    
    # Start the application event loop
    app.exec() 