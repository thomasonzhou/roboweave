"""A PySide6 game widget with movable red square, live preview, and cloud navigation."""

import asyncio, sys, os, subprocess, tempfile, json
from pathlib import Path
from PySide6.QtCore import Qt, QRect, QTimer, QThread, Signal, QBuffer, QIODevice
from PySide6.QtGui import QPainter, QColor, QKeyEvent
from PySide6.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPlainTextEdit, QPushButton

# Load .env file to ensure GEMINI_API_KEY is available
try:
    from dotenv import load_dotenv
    env_path = Path(__file__).parent / '.env'
    load_dotenv(env_path)
    print(f"[DEBUG] Loaded .env from {env_path}")
    print(f"[DEBUG] GEMINI_API_KEY present: {bool(os.getenv('GEMINI_API_KEY'))}")
except ImportError:
    print("[DEBUG] python-dotenv not available, setting API key directly")
    # Set the API key directly if dotenv loading fails
    os.environ['GEMINI_API_KEY'] = 'AIzaSyCSgGMMVyBaW6KUi5LyZ3JNZsRt0MwY2Z4'
    print(f"[DEBUG] GEMINI_API_KEY set directly: {bool(os.getenv('GEMINI_API_KEY'))}")

# Double-check the API key is available
if not os.getenv('GEMINI_API_KEY'):
    print("[DEBUG] GEMINI_API_KEY still not found, setting fallback")
    os.environ['GEMINI_API_KEY'] = 'AIzaSyCSgGMMVyBaW6KUi5LyZ3JNZsRt0MwY2Z4'

sys.path.insert(0, str(Path(__file__).parent / 'capture_demo'))
try:
    from live_api_client import next_move
except ImportError:
    next_move = None


# Removed sync_next_move - using NavigationWorker QThread instead


class NavigationWorker(QThread):
    move_suggested = Signal(str, int, str)  # direction, velocity, full_response
    analysis_error = Signal(str)
    
    def __init__(self, pixmap_bytes):
        super().__init__()
        self.pixmap_bytes = pixmap_bytes
        self._should_stop = False
    
    def stop(self):
        """Request the worker to stop"""
        self._should_stop = True
        
    def run(self):
        try:
            if self._should_stop:
                return
                
            print(f"[DEBUG] NavigationWorker starting, next_move is: {next_move}")
            if next_move is None:
                self.analysis_error.emit("Cloud response not available. Check capture_demo setup.")
                return
            
            if self._should_stop:
                return
            
            # Use subprocess to completely isolate async operations from Qt threading
            import subprocess
            import tempfile
            import json
            
            # Save image data to temporary file
            with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as temp_file:
                temp_file.write(self.pixmap_bytes)
                temp_path = temp_file.name
            
            try:
                # Create a separate script to run the async call
                script_content = f'''
import asyncio
import sys
import os
sys.path.insert(0, r"{Path(__file__).parent / 'capture_demo'}")
from live_api_client import next_move
import json

async def main():
    with open(r"{temp_path}", "rb") as f:
        image_bytes = f.read()
    
    try:
        direction, velocity, response = await next_move(image_bytes)
        result = {{"direction": direction, "velocity": velocity, "response": response}}
        print(json.dumps(result))
    except Exception as e:
        result = {{"direction": None, "velocity": 0, "response": f"Error: {{str(e)}}"}}
        print(json.dumps(result))

if __name__ == "__main__":
    asyncio.run(main())
'''
                
                # Write script to temporary file
                with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as script_file:
                    script_file.write(script_content)
                    script_path = script_file.name
                
                # Run the script in a separate process
                result = subprocess.run([sys.executable, script_path], 
                                      capture_output=True, text=True, timeout=30)
                
                if result.returncode == 0:
                    # Enhanced error handling with detailed logging
                    try:
                        # Clean and validate the response
                        raw_output = result.stdout.strip()
                        print(f"[DEBUG] Raw API output: {raw_output[:500]}...")  # Log first 500 chars
                        
                        # Try to parse JSON
                        response_data = json.loads(raw_output)
                        
                        # Validate required fields
                        if not isinstance(response_data, dict):
                            raise ValueError(f"Expected dict, got {type(response_data)}")
                        
                        direction = response_data.get("direction")
                        velocity = response_data.get("velocity", 400)  # Default to 400 instead of 50
                        full_response = response_data.get("response", "No response")
                        
                        # Validate direction
                        if direction not in ["Up", "Down", "Left", "Right", None]:
                            print(f"[DEBUG] Invalid direction: {direction}, setting to None")
                            direction = None
                        
                        # Validate velocity
                        if not isinstance(velocity, (int, float)) or velocity < 0:
                            print(f"[DEBUG] Invalid velocity: {velocity}, setting to 400")
                            velocity = 400
                        
                        if not self._should_stop:
                            self.move_suggested.emit(direction, velocity, full_response)
                            
                    except json.JSONDecodeError as e:
                        print(f"[DEBUG] JSON parsing failed: {e}")
                        print(f"[DEBUG] Raw output was: {result.stdout}")
                        print(f"[DEBUG] Stderr was: {result.stderr}")
                        self.analysis_error.emit("PARSE_ERROR")  # Special error code for consecutive failure tracking
                        
                    except (ValueError, KeyError, TypeError) as e:
                        print(f"[DEBUG] Response validation failed: {e}")
                        print(f"[DEBUG] Raw output was: {result.stdout}")
                        self.analysis_error.emit("VALIDATION_ERROR")
                        
                else:
                    print(f"[DEBUG] Subprocess failed with return code {result.returncode}")
                    print(f"[DEBUG] Stdout: {result.stdout}")
                    print(f"[DEBUG] Stderr: {result.stderr}")
                    
                    # Check for specific error patterns
                    stderr_lower = result.stderr.lower()
                    if "429" in stderr_lower or "quota" in stderr_lower or "resource_exhausted" in stderr_lower:
                        self.analysis_error.emit("RATE_LIMIT")
                    elif "timeout" in stderr_lower or "connection" in stderr_lower:
                        self.analysis_error.emit("CONNECTION_ERROR")
                    else:
                        self.analysis_error.emit("SUBPROCESS_ERROR")
                    
            finally:
                # Cleanup temporary files
                try:
                    os.unlink(temp_path)
                    os.unlink(script_path)
                except:
                    pass
                
        except Exception as e:
            if self._should_stop:
                return
                
            print(f"[DEBUG] Exception in NavigationWorker: {e}")
            # Handle rate limit errors specifically
            if "429" in str(e) or "RESOURCE_EXHAUSTED" in str(e):
                self.analysis_error.emit("RATE_LIMIT")
            elif "Event loop" in str(e):
                self.analysis_error.emit("EVENT_LOOP_ERROR")
            else:
                self.analysis_error.emit("GENERAL_ERROR")


class GameWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(400, 400)
        self.setFocusPolicy(Qt.StrongFocus)
        self.square_size = 30
        self.step_size = 10
        self.red_x = 10.0  # Use float for smooth movement
        self.red_y = 10.0
        
        # Create multiple blue obstacles with varying sizes
        self.blue_obstacles = self.generate_obstacles()
        
        # Vectorized movement properties
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.speed = 500.0  # pixels per second (10x faster)
        self.movement_timer = QTimer()
        self.movement_timer.timeout.connect(self.update_position)
        self.movement_timer.start(16)  # ~60 FPS for smooth movement
        
        # Add star target that moves randomly
        import random
        self.star_size = 20
        self.move_star()  # Place star in valid location
    
    def generate_obstacles(self):
        """Generate 5-10 blue obstacles with varying sizes, ensuring proper spacing"""
        import random
        obstacles = []
        min_distance = self.square_size * 2  # 60 pixels minimum distance
        max_attempts = 100
        
        num_obstacles = random.randint(10, 20)  # 2x as many
        
        for _ in range(num_obstacles):
            attempts = 0
            while attempts < max_attempts:
                # Random size for obstacle (30-75 pixels, 1.5x larger)
                width = random.randint(30, 75)
                height = random.randint(30, 75)
                
                # Random position
                x = random.randint(0, 400 - width)
                y = random.randint(0, 400 - height)
                
                # Check if this position conflicts with existing obstacles or player start
                valid = True
                
                # Don't place obstacle too close to player starting position
                if (abs(x - 10) < min_distance and abs(y - 10) < min_distance):
                    valid = False
                
                # Check distance from existing obstacles
                for existing in obstacles:
                    ex, ey, ew, eh = existing
                    # Calculate minimum distance between rectangles
                    dx = max(0, max(x - (ex + ew), ex - (x + width)))
                    dy = max(0, max(y - (ey + eh), ey - (y + height)))
                    distance = (dx * dx + dy * dy) ** 0.5
                    
                    if distance < min_distance:
                        valid = False
                        break
                
                if valid:
                    obstacles.append((x, y, width, height))
                    break
                
                attempts += 1
        
        return obstacles
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(255, 255, 255))
        
        # Draw all blue obstacles
        for obstacle in self.blue_obstacles:
            x, y, width, height = obstacle
            painter.fillRect(QRect(x, y, width, height), QColor(0, 0, 255))
        
        # Draw red player (convert float positions to int for drawing)
        painter.fillRect(QRect(int(self.red_x), int(self.red_y), self.square_size, self.square_size), QColor(255, 0, 0))
        
        # Draw yellow star target
        painter.fillRect(QRect(self.star_x, self.star_y, self.star_size, self.star_size), QColor(255, 255, 0))
        
        # Check if red square reached the star
        if (abs(self.red_x - self.star_x) < self.square_size and 
            abs(self.red_y - self.star_y) < self.square_size):
            self.move_star()
    
    def update_position(self):
        """Update position based on current velocity (smooth movement)"""
        # Calculate time delta (16ms = ~0.016 seconds for 60 FPS)
        dt = 0.016
        
        # Update position based on velocity
        self.red_x += self.velocity_x * dt
        self.red_y += self.velocity_y * dt
        
        # Keep within bounds
        self.red_x = max(0, min(self.red_x, 400 - self.square_size))
        self.red_y = max(0, min(self.red_y, 400 - self.square_size))
        
        # Apply friction to gradually stop movement
        friction = 0.85
        self.velocity_x *= friction
        self.velocity_y *= friction
        
        # Stop very slow movement to prevent jitter
        if abs(self.velocity_x) < 1.0:
            self.velocity_x = 0.0
        if abs(self.velocity_y) < 1.0:
            self.velocity_y = 0.0
        
        self.update()
        
    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        # Set velocity in the appropriate direction for smooth movement
        if key == Qt.Key_W or key == Qt.Key_Up:
            self.velocity_y = -self.speed
        elif key == Qt.Key_S or key == Qt.Key_Down:
            self.velocity_y = self.speed
        elif key == Qt.Key_A or key == Qt.Key_Left:
            self.velocity_x = -self.speed
        elif key == Qt.Key_D or key == Qt.Key_Right:
            self.velocity_x = self.speed
        else:
            super().keyPressEvent(event)
            return
    
    def move_star(self):
        """Move star to a new random location that doesn't overlap with obstacles"""
        import random
        max_attempts = 50
        
        for _ in range(max_attempts):
            self.star_x = random.randint(0, 400 - self.star_size)
            self.star_y = random.randint(0, 400 - self.star_size)
            
            # Check if star overlaps with any obstacle
            star_valid = True
            for obstacle in self.blue_obstacles:
                ox, oy, ow, oh = obstacle
                # Check if star rectangle overlaps with obstacle rectangle
                if (self.star_x < ox + ow and self.star_x + self.star_size > ox and
                    self.star_y < oy + oh and self.star_y + self.star_size > oy):
                    star_valid = False
                    break
            
            # Also make sure star isn't too close to player
            if (abs(self.star_x - self.red_x) < self.square_size and 
                abs(self.star_y - self.red_y) < self.square_size):
                star_valid = False
            
            if star_valid:
                break
    
    def auto_move(self, direction, velocity=None):
        """Set velocity based on AI direction and specified velocity for pool-like movement"""
        if velocity is None:
            velocity = self.speed  # Use default speed if no velocity specified
            
        if direction == "Up":
            self.velocity_y = -velocity
        elif direction == "Down":
            self.velocity_y = velocity
        elif direction == "Left":
            self.velocity_x = -velocity
        elif direction == "Right":
            self.velocity_x = velocity


class Main(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GameWidget with Cloud Navigation")
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
        self.reply_box.setPlaceholderText("Navigate the red square to the yellow star! Press SPACE for cloud guidance, ENTER for auto-move...")
        
        right_layout.addWidget(self.preview_label)
        right_layout.addWidget(self.reply_box)
        
        # AI Control Button
        self.ai_control_button = QPushButton("Start AI")
        self.ai_control_button.clicked.connect(self.toggle_ai_control)
        right_layout.addWidget(self.ai_control_button)
        
        # Mock AI Button (for testing when API quota is exceeded)
        self.mock_ai_button = QPushButton("Start Mock AI")
        self.mock_ai_button.clicked.connect(self.toggle_mock_ai)
        right_layout.addWidget(self.mock_ai_button)
        
        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        
        main_layout.addWidget(self.game_widget)
        main_layout.addWidget(right_widget)
        self.setLayout(main_layout)
        
        self.latest_pix = None
        self.latest_direction = None
        self.latest_velocity = 400  # Default velocity for manual suggestions
        self.ai_running = False
        self.mock_ai_running = False
        self.request_in_flight = False
        self.last_ai_request = 0  # Rate limiting
        self.consecutive_failures = 0  # Track consecutive API failures for auto-fallback
        self.max_consecutive_failures = 3  # Switch to Mock AI after 3 consecutive failures
        
        # Enhanced AI state tracking
        self.previous_direction = None
        self.previous_velocity = 400
        self.previous_red_x = 10.0
        self.previous_red_y = 10.0
        self.movement_history = []  # Track last few moves for momentum analysis
        self.stuck_counter = 0  # Track how many times we've been stuck in same area
        self.last_stuck_position = None  # Remember where we got stuck
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_preview)
        self.timer.start(100)
        
        self.nav_worker = None
        self.ai_worker = None
        self.game_widget.setFocus()
    
    def update_preview(self):
        self.latest_pix = self.game_widget.grab()
        self.preview_label.setPixmap(self.latest_pix)
        
        # AI Control: trigger AI request if enabled and no request in flight
        # Rate limit: max 1 request every 2 seconds to avoid overwhelming the API and ensure proper cleanup
        import time
        current_time = time.time()
        
        # Handle real AI requests
        if (self.ai_running and not self.request_in_flight and 
            self.latest_pix is not None and 
            current_time - self.last_ai_request >= 2):
            
            self.request_in_flight = True
            self.last_ai_request = current_time
            pixmap_bytes = self.pixmap_to_bytes(self.latest_pix)
            
            # Stop any existing AI worker to prevent conflicts
            if self.ai_worker and self.ai_worker.isRunning():
                self.ai_worker.stop()
                self.ai_worker.quit()
                self.ai_worker.wait()
                # Small delay to ensure complete cleanup
                time.sleep(0.1)
            
            # Create new AI worker with movement context
            self.ai_worker = NavigationWorker(pixmap_bytes)
            self.ai_worker.move_suggested.connect(self.on_ai_move_complete)
            self.ai_worker.analysis_error.connect(self.on_ai_error)
            self.ai_worker.start()
            
        # Handle mock AI requests (for testing when API quota is exceeded)
        elif (self.mock_ai_running and not self.request_in_flight and 
              self.latest_pix is not None and 
              current_time - self.last_ai_request >= 1):  # Faster for mock
            
            self.request_in_flight = True
            self.last_ai_request = current_time
            
            # Generate mock AI response
            direction, velocity, reasoning = self.generate_mock_ai_response()
            self.on_ai_move_complete(direction, velocity, reasoning)
    
    def toggle_ai_control(self):
        """Toggle AI control on/off"""
        # Disable mock AI if it's running
        if self.mock_ai_running:
            self.mock_ai_running = False
            self.mock_ai_button.setText("Start Mock AI")
            
        self.ai_running = not self.ai_running
        if self.ai_running:
            self.ai_control_button.setText("Stop AI")
            self.consecutive_failures = 0  # Reset failure counter when starting AI
            self.stuck_counter = 0  # Reset stuck counter
            self.last_stuck_position = None  # Reset stuck position
            self.reply_box.appendPlainText("Real AI Control: ENABLED - Navigate to the yellow star!")
        else:
            self.ai_control_button.setText("Start AI")
            self.reply_box.appendPlainText("Real AI Control: DISABLED")
            self.request_in_flight = False  # Reset request flag when stopping
            self.consecutive_failures = 0  # Reset failure counter when stopping
            self.stuck_counter = 0  # Reset stuck counter
            self.last_stuck_position = None  # Reset stuck position
            # Stop any running AI worker
            if self.ai_worker and self.ai_worker.isRunning():
                self.ai_worker.stop()
                self.ai_worker.quit()
                self.ai_worker.wait()
    
    def toggle_mock_ai(self):
        """Toggle mock AI control on/off (for testing when API quota exceeded)"""
        # Disable real AI if it's running
        if self.ai_running:
            self.ai_running = False
            self.ai_control_button.setText("Start AI")
            if self.ai_worker and self.ai_worker.isRunning():
                self.ai_worker.stop()
                self.ai_worker.quit()
                self.ai_worker.wait()
                
        self.mock_ai_running = not self.mock_ai_running
        if self.mock_ai_running:
            self.mock_ai_button.setText("Stop Mock AI")
            self.stuck_counter = 0  # Reset stuck counter
            self.last_stuck_position = None  # Reset stuck position
            self.reply_box.appendPlainText("Mock AI Control: ENABLED - Simulated navigation (no API calls)")
        else:
            self.mock_ai_button.setText("Start Mock AI")
            self.reply_box.appendPlainText("Mock AI Control: DISABLED")
            self.request_in_flight = False
            self.stuck_counter = 0  # Reset stuck counter
            self.last_stuck_position = None  # Reset stuck position
    
    def generate_mock_ai_response(self):
        """Generate highly intelligent AI response with enhanced obstacle avoidance and velocity calibration"""
        import math, random
        
        # Get current positions
        red_x, red_y = self.game_widget.red_x, self.game_widget.red_y
        star_x, star_y = self.game_widget.star_x, self.game_widget.star_y
        obstacles = self.game_widget.blue_obstacles
        
        # PRIORITY 1: Check if we need to backtrack due to collision or being stuck
        if self.should_backtrack(red_x, red_y):
            if self.previous_direction and self.previous_velocity:
                # Simple backtrack: exact same motion but opposite direction
                backtrack_direction = self.get_opposite_direction(self.previous_direction)
                backtrack_velocity = self.previous_velocity  # Use EXACT same velocity as last step
                
                collision_reason = ""
                if self.is_colliding_with_obstacles(red_x, red_y):
                    collision_reason = "COLLISION DETECTED with blue obstacle"
                elif self.is_stuck_at_boundary(red_x, red_y):
                    collision_reason = "BOUNDARY COLLISION at frame edge"
                else:
                    collision_reason = f"STUCK in same area for {self.stuck_counter} moves"
                
                reasoning = f"BACKTRACK: {collision_reason}. Reversing last move: {self.previous_direction}@{self.previous_velocity} -> {backtrack_direction}@{backtrack_velocity} (exact opposite motion)."
                
                # Update tracking
                self.previous_direction = backtrack_direction
                self.previous_velocity = backtrack_velocity
                self.previous_red_x = red_x
                self.previous_red_y = red_y
                
                return backtrack_direction, backtrack_velocity, reasoning
            else:
                # No previous move to backtrack, use emergency escape
                # Find direction away from obstacles or boundaries
                escape_direction = "Up"  # Default
                if red_x <= 15:  # Left boundary
                    escape_direction = "Right"
                elif red_x >= 365:  # Right boundary
                    escape_direction = "Left"
                elif red_y <= 15:  # Top boundary
                    escape_direction = "Down"
                elif red_y >= 365:  # Bottom boundary
                    escape_direction = "Up"
                
                reasoning = f"EMERGENCY ESCAPE: No previous move to backtrack, using {escape_direction} to escape boundary/collision."
                return escape_direction, 500, reasoning
        
        # Track movement for momentum analysis
        movement_delta_x = red_x - self.previous_red_x
        movement_delta_y = red_y - self.previous_red_y
        movement_speed = math.sqrt(movement_delta_x**2 + movement_delta_y**2)
        
        # Update movement history (keep last 5 moves)
        self.movement_history.append((self.previous_direction, movement_speed))
        if len(self.movement_history) > 5:
            self.movement_history.pop(0)
        
        # Calculate vector to target
        to_star_x = star_x - red_x
        to_star_y = star_y - red_y
        dist_to_star = math.sqrt(to_star_x**2 + to_star_y**2)
        
        # Advanced obstacle analysis - calculate threat levels for all obstacles
        obstacle_threats = []
        for obstacle in obstacles:
            ox, oy, ow, oh = obstacle
            
            # Calculate distance to closest point on rectangle
            closest_x = max(ox, min(red_x, ox + ow))
            closest_y = max(oy, min(red_y, oy + oh))
            dist = math.sqrt((red_x - closest_x)**2 + (red_y - closest_y)**2)
            
            # Calculate threat level based on distance and size
            threat_level = max(0, 150 - dist) * (ow + oh) / 100  # Bigger obstacles = more threatening
            
            obstacle_threats.append({
                'obstacle': obstacle,
                'distance': dist,
                'threat_level': threat_level,
                'center_x': ox + ow/2,
                'center_y': oy + oh/2
            })
        
        # Sort by threat level (highest threat first)
        obstacle_threats.sort(key=lambda x: x['threat_level'], reverse=True)
        
        # Analyze directional safety with fine-grained collision detection
        direction_analysis = {}
        test_distances = [30, 60, 90]  # Test at multiple distances ahead
        
        for test_dir in ["Up", "Down", "Left", "Right"]:
            safety_score = 100  # Start with perfect safety
            clear_distance = 200  # How far we can see ahead
            
            for test_dist in test_distances:
                test_x, test_y = red_x, red_y
                
                if test_dir == "Up":
                    test_y -= test_dist
                elif test_dir == "Down":
                    test_y += test_dist
                elif test_dir == "Left":
                    test_x -= test_dist
                elif test_dir == "Right":
                    test_x += test_dist
                
                # Check boundaries
                if (test_x < 15 or test_x > 370 or test_y < 15 or test_y > 370):
                    safety_score -= 50  # Heavy penalty for approaching walls
                    clear_distance = min(clear_distance, test_dist)
                
                # Check all obstacles
                for threat in obstacle_threats:
                    ox, oy, ow, oh = threat['obstacle']
                    
                    # Use expanded collision detection with safety margins
                    margin = 25  # Safety margin around obstacles
                    if (test_x < ox + ow + margin and test_x + self.game_widget.square_size > ox - margin and
                        test_y < oy + oh + margin and test_y + self.game_widget.square_size > oy - margin):
                        
                        # Heavy penalty for collision path
                        safety_score -= 40
                        clear_distance = min(clear_distance, test_dist)
                        
                        # Additional penalty based on obstacle size and threat level
                        safety_score -= threat['threat_level'] * 0.5
            
            # Bonus for moving toward star
            if test_dir == "Up" and to_star_y < 0:
                safety_score += 15
            elif test_dir == "Down" and to_star_y > 0:
                safety_score += 15
            elif test_dir == "Left" and to_star_x < 0:
                safety_score += 15
            elif test_dir == "Right" and to_star_x > 0:
                safety_score += 15
            
            # Penalty for oscillating (changing direction too often)
            if (len(self.movement_history) >= 2 and 
                self.previous_direction and 
                self.get_opposite_direction(test_dir) == self.previous_direction):
                safety_score -= 10
            
            direction_analysis[test_dir] = {
                'safety_score': safety_score,
                'clear_distance': clear_distance
            }
        
        # Select best direction based on safety analysis
        best_direction = max(direction_analysis.keys(), key=lambda d: direction_analysis[d]['safety_score'])
        best_analysis = direction_analysis[best_direction]
        
        # Advanced velocity calibration based on multiple factors
        base_velocity = 400
        
        # Factor 1: Distance to closest obstacle (more conservative in medium range)
        if obstacle_threats:
            closest_dist = obstacle_threats[0]['distance']
            if closest_dist < 30:
                velocity_factor = 0.4  # Very slow when very close
            elif closest_dist < 60:
                velocity_factor = 0.6  # Slow when close
            elif closest_dist < 100:
                velocity_factor = 0.7  # Conservative when moderate distance
            elif closest_dist < 140:
                velocity_factor = 0.9  # Careful when medium distance
            else:
                velocity_factor = 1.3  # Fast when far
        else:
            velocity_factor = 1.3  # Fast when no obstacles
        
        # Factor 2: Path clearance ahead
        clearance_factor = min(best_analysis['clear_distance'] / 100, 1.5)
        
        # Factor 3: Distance to star (slow down when approaching)
        if dist_to_star < 50:
            target_factor = 0.6  # Slow approach
        elif dist_to_star < 100:
            target_factor = 0.8  # Moderate approach
        else:
            target_factor = 1.2  # Normal speed
        
        # Factor 4: Previous movement momentum (maintain smooth movement)
        if self.previous_direction == best_direction and movement_speed > 0:
            momentum_factor = 1.1  # Slight boost for continuing same direction
        else:
            momentum_factor = 1.0
        
        # Calculate final velocity
        velocity = int(base_velocity * velocity_factor * clearance_factor * target_factor * momentum_factor)
        velocity = max(200, min(900, velocity))  # Clamp to effective range
        
        # Generate intelligent reasoning
        safety_level = "High" if best_analysis['safety_score'] > 80 else "Medium" if best_analysis['safety_score'] > 40 else "Low"
        
        if obstacle_threats and obstacle_threats[0]['threat_level'] > 50:
            reasoning = f"OBSTACLE AVOIDANCE: Closest blue obstacle at {obstacle_threats[0]['distance']:.0f} pixels. Safety level: {safety_level}. Choosing {best_direction} direction with carefully calibrated velocity {velocity} to avoid collision while progressing toward star."
        elif dist_to_star < 80:
            reasoning = f"PRECISION APPROACH: Yellow star {dist_to_star:.0f} pixels away. Using {best_direction} direction with controlled velocity {velocity} for accurate targeting."
        else:
            reasoning = f"NAVIGATION: Clear path ahead. Moving {best_direction} at velocity {velocity} toward star. Safety score: {best_analysis['safety_score']:.0f}/100."
        
        # Update tracking variables
        self.previous_direction = best_direction
        self.previous_velocity = velocity
        self.previous_red_x = red_x
        self.previous_red_y = red_y
        
        full_reasoning = f"{reasoning}\nDirection: {best_direction}\nVelocity: {velocity}"
        
        return best_direction, velocity, full_reasoning
    
    def get_opposite_direction(self, direction):
        """Get the opposite direction to detect oscillation"""
        opposites = {"Up": "Down", "Down": "Up", "Left": "Right", "Right": "Left"}
        return opposites.get(direction)
    
    def is_colliding_with_obstacles(self, x, y):
        """Check if position overlaps with any blue obstacle (with small margin for early detection)"""
        margin = 5  # Small margin for early collision detection
        player_rect = (x - margin, y - margin, x + self.game_widget.square_size + margin, y + self.game_widget.square_size + margin)
        
        for obstacle in self.game_widget.blue_obstacles:
            ox, oy, ow, oh = obstacle
            obstacle_rect = (ox, oy, ox + ow, oy + oh)
            
            # Check rectangle overlap
            if (player_rect[0] < obstacle_rect[2] and player_rect[2] > obstacle_rect[0] and
                player_rect[1] < obstacle_rect[3] and player_rect[3] > obstacle_rect[1]):
                return True
        return False
    
    def is_stuck_at_boundary(self, x, y):
        """Check if position is too close to frame boundaries"""
        margin = 15  # Increased margin for earlier boundary detection
        return (x <= margin or x >= 400 - self.game_widget.square_size - margin or
                y <= margin or y >= 400 - self.game_widget.square_size - margin)
    
    def should_backtrack(self, red_x, red_y):
        """Determine if we should backtrack instead of finding new direction"""
        # Check if currently colliding with obstacles
        if self.is_colliding_with_obstacles(red_x, red_y):
            return True
        
        # Check if stuck at frame boundary
        if self.is_stuck_at_boundary(red_x, red_y):
            return True
        
        # Check if we've been in the same area for too long
        current_pos = (int(red_x / 20), int(red_y / 20))  # Grid-based position
        if self.last_stuck_position == current_pos:
            self.stuck_counter += 1
            if self.stuck_counter >= 3:  # Been stuck for 3 moves
                return True
        else:
            self.stuck_counter = 0
            self.last_stuck_position = current_pos
        
        return False
    
    def on_ai_move_complete(self, direction, velocity, full_response):
        """Handle AI move completion with full LLM response including velocity"""
        self.request_in_flight = False
        self.consecutive_failures = 0  # Reset failure counter on successful response
        
        if self.ai_running or self.mock_ai_running:  # Move if either AI mode is running
            # Display the full LLM response
            self.reply_box.appendPlainText(f"Response: {full_response}")
            
            if direction and direction in ["Up", "Down", "Left", "Right"]:
                self.reply_box.appendPlainText(f"Moving: {direction} at velocity {velocity}")
                self.reply_box.appendPlainText("-" * 40)
                self.reply_box.ensureCursorVisible()
                
                # Update movement tracking (for both real and mock AI)
                if not self.mock_ai_running:  # Only update for real AI - mock AI updates itself
                    self.previous_direction = direction
                    self.previous_velocity = velocity
                    self.previous_red_x = self.game_widget.red_x
                    self.previous_red_y = self.game_widget.red_y
                
                # Execute the move with AI-specified velocity
                self.execute_ai_move(direction, velocity)
            else:
                # If real AI fails, immediately try mock AI for this step
                if self.ai_running and not self.mock_ai_running:
                    self.reply_box.appendPlainText("Invalid direction from real AI, trying Mock AI...")
                    direction, velocity, reasoning = self.generate_mock_ai_response()
                    if direction:
                        self.reply_box.appendPlainText(f"Mock AI: Moving {direction} at velocity {velocity}")
                        self.reply_box.appendPlainText("-" * 40)
                        self.reply_box.ensureCursorVisible()
                        self.execute_ai_move(direction, velocity)
                    else:
                        self.reply_box.appendPlainText("No valid move available, staying stationary")
                        self.reply_box.appendPlainText("-" * 40)
                        self.reply_box.ensureCursorVisible()
                else:
                    self.reply_box.appendPlainText("No valid move available, staying stationary")
                    self.reply_box.appendPlainText("-" * 40)
                    self.reply_box.ensureCursorVisible()
    
    def on_ai_error(self, error):
        """Handle AI error with enhanced error handling and auto-fallback"""
        self.request_in_flight = False
        self.consecutive_failures += 1
        
        if self.ai_running:
            # Create user-friendly error messages
            error_messages = {
                "PARSE_ERROR": "Failed to parse API response",
                "VALIDATION_ERROR": "Invalid API response format",
                "RATE_LIMIT": "Rate limit exceeded",
                "CONNECTION_ERROR": "Network connection issue",
                "SUBPROCESS_ERROR": "API subprocess failed",
                "EVENT_LOOP_ERROR": "Threading issue detected",
                "GENERAL_ERROR": "API error occurred"
            }
            
            user_message = error_messages.get(error, f"Unknown error: {error}")
            self.reply_box.appendPlainText(f"Error: {user_message}")
            
            # Auto-switch to Mock AI after consecutive failures
            if self.consecutive_failures >= self.max_consecutive_failures:
                self.reply_box.appendPlainText(f"⚠️ {self.consecutive_failures} consecutive failures detected!")
                self.reply_box.appendPlainText("Auto-switching to Mock AI for continued navigation...")
                self.reply_box.appendPlainText("-" * 40)
                
                # Switch to mock AI automatically
                self.ai_running = False
                self.ai_control_button.setText("Start AI")
                self.mock_ai_running = True
                self.mock_ai_button.setText("Stop Mock AI")
                self.consecutive_failures = 0  # Reset counter
                
                # Immediately generate a mock move so no step is wasted
                direction, velocity, reasoning = self.generate_mock_ai_response()
                self.on_ai_move_complete(direction, velocity, f"Mock AI (Auto-fallback): {reasoning}")
                
            elif error in ["RATE_LIMIT", "EVENT_LOOP_ERROR"]:
                # Immediate switch for critical errors
                self.reply_box.appendPlainText("Critical error detected - switching to Mock AI...")
                self.reply_box.appendPlainText("-" * 40)
                
                # Switch to mock AI automatically
                self.ai_running = False
                self.ai_control_button.setText("Start AI")
                self.mock_ai_running = True
                self.mock_ai_button.setText("Stop Mock AI")
                self.consecutive_failures = 0  # Reset counter
                
                # Immediately generate a mock move so no step is wasted
                direction, velocity, reasoning = self.generate_mock_ai_response()
                self.on_ai_move_complete(direction, velocity, f"Mock AI (Critical fallback): {reasoning}")
                
            else:
                self.reply_box.appendPlainText(f"Staying stationary until next valid response... (Failure {self.consecutive_failures}/{self.max_consecutive_failures})")
                self.reply_box.appendPlainText("-" * 40)
    
    def execute_ai_move(self, direction, velocity=None):
        """Execute AI move using the auto_move method with specified velocity"""
        if direction in ["Up", "Down", "Left", "Right"]:
            self.game_widget.auto_move(direction, velocity)
    
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
            self.reply_box.appendPlainText("Cloud analysis already in progress...")
            return
        pixmap_bytes = self.pixmap_to_bytes(self.latest_pix)
        self.nav_worker = NavigationWorker(pixmap_bytes)
        self.nav_worker.move_suggested.connect(self.on_move_suggested)
        self.nav_worker.analysis_error.connect(self.on_navigation_error)
        self.nav_worker.start()
        self.reply_box.appendPlainText("Consulting cloud response...")
    
    def on_move_suggested(self, direction, velocity, full_response):
        self.latest_direction = direction if direction in ["Up", "Down", "Left", "Right"] else None
        self.latest_velocity = velocity if self.latest_direction else 0
        self.reply_box.appendPlainText(f"\nCloud reasoning: {full_response}")
        
        if self.latest_direction:
            self.reply_box.appendPlainText(f"Suggested move: {direction} at velocity {velocity}")
            self.reply_box.appendPlainText("Press ENTER to auto-move or continue manually with WASD")
        else:
            self.reply_box.appendPlainText("No valid direction suggested, staying stationary")
            self.reply_box.appendPlainText("Continue manually with WASD")
            
        self.reply_box.appendPlainText("="*50)
        self.reply_box.ensureCursorVisible()
    
    def on_navigation_error(self, error):
        self.reply_box.appendPlainText(f"\nCloud response error: {error}\n" + "="*50)
        self.reply_box.ensureCursorVisible()
    
    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key_Space:
            self.get_navigation_guidance()
        elif event.key() == Qt.Key_Return and self.latest_direction:
            self.game_widget.auto_move(self.latest_direction, self.latest_velocity)
            self.reply_box.appendPlainText(f"Auto-moved: {self.latest_direction} at velocity {self.latest_velocity}")
            self.latest_direction = None
            self.latest_velocity = 400
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

Controls: WASD/arrows=Manual, SPACE=Cloud guidance, ENTER=Auto-move

Enhanced Error Handling (Forever Fix):
- Detailed debug logging for all API responses
- Response validation before parsing
- Auto-fallback to Mock AI after 3 consecutive failures
- Immediate switch to Mock AI for critical errors (rate limits, threading issues)
- Comprehensive error categorization and user-friendly messages
- Consecutive failure tracking with visual indicators
- Bulletproof JSON parsing with fallback mechanisms

Anti-Stuck Backtracking System:
- Collision detection with blue obstacles
- Boundary detection at frame edges
- Stuck position tracking with grid-based analysis
- Simple backtracking: exact same motion but opposite direction
- Uses identical velocity from previous move for precise reversal
- Emergency escape when no previous move exists
- More conservative velocity calibration in medium range (60-140 pixels)
- Automatic stuck counter reset when AI is toggled
""" 