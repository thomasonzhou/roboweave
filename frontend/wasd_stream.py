"""A PySide6 game widget with movable red square, live preview, and cloud navigation with W&B Weave observability."""

import asyncio, sys, os, subprocess, tempfile, json
from pathlib import Path
from PySide6.QtCore import Qt, QRect, QTimer, QThread, Signal, QBuffer, QIODevice
from PySide6.QtGui import QPainter, QColor, QKeyEvent
from PySide6.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPlainTextEdit, QPushButton
import weave

# Helper function for conditional Weave decorators
def weave_op():
    """Conditional weave.op decorator - only applies if Weave is enabled"""
    def decorator(func):
        return weave.op()(func) if 'WEAVE_ENABLED' in globals() and WEAVE_ENABLED else func
    return decorator

# Load .env file to ensure GEMINI_API_KEY is available
try:
    from dotenv import load_dotenv
    env_path = Path(__file__).parent / '.env'
    load_dotenv(env_path)
    print(f"[DEBUG] Loaded .env from {env_path}")
    print(f"[DEBUG] GEMINI_API_KEY present: {bool(os.getenv('GEMINI_API_KEY'))}")
except ImportError:
    print("[DEBUG] python-dotenv not available, please install with: pip install python-dotenv")

# Initialize W&B Weave for LLM observability (optional)
try:
    weave.init('gemini-navigation-game')
    print("[DEBUG] W&B Weave initialized successfully - full observability enabled")
    WEAVE_ENABLED = True
except Exception as e:
    print(f"[DEBUG] W&B Weave not available: {e}")
    print("[DEBUG] Running without observability - core AI functionality still available")
    WEAVE_ENABLED = False

# Check if API key is available
if not os.getenv('GEMINI_API_KEY'):
    print("[ERROR] GEMINI_API_KEY not found!")
    print("[ERROR] Please create a .env file in the frontend directory with:")
    print("[ERROR] GEMINI_API_KEY=your_api_key_here")
    print("[ERROR] Or set the environment variable directly.")
    # Don't hardcode the API key - force user to set it properly

sys.path.insert(0, str(Path(__file__).parent / 'capture_demo'))
try:
    from live_api_client import next_move
    print("[DEBUG] Successfully imported enhanced Gemini 1.5 Pro with W&B Weave tracking")
except ImportError as e:
    next_move = None
    print(f"[DEBUG] Failed to import live_api_client - Mock AI only. Error: {e}")


# Removed sync_next_move - using NavigationWorker QThread instead


class NavigationWorker(QThread):
    motion_primitives_ready = Signal(list, str)  # motion_primitives, overall_strategy
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
os.environ['SUBPROCESS_MODE'] = '1'  # Suppress debug output
sys.path.insert(0, r"{Path(__file__).parent / 'capture_demo'}")
from live_api_client import next_move
import json

async def main():
    with open(r"{temp_path}", "rb") as f:
        image_bytes = f.read()
    
    try:
        motion_primitives, strategy = await next_move(image_bytes)
        result = {{"motion_primitives": motion_primitives, "strategy": strategy}}
        print(json.dumps(result))
    except Exception as e:
        result = {{"motion_primitives": None, "strategy": f"Error: {{str(e)}}"}}
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
                        
                        # Remove any debug output contamination from the start
                        json_start = raw_output.find('{"motion_primitives":')
                        if json_start == -1:
                            json_start = raw_output.find('{"')
                        
                        if json_start > 0:
                            raw_output = raw_output[json_start:]
                        
                        # Find the end of the JSON object (matching braces)
                        if raw_output.startswith('{'):
                            brace_count = 0
                            json_end = 0
                            for i, char in enumerate(raw_output):
                                if char == '{':
                                    brace_count += 1
                                elif char == '}':
                                    brace_count -= 1
                                    if brace_count == 0:
                                        json_end = i + 1
                                        break
                            if json_end > 0:
                                raw_output = raw_output[:json_end]
                        
                        print(f"[DEBUG] Cleaned JSON: {raw_output[:300]}...")  # Log first 300 chars
                        
                        # Try to parse JSON
                        response_data = json.loads(raw_output)
                        
                        # Validate required fields
                        if not isinstance(response_data, dict):
                            raise ValueError(f"Expected dict, got {type(response_data)}")
                        
                        motion_primitives = response_data.get("motion_primitives")
                        strategy = response_data.get("strategy", "No strategy provided")
                        
                        # Handle case where motion primitives are embedded in strategy as JSON string
                        if motion_primitives is None and isinstance(strategy, str):
                            try:
                                # Check if strategy looks like JSON but might be truncated
                                print(f"[DEBUG] Strategy field length: {len(strategy)}")
                                print(f"[DEBUG] Strategy ends with: ...{strategy[-50:]}")
                                
                                # Try to parse strategy as JSON to extract motion primitives
                                print(f"[DEBUG] Attempting to parse strategy field as JSON")
                                
                                # If the JSON looks truncated, try to find complete motion primitives
                                if strategy.endswith('"control2') or '"control2' in strategy[-100:]:
                                    print(f"[DEBUG] Detected truncated JSON, attempting to extract complete primitives")
                                    
                                    # Find complete motion primitives before the truncation
                                    strategy_fixed = strategy
                                    if not strategy_fixed.endswith('}'):
                                        # Try to find the last complete primitive
                                        last_complete = strategy_fixed.rfind('}, {"type":')
                                        if last_complete > 0:
                                            strategy_fixed = strategy_fixed[:last_complete] + ']'
                                            if '"motion_primitives":' in strategy_fixed:
                                                strategy_fixed += ', "overall_strategy": "Extracted from truncated response"}'
                                    
                                    print(f"[DEBUG] Fixed strategy length: {len(strategy_fixed)}")
                                    strategy_data = json.loads(strategy_fixed)
                                else:
                                    strategy_data = json.loads(strategy)
                                    
                                print(f"[DEBUG] Strategy JSON parsed successfully, keys: {list(strategy_data.keys())}")
                                
                                # Handle both compact and full field names
                                if isinstance(strategy_data, dict):
                                    # Try compact field names first, then full names
                                    motion_primitives = strategy_data.get("m", strategy_data.get("motion_primitives"))
                                    if motion_primitives:
                                        strategy = strategy_data.get("st", strategy_data.get("overall_strategy", "Extracted from embedded JSON"))
                                        print(f"[DEBUG] Extracted {len(motion_primitives) if isinstance(motion_primitives, list) else 'invalid'} motion primitives from strategy field")
                                    else:
                                        print(f"[DEBUG] Strategy data doesn't contain motion_primitives ('m' or 'motion_primitives') key")
                                        print(f"[DEBUG] Available keys: {list(strategy_data.keys())}")
                                else:
                                    print(f"[DEBUG] Strategy data is not a dict: {type(strategy_data)}")
                            except json.JSONDecodeError as e:
                                print(f"[DEBUG] Strategy field JSON parsing failed: {e}")
                                print(f"[DEBUG] Strategy content: {strategy[:200]}...")
                                
                                # Last resort: try to extract primitives with regex (both compact and full names)
                                import re
                                primitives_match = re.search(r'"m":\s*(\[.*?\])', strategy)
                                if not primitives_match:
                                    primitives_match = re.search(r'"motion_primitives":\s*(\[.*?\])', strategy)
                                    
                                if primitives_match:
                                    try:
                                        motion_primitives = json.loads(primitives_match.group(1))
                                        print(f"[DEBUG] Extracted primitives with regex: {len(motion_primitives)} items")
                                    except:
                                        print(f"[DEBUG] Regex extraction also failed")
                        
                        # Debug: show what we extracted
                        print(f"[DEBUG] Final motion_primitives type: {type(motion_primitives)}")
                        if isinstance(motion_primitives, list):
                            print(f"[DEBUG] Motion primitives count: {len(motion_primitives)}")
                        elif motion_primitives is not None:
                            print(f"[DEBUG] Motion primitives value: {motion_primitives}")
                        
                        # Validate motion primitives
                        if motion_primitives is None:
                            print(f"[DEBUG] No motion primitives received")
                            motion_primitives = []
                        elif not isinstance(motion_primitives, list):
                            print(f"[DEBUG] Invalid motion primitives format: {type(motion_primitives)}")
                            motion_primitives = []
                        
                        if not self._should_stop:
                            self.motion_primitives_ready.emit(motion_primitives, strategy)
                            
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
        
        # Vectorized movement properties (for manual control)
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.speed = 500.0  # pixels per second (10x faster)
        
        # Bezier motion primitive system
        self.motion_primitives = []  # Queue of motion primitives
        self.current_primitive = None
        self.primitive_start_time = 0
        self.primitive_index = 0
        
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
    
    def bezier_point(self, t, p0, p1, p2, p3):
        """Calculate point on cubic bezier curve at parameter t (0-1)"""
        u = 1 - t
        tt = t * t
        uu = u * u
        uuu = uu * u
        ttt = tt * t
        
        x = uuu * p0[0] + 3 * uu * t * p1[0] + 3 * u * tt * p2[0] + ttt * p3[0]
        y = uuu * p0[1] + 3 * uu * t * p1[1] + 3 * u * tt * p2[1] + ttt * p3[1]
        
        return (x, y)
    
    def execute_motion_primitives(self, primitives_data):
        """Load new motion primitives with smooth transition from current motion"""
        import time
        current_time = time.time()
        
        # Stop manual movement when starting AI motion
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        
        # Convert primitives data to motion primitive queue
        new_primitives = []
        for primitive in primitives_data:
            motion_primitive = {
                'type': primitive.get('type', 'primary'),
                'start': primitive.get('start', [self.red_x, self.red_y]),
                'control1': primitive.get('control1', [self.red_x, self.red_y]),
                'control2': primitive.get('control2', [self.red_x, self.red_y]),
                'end': primitive.get('end', [self.red_x, self.red_y]),
                'duration': primitive.get('duration', 0.16),
                'velocity': primitive.get('velocity', 400),
                'reasoning': primitive.get('reasoning', '')
            }
            new_primitives.append(motion_primitive)
        
        # Handle smooth transition from current motion
        if self.current_primitive and self.motion_primitives:
            # Calculate remaining time in current primitive
            elapsed = current_time - self.primitive_start_time
            remaining_time = max(0, self.current_primitive['duration'] - elapsed)
            
            if remaining_time > 0.2:  # If more than 200ms left, create transition
                # Create a transition primitive from current position to new plan start
                if new_primitives:
                    transition_primitive = {
                        'type': 'transition',
                        'start': [self.red_x, self.red_y],
                        'control1': [self.red_x, self.red_y],
                        'control2': new_primitives[0]['start'],
                        'end': new_primitives[0]['start'],
                        'duration': min(0.3, remaining_time * 0.5),  # Slightly longer transition for slow motion
                        'velocity': 200,
                        'reasoning': 'Smooth transition to new plan'
                    }
                    
                    # Adjust first new primitive to start smoothly
                    new_primitives[0]['start'] = new_primitives[0]['start']
                    
                    # Replace current primitives with transition + new plan
                    self.motion_primitives = [transition_primitive] + new_primitives
                    self.current_primitive = transition_primitive
                    self.primitive_start_time = current_time
                    self.primitive_index = 0
                else:
                    # No new primitives, just stop current motion
                    self.motion_primitives = []
                    self.current_primitive = None
            else:
                # Current primitive almost finished, start new plan immediately
                self.motion_primitives = new_primitives
                if new_primitives:
                    self.current_primitive = new_primitives[0]
                    self.primitive_start_time = current_time
                    self.primitive_index = 0
                    self.current_primitive['start'] = [self.red_x, self.red_y]
        else:
            # No current motion, start new plan immediately
            self.motion_primitives = new_primitives
            if new_primitives:
                self.current_primitive = new_primitives[0]
                self.primitive_start_time = current_time
                self.primitive_index = 0
                self.current_primitive['start'] = [self.red_x, self.red_y]
    
    def update_position(self):
        """Update position based on bezier motion primitives or manual velocity"""
        import time
        current_time = time.time()
        
        # Execute bezier motion primitives if available
        if self.current_primitive and self.motion_primitives:
            primitive_elapsed = current_time - self.primitive_start_time
            t = primitive_elapsed / self.current_primitive['duration']
            
            if t <= 1.0:
                # Calculate bezier position
                pos = self.bezier_point(
                    t,
                    self.current_primitive['start'],
                    self.current_primitive['control1'],
                    self.current_primitive['control2'],
                    self.current_primitive['end']
                )
                self.red_x, self.red_y = pos
            else:
                # Move to next primitive
                self.primitive_index += 1
                if self.primitive_index < len(self.motion_primitives):
                    self.current_primitive = self.motion_primitives[self.primitive_index]
                    self.primitive_start_time = current_time
                    
                    # Adjust start position for seamless transition
                    self.current_primitive['start'] = [self.red_x, self.red_y]
                else:
                    # All primitives completed
                    self.current_primitive = None
                    self.motion_primitives = []
        else:
            # Manual control mode (keyboard input)
            dt = 0.016  # 16ms = ~0.016 seconds for 60 FPS
            
            # Update position based on velocity
            self.red_x += self.velocity_x * dt
            self.red_y += self.velocity_y * dt
            
            # Apply friction to gradually stop movement
            friction = 0.85
            self.velocity_x *= friction
            self.velocity_y *= friction
            
            # Stop very slow movement to prevent jitter
            if abs(self.velocity_x) < 1.0:
                self.velocity_x = 0.0
            if abs(self.velocity_y) < 1.0:
                self.velocity_y = 0.0
        
        # Keep within bounds
        self.red_x = max(0, min(self.red_x, 400 - self.square_size))
        self.red_y = max(0, min(self.red_y, 400 - self.square_size))
        
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
        

        
        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        
        main_layout.addWidget(self.game_widget)
        main_layout.addWidget(right_widget)
        self.setLayout(main_layout)
        
        self.latest_pix = None
        self.latest_direction = None
        self.latest_velocity = 400  # Default velocity for manual suggestions
        self.ai_running = False
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
        
        # Handle real AI requests - every 0.5 seconds for super fast AI thinking
        if (self.ai_running and not self.request_in_flight and 
            self.latest_pix is not None and 
            current_time - self.last_ai_request >= 0.5):
            
            self.request_in_flight = True
            self.last_ai_request = current_time
            pixmap_bytes = self.pixmap_to_bytes(self.latest_pix)
            
            # Debug: show fast AI timing
            print(f"[DEBUG] Fast AI request at {current_time:.2f}s (0.5s interval)")
            
            # Stop any existing AI worker to prevent conflicts
            if self.ai_worker and self.ai_worker.isRunning():
                self.ai_worker.stop()
                self.ai_worker.quit()
                self.ai_worker.wait()
                # Small delay to ensure complete cleanup
                time.sleep(0.1)
            
            # Create new AI worker with movement context
            self.ai_worker = NavigationWorker(pixmap_bytes)
            self.ai_worker.motion_primitives_ready.connect(self.on_motion_primitives_ready)
            self.ai_worker.analysis_error.connect(self.on_ai_error)
            self.ai_worker.start()
            

    
    def toggle_ai_control(self):
        """Toggle AI control on/off"""
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
    
    @weave_op()
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
    
    @weave_op()
    def on_motion_primitives_ready(self, motion_primitives, strategy):
        """Handle AI motion primitives completion with bezier curve planning"""
        self.request_in_flight = False
        self.consecutive_failures = 0  # Reset failure counter on successful response
        
        if self.ai_running:  # Execute motion if AI is running
            # Display the overall strategy
            self.reply_box.appendPlainText(f"Strategy: {strategy}")
            
            if motion_primitives and len(motion_primitives) > 0:
                # Display motion primitive summary
                num_primitives = len(motion_primitives)
                primary_count = sum(1 for p in motion_primitives if p.get('type') == 'primary')
                speculative_count = num_primitives - primary_count
                
                # Calculate total execution time
                total_time = sum(p.get('duration', 0.6) for p in motion_primitives)
                
                self.reply_box.appendPlainText(f"Executing {num_primitives} SLOW motion primitives ({total_time:.1f}s total):")
                self.reply_box.appendPlainText(f"  • {primary_count} primary paths")
                self.reply_box.appendPlainText(f"  • {speculative_count} speculative paths")
                
                # Show first few primitive details with movement distance
                for i, primitive in enumerate(motion_primitives[:3]):
                    reasoning = primitive.get('reasoning', 'No reasoning')[:30] + "..."
                    duration = primitive.get('duration', 0.16)
                    
                    # Calculate movement distance
                    start = primitive.get('start', [0, 0])
                    end = primitive.get('end', [0, 0])
                    distance = ((end[0] - start[0])**2 + (end[1] - start[1])**2)**0.5
                    
                    self.reply_box.appendPlainText(f"  {i+1}: {reasoning} ({duration:.2f}s, {distance:.0f}px)")
                    if distance > 50:  # Warn about large movements
                        self.reply_box.appendPlainText(f"      ⚠️ Large movement: {start} → {end}")
                
                self.reply_box.appendPlainText("-" * 40)
                self.reply_box.ensureCursorVisible()
                
                # Update movement tracking
                if motion_primitives:
                    first_primitive = motion_primitives[0]
                    self.previous_red_x = self.game_widget.red_x
                    self.previous_red_y = self.game_widget.red_y
                
                # Execute the motion primitives
                self.execute_motion_primitives(motion_primitives)
            else:
                self.reply_box.appendPlainText("No valid motion primitives from AI, staying stationary")
                self.reply_box.appendPlainText("-" * 40)
                self.reply_box.ensureCursorVisible()
    
    def on_ai_error(self, error):
        """Handle AI error with enhanced error handling"""
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
            
            # Auto-disable AI after too many consecutive failures
            if self.consecutive_failures >= self.max_consecutive_failures:
                self.reply_box.appendPlainText(f"⚠️ {self.consecutive_failures} consecutive failures detected!")
                self.reply_box.appendPlainText("Auto-disabling AI due to persistent errors.")
                self.reply_box.appendPlainText("Please check your API key and network connection.")
                self.reply_box.appendPlainText("-" * 40)
                
                # Disable AI automatically
                self.ai_running = False
                self.ai_control_button.setText("Start AI")
                self.consecutive_failures = 0  # Reset counter
                
            else:
                self.reply_box.appendPlainText(f"Staying stationary until next valid response... (Failure {self.consecutive_failures}/{self.max_consecutive_failures})")
                self.reply_box.appendPlainText("-" * 40)
    
    def execute_motion_primitives(self, motion_primitives):
        """Execute AI motion primitives using bezier curves"""
        if motion_primitives:
            self.game_widget.execute_motion_primitives(motion_primitives)
    
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
pip install PySide6 google-genai weave pydantic
export GEMINI_API_KEY="YOUR-KEY"
python wasd_stream.py

Controls: WASD/arrows=Manual, SPACE=Cloud guidance, ENTER=Auto-move

W&B Weave Integration (Complete LLM Observability):
- Real-time tracking of all AI function calls and responses
- Gemini 1.5 Pro with maximum token support (1M+ tokens)
- Structured JSON output with Pydantic validation
- Complete trace visualization in W&B dashboard
- Debug AI reasoning and decision-making process
- Performance metrics and evaluation capabilities
- Access dashboard: https://wandb.ai/your-username/gemini-navigation-game

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

AI Model Features:
- Gemini 1.5 Pro: 1,048,576 token context window
- Structured JSON output for reliable parsing
- Enhanced obstacle avoidance prompts
- Velocity calibration: 200-900 range with conservative medium speeds
- Real-time observability with W&B Weave tracking
""" 