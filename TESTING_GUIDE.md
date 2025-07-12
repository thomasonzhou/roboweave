# Complete Testing Guide for RoboWeave Setup

This guide will help you test all components of the RoboWeave setup and identify what's missing.

## Quick Start Testing

### Step 1: Run Quick Test
```bash
python quick_test.py
```

This will quickly identify the most common issues:
- ✅ **Green checkmarks** = Working correctly
- ❌ **Red X marks** = Needs to be fixed
- ⏱️ **Clock** = Timed out (usually a dependency issue)

### Step 2: Run Full Test
```bash
python test_setup.py
```

This runs comprehensive tests on all components.

## Component-by-Component Testing

### 1. Environment Setup

**Test Python Version:**
```bash
python3 --version
# Should show Python 3.9 or higher
```

**Test Basic Tools:**
```bash
pip3 --version    # Package installer
npm --version     # Node.js (for Live-API console)
git --version     # Version control
```

**❌ If any fail:** Install the missing tools first.

### 2. API Key Setup

**Test API Key:**
```bash
echo $GEMINI_API_KEY
# Should show your API key (not empty)
```

**❌ If empty:**
1. Get API key from https://makersuite.google.com/app/apikey
2. Set it: `export GEMINI_API_KEY='your-api-key-here'`
3. Add to shell profile: `echo 'export GEMINI_API_KEY="your-key"' >> ~/.bashrc`

### 3. Python Dependencies

**Test PySide6:**
```bash
python3 -c "import PySide6; print('PySide6 OK')"
```

**Test Google AI:**
```bash
python3 -c "import google.generativeai; print('Google AI OK')"
```

**Test PIL/Pillow:**
```bash
python3 -c "import PIL; print('PIL OK')"
```

**❌ If any fail:**
```bash
pip install PySide6>=6.5.0 google-generativeai>=0.5.0 pillow>=10.0.0
```

### 4. Live API Client

**Test Import:**
```bash
cd frontend/capture_demo
python3 -c "from live_api_client import VisionSession, analyze; print('Import OK')"
```

**Test API Connection:**
```bash
cd frontend/capture_demo
python3 verify_live_api.py --help
```

**Test Full Round-trip:**
```bash
cd frontend/capture_demo
python3 verify_live_api.py
```

**❌ Expected Issues:**
- `ModuleNotFoundError: No module named 'google'` → Install dependencies
- `GEMINI_API_KEY not set` → Set API key
- `Image file not found` → Make sure sample.png exists
- `Empty response from Gemini API` → Check API key validity

### 5. PySide6 Game Widget

**Test Import:**
```bash
cd frontend
python3 -c "import wasd_stream; print('Import OK')"
```

**Test GUI (without showing):**
```bash
cd frontend
python3 -c "
from PySide6.QtWidgets import QApplication
import wasd_stream
app = QApplication([])
main = wasd_stream.Main()
print('GUI creation OK')
"
```

**Test Full Application:**
```bash
cd frontend
python3 wasd_stream.py
# Should show window with game on left, preview on right
# Press WASD to move red square
```

**❌ Expected Issues:**
- `ModuleNotFoundError: No module named 'PySide6'` → Install PySide6
- Window doesn't show → Check display/X11 setup
- Keys don't work → Click on the window to focus it

### 6. Live-API Web Console

**Test Submodule:**
```bash
ls frontend/live_console/
# Should show package.json and other files
```

**Test Build:**
```bash
ls frontend/live_console/build/
# Should show index.html and static/ folder
```

**❌ If build/ missing:**
```bash
cd frontend/live_console
npm install
npm run build
```

**❌ If submodule missing:**
```bash
git submodule update --init --recursive
```

## Integration Testing

### Test 1: Live API Round-trip
```bash
cd frontend/capture_demo
python3 verify_live_api.py --image sample.png --prompt "What do you see?"
```

**Expected Output:**
```
### Gemini replied in 1234.5 ms ###

This image shows geometric shapes including a blue rectangle, red circle, and green triangle...

✅ Live API verification successful!
```

### Test 2: PySide6 Game Widget
```bash
cd frontend
python3 wasd_stream.py
```

**Expected Behavior:**
- Window opens with 800x400 size
- Left side: Interactive game with blue/red squares
- Right side: Live preview of the game
- WASD keys move the red square
- Preview updates in real-time

### Test 3: Live-API Web Console
```bash
cd frontend/live_console
python3 -m http.server 8000
# Open browser to http://localhost:8000/build/
```

**Expected Behavior:**
- Web page loads with Google Live API interface
- Can set API key in the interface
- Can start voice/video chat with Gemini

## Common Issues & Solutions

### "ModuleNotFoundError" Issues
**Cause:** Missing Python dependencies
**Solution:** 
```bash
pip install PySide6>=6.5.0 google-generativeai>=0.5.0 pillow>=10.0.0
```

### "GEMINI_API_KEY not set" Issues
**Cause:** API key not configured
**Solution:**
```bash
export GEMINI_API_KEY='your-api-key-here'
# Make permanent:
echo 'export GEMINI_API_KEY="your-key"' >> ~/.bashrc
source ~/.bashrc
```

### "npm command not found" Issues
**Cause:** Node.js not installed
**Solution:**
- Install Node.js from https://nodejs.org/
- Or use package manager: `apt install nodejs npm` (Ubuntu) or `brew install node` (Mac)

### "git submodule" Issues
**Cause:** Live-API console submodule not initialized
**Solution:**
```bash
git submodule update --init --recursive
cd frontend/live_console
npm install
npm run build
```

### GUI Won't Show Issues
**Cause:** Display/X11 setup problems
**Solution:**
- Linux: Install X11 packages, set DISPLAY variable
- Mac: Install XQuartz if needed
- Windows: Should work out of the box

## Architecture Overview

```
RoboWeave/
├── frontend/
│   ├── wasd_stream.py          # PySide6 game widget
│   ├── capture_demo/           # Live API integration
│   │   ├── live_api_client.py  # Google Gemini API client
│   │   ├── verify_live_api.py  # Smoke test script
│   │   └── sample.png          # Test image
│   └── live_console/           # Google Live-API Web Console
│       └── build/              # Built web interface
├── quick_test.py               # Quick diagnostic tool
└── test_setup.py               # Comprehensive test suite
```

## Next Steps After Setup

1. **If all tests pass:**
   - Your setup is complete!
   - Try the PySide6 game: `python frontend/wasd_stream.py`
   - Try the Live API: `python frontend/capture_demo/verify_live_api.py`
   - Try the Web Console: Serve `frontend/live_console/build/`

2. **If tests fail:**
   - Follow the specific error messages
   - Fix dependencies one by one
   - Re-run tests after each fix
   - Ask for help with specific error messages

3. **For development:**
   - All components are independent
   - Modify `wasd_stream.py` for UI changes
   - Modify `live_api_client.py` for API changes
   - Use Web Console for interactive testing

## Getting Help

When asking for help, please include:
1. Output from `python quick_test.py`
2. Your operating system
3. Python version (`python3 --version`)
4. Specific error messages
5. What you were trying to do

This will help identify the exact issue and provide targeted solutions. 