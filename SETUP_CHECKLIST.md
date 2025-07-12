# RoboWeave Setup Checklist

This checklist helps external software agents guide users through the complete RoboWeave setup process.

## 🔧 Pre-Setup Diagnostic

**Run this first to identify all issues:**
```bash
python3 quick_test.py
```

**Expected Output Analysis:**
- ✅ = Component working correctly
- ❌ = Component needs fixing
- ⏱️ = Timeout (usually dependency issue)

## 📋 Step-by-Step Setup

### Step 1: Install System Dependencies
```bash
# Check what's missing:
python3 --version   # Need 3.9+
pip3 --version      # Need pip
npm --version       # Need Node.js
git --version       # Need git

# Install missing tools based on OS:
# Ubuntu/Debian: sudo apt install python3 python3-pip nodejs npm git
# macOS: brew install python3 node git
# Windows: Install from official sites
```

### Step 2: Install Python Dependencies
```bash
pip3 install PySide6>=6.5.0 google-generativeai>=0.5.0 pillow>=10.0.0
```

### Step 3: Set API Key
```bash
# Get API key from: https://makersuite.google.com/app/apikey
export GEMINI_API_KEY='your-api-key-here'

# Make permanent:
echo 'export GEMINI_API_KEY="your-key"' >> ~/.bashrc
source ~/.bashrc
```

### Step 4: Build Live-API Console
```bash
cd frontend/live_console
npm install
npm run build
cd ../..
```

### Step 5: Verify Setup
```bash
python3 quick_test.py
# Should show all ✅ checkmarks
```

## 🧪 Component Testing

### Test 1: Live API Client
```bash
cd frontend/capture_demo
python3 verify_live_api.py
```

**Success Indicators:**
- Shows "### Gemini replied in X ms ###"
- Displays image analysis
- Shows "✅ Live API verification successful!"

### Test 2: PySide6 Game Widget
```bash
cd frontend
python3 wasd_stream.py
```

**Success Indicators:**
- Window opens (800x400)
- Left: Blue/red squares
- Right: Live preview
- WASD keys move red square

### Test 3: Live-API Web Console
```bash
cd frontend/live_console
python3 -m http.server 8000
# Open: http://localhost:8000/build/
```

**Success Indicators:**
- Web page loads
- Can enter API key
- Can start voice chat

## 🚨 Common Issues & Solutions

### Issue: "ModuleNotFoundError: No module named 'google'"
**Solution:** `pip3 install google-generativeai>=0.5.0`

### Issue: "GEMINI_API_KEY not set"
**Solution:** 
```bash
export GEMINI_API_KEY='your-key'
echo 'export GEMINI_API_KEY="your-key"' >> ~/.bashrc
```

### Issue: "npm command not found"
**Solution:** Install Node.js from https://nodejs.org/

### Issue: "frontend/live_console missing"
**Solution:** 
```bash
git submodule update --init --recursive
cd frontend/live_console
npm install && npm run build
```

### Issue: PySide6 GUI won't show
**Solution:** 
- Linux: Install X11 packages
- Mac: Install XQuartz if needed
- Windows: Should work by default

## 🎯 Agent Instructions

When helping users, follow this pattern:

1. **Run Diagnostic:** `python3 quick_test.py`
2. **Identify Issues:** Count ❌ marks and specific error messages
3. **Prioritize Fixes:** System deps → Python deps → API key → Build steps
4. **Fix One at a Time:** Don't fix everything at once
5. **Re-test:** Run `python3 quick_test.py` after each fix
6. **Verify Components:** Test each component individually

## 📊 Success Criteria

**Setup Complete When:**
- `python3 quick_test.py` shows all ✅
- `python3 frontend/capture_demo/verify_live_api.py` works
- `python3 frontend/wasd_stream.py` shows GUI
- Web console serves at `frontend/live_console/build/`

## 🔍 Debugging Commands

**For detailed diagnostics:**
```bash
python3 test_setup.py  # Comprehensive test
```

**For specific component issues:**
```bash
# Test API connection:
python3 -c "import google.generativeai; print('API import OK')"

# Test GUI without showing:
python3 -c "from PySide6.QtWidgets import QApplication; print('GUI OK')"

# Test Live API client:
cd frontend/capture_demo
python3 -c "from live_api_client import analyze; print('Client OK')"
```

## 📁 Repository Structure

```
RoboWeave/
├── quick_test.py           # Quick diagnostic tool
├── test_setup.py           # Comprehensive test suite
├── TESTING_GUIDE.md        # Detailed testing instructions
├── SETUP_CHECKLIST.md      # This file
└── frontend/
    ├── wasd_stream.py      # PySide6 game widget
    ├── capture_demo/       # Live API integration
    │   ├── live_api_client.py
    │   ├── verify_live_api.py
    │   └── sample.png
    └── live_console/       # Google Live-API Web Console
        └── build/          # Built web interface
```

This checklist ensures systematic setup and troubleshooting for the complete RoboWeave system. 