#!/usr/bin/env python3
"""
Quick test script to identify immediate issues and provide next steps.
This is a simplified version for rapid troubleshooting.
"""

import os
import sys
import subprocess
from pathlib import Path

def test_command(cmd, description):
    """Test if a command can be run successfully."""
    print(f"Testing: {description}")
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✅ {description} - OK")
            return True
        else:
            print(f"❌ {description} - Failed")
            print(f"   Error: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print(f"⏱️  {description} - Timeout")
        return False
    except Exception as e:
        print(f"❌ {description} - Error: {e}")
        return False

def check_basics():
    """Check basic requirements."""
    print("\n=== BASIC CHECKS ===")
    
    issues = []
    
    # Check Python
    if not test_command("python3 --version", "Python 3 available"):
        issues.append("Install Python 3.9+")
    
    # Check pip
    if not test_command("pip3 --version", "pip3 available"):
        issues.append("Install pip3")
    
    # Check npm (for Live-API console)
    if not test_command("npm --version", "npm available"):
        issues.append("Install Node.js and npm")
    
    # Check git
    if not test_command("git --version", "git available"):
        issues.append("Install git")
    
    return issues

def check_api_key():
    """Check API key setup."""
    print("\n=== API KEY CHECK ===")
    
    api_key = os.getenv('GEMINI_API_KEY')
    if not api_key:
        print("❌ GEMINI_API_KEY not set")
        return ["Set GEMINI_API_KEY environment variable"]
    
    if len(api_key) < 20:
        print("❌ GEMINI_API_KEY seems too short")
        return ["Check GEMINI_API_KEY - get from https://makersuite.google.com/app/apikey"]
    
    print("✅ GEMINI_API_KEY is set")
    return []

def check_dependencies():
    """Check Python dependencies."""
    print("\n=== DEPENDENCY CHECK ===")
    
    issues = []
    
    # Test imports
    deps = ['PySide6', 'google.generativeai', 'PIL']
    for dep in deps:
        try:
            if dep == 'PIL':
                import PIL
            else:
                __import__(dep)
            print(f"✅ {dep} installed")
        except ImportError:
            print(f"❌ {dep} missing")
            issues.append(f"pip install {dep}")
    
    return issues

def check_files():
    """Check required files exist."""
    print("\n=== FILE CHECK ===")
    
    issues = []
    required_files = [
        'frontend/wasd_stream.py',
        'frontend/capture_demo/live_api_client.py',
        'frontend/capture_demo/verify_live_api.py',
        'frontend/capture_demo/sample.png',
        'frontend/live_console/package.json'
    ]
    
    for file_path in required_files:
        if Path(file_path).exists():
            print(f"✅ {file_path}")
        else:
            print(f"❌ {file_path} missing")
            issues.append(f"Create or restore {file_path}")
    
    return issues

def test_live_api():
    """Test Live API functionality."""
    print("\n=== LIVE API TEST ===")
    
    if not Path('frontend/capture_demo/verify_live_api.py').exists():
        print("❌ verify_live_api.py missing")
        return ["Create verify_live_api.py"]
    
    # Try to run the verification script
    cmd = "cd frontend/capture_demo && python3 verify_live_api.py --help"
    if test_command(cmd, "Live API script help"):
        print("✅ Live API script can run")
        return []
    else:
        return ["Fix Live API script dependencies"]

def test_pyside():
    """Test PySide6 functionality."""
    print("\n=== PYSIDE6 TEST ===")
    
    if not Path('frontend/wasd_stream.py').exists():
        print("❌ wasd_stream.py missing")
        return ["Create wasd_stream.py"]
    
    # Try to import without running GUI
    cmd = "cd frontend && python3 -c 'import wasd_stream; print(\"Import OK\")'"
    if test_command(cmd, "PySide6 app import"):
        print("✅ PySide6 app can import")
        return []
    else:
        return ["Fix PySide6 app dependencies"]

def main():
    """Run all quick tests."""
    print("🚀 QUICK SETUP TEST")
    print("=" * 50)
    
    all_issues = []
    
    # Run all checks
    all_issues.extend(check_basics())
    all_issues.extend(check_api_key())
    all_issues.extend(check_dependencies())
    all_issues.extend(check_files())
    all_issues.extend(test_live_api())
    all_issues.extend(test_pyside())
    
    # Summary
    print(f"\n📊 SUMMARY")
    print("=" * 20)
    
    if not all_issues:
        print("🎉 All quick tests passed!")
        print("\nNext steps:")
        print("1. Run full test: python test_setup.py")
        print("2. Test Live API: python frontend/capture_demo/verify_live_api.py")
        print("3. Test PySide6: python frontend/wasd_stream.py")
        return True
    else:
        print(f"❌ Found {len(all_issues)} issues:")
        for i, issue in enumerate(all_issues, 1):
            print(f"{i}. {issue}")
        
        print("\nFIX THESE ISSUES:")
        print("1. Install missing dependencies:")
        print("   pip install PySide6>=6.5.0 google-generativeai>=0.5.0 pillow>=10.0.0")
        print("\n2. Set API key:")
        print("   export GEMINI_API_KEY='your-api-key-here'")
        print("\n3. Build Live-API console:")
        print("   cd frontend/live_console && npm install && npm run build")
        print("\n4. Run again: python quick_test.py")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 