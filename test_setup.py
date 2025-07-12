#!/usr/bin/env python3
"""
Comprehensive testing script to identify missing components and guide setup.
This script will check all dependencies, API keys, and component functionality.
"""

import os
import sys
import subprocess
import importlib
from pathlib import Path
import json

class SetupTester:
    def __init__(self):
        self.issues = []
        self.successes = []
        self.root_dir = Path(__file__).parent
        
    def log_success(self, message):
        self.successes.append(f"✅ {message}")
        print(f"✅ {message}")
        
    def log_issue(self, message, fix=None):
        issue = f"❌ {message}"
        if fix:
            issue += f"\n   Fix: {fix}"
        self.issues.append(issue)
        print(issue)
        
    def check_python_version(self):
        """Check Python version compatibility."""
        print("\n=== Python Version Check ===")
        version = sys.version_info
        if version >= (3, 9):
            self.log_success(f"Python {version.major}.{version.minor}.{version.micro} (compatible)")
        else:
            self.log_issue(f"Python {version.major}.{version.minor}.{version.micro} (need 3.9+)", 
                          "Upgrade to Python 3.9 or higher")
    
    def check_dependencies(self):
        """Check if required Python packages are installed."""
        print("\n=== Dependency Check ===")
        
        # Core dependencies
        deps = {
            'PySide6': 'pip install PySide6>=6.5.0',
            'google.generativeai': 'pip install google-generativeai>=0.5.0',
            'PIL': 'pip install pillow>=10.0.0'
        }
        
        for module, install_cmd in deps.items():
            try:
                if module == 'PIL':
                    importlib.import_module('PIL')
                else:
                    importlib.import_module(module)
                self.log_success(f"{module} installed")
            except ImportError:
                self.log_issue(f"{module} missing", install_cmd)
    
    def check_api_key(self):
        """Check if GEMINI_API_KEY is set."""
        print("\n=== API Key Check ===")
        api_key = os.getenv('GEMINI_API_KEY')
        if api_key:
            if len(api_key) > 20:  # Basic length check
                self.log_success("GEMINI_API_KEY is set and appears valid")
            else:
                self.log_issue("GEMINI_API_KEY seems too short", 
                              "Get a valid API key from https://makersuite.google.com/app/apikey")
        else:
            self.log_issue("GEMINI_API_KEY not set", 
                          "export GEMINI_API_KEY='your-api-key-here'")
    
    def check_files(self):
        """Check if required files exist."""
        print("\n=== File Structure Check ===")
        
        required_files = [
            'frontend/wasd_stream.py',
            'frontend/capture_demo/live_api_client.py',
            'frontend/capture_demo/verify_live_api.py',
            'frontend/capture_demo/sample.png',
            'frontend/capture_demo/requirements.txt',
            'frontend/live_console/build/index.html'
        ]
        
        for file_path in required_files:
            full_path = self.root_dir / file_path
            if full_path.exists():
                self.log_success(f"Found: {file_path}")
            else:
                self.log_issue(f"Missing: {file_path}", 
                              f"File should exist at {full_path}")
    
    def test_live_api_client(self):
        """Test the Live API client functionality."""
        print("\n=== Live API Client Test ===")
        
        try:
            # Change to the capture_demo directory
            os.chdir(self.root_dir / 'frontend' / 'capture_demo')
            
            # Try to import the client
            sys.path.insert(0, str(Path.cwd()))
            from live_api_client import VisionSession, analyze
            self.log_success("Live API client imports successfully")
            
            # Test API key validation
            if os.getenv('GEMINI_API_KEY'):
                try:
                    session = VisionSession()
                    self.log_success("VisionSession created successfully")
                except Exception as e:
                    self.log_issue(f"VisionSession creation failed: {e}", 
                                  "Check API key validity")
            else:
                self.log_issue("Cannot test VisionSession without API key", 
                              "Set GEMINI_API_KEY first")
                
        except ImportError as e:
            self.log_issue(f"Cannot import live_api_client: {e}", 
                          "Check if all dependencies are installed")
        except Exception as e:
            self.log_issue(f"Live API client test failed: {e}")
        finally:
            # Restore original directory
            os.chdir(self.root_dir)
    
    def test_pyside_app(self):
        """Test PySide6 application."""
        print("\n=== PySide6 App Test ===")
        
        try:
            # Try to import PySide6
            from PySide6.QtWidgets import QApplication
            self.log_success("PySide6 imports successfully")
            
            # Test app creation (without showing)
            app = QApplication.instance()
            if app is None:
                app = QApplication([])
            self.log_success("QApplication can be created")
            
            # Test wasd_stream import
            sys.path.insert(0, str(self.root_dir / 'frontend'))
            try:
                import wasd_stream
                self.log_success("wasd_stream.py imports successfully")
            except ImportError as e:
                self.log_issue(f"Cannot import wasd_stream: {e}")
                
        except ImportError as e:
            self.log_issue(f"PySide6 import failed: {e}", 
                          "pip install PySide6>=6.5.0")
    
    def test_live_console(self):
        """Test Live-API Web Console."""
        print("\n=== Live-API Web Console Test ===")
        
        console_path = self.root_dir / 'frontend' / 'live_console'
        build_path = console_path / 'build'
        
        if not console_path.exists():
            self.log_issue("live_console directory missing", 
                          "Git submodule may not be initialized")
            return
            
        if not build_path.exists():
            self.log_issue("live_console/build directory missing", 
                          "Run: cd frontend/live_console && npm install && npm run build")
            return
            
        # Check key files
        key_files = ['index.html', 'static']
        for file_name in key_files:
            file_path = build_path / file_name
            if file_path.exists():
                self.log_success(f"Live console: {file_name} exists")
            else:
                self.log_issue(f"Live console: {file_name} missing", 
                              "Rebuild the console: npm run build")
    
    def create_setup_guide(self):
        """Create a setup guide based on found issues."""
        print("\n" + "="*50)
        print("SETUP GUIDE")
        print("="*50)
        
        if not self.issues:
            print("🎉 Everything looks good! No issues found.")
            return
            
        print(f"Found {len(self.issues)} issues to fix:\n")
        
        for i, issue in enumerate(self.issues, 1):
            print(f"{i}. {issue}\n")
        
        print("RECOMMENDED SETUP STEPS:")
        print("=" * 25)
        print("1. Install dependencies:")
        print("   pip install PySide6>=6.5.0 google-generativeai>=0.5.0 pillow>=10.0.0")
        print("\n2. Set API key:")
        print("   export GEMINI_API_KEY='your-api-key-here'")
        print("\n3. Build Live-API Web Console:")
        print("   cd frontend/live_console")
        print("   npm install")
        print("   npm run build")
        print("\n4. Test components:")
        print("   python frontend/capture_demo/verify_live_api.py")
        print("   python frontend/wasd_stream.py")
        
    def run_all_tests(self):
        """Run all tests and provide comprehensive report."""
        print("🚀 Starting comprehensive setup test...")
        
        self.check_python_version()
        self.check_dependencies()
        self.check_api_key()
        self.check_files()
        self.test_live_api_client()
        self.test_pyside_app()
        self.test_live_console()
        
        self.create_setup_guide()
        
        print(f"\n📊 SUMMARY: {len(self.successes)} successes, {len(self.issues)} issues")
        return len(self.issues) == 0


if __name__ == "__main__":
    tester = SetupTester()
    success = tester.run_all_tests()
    
    if success:
        print("\n✅ All tests passed! Your setup is ready.")
        sys.exit(0)
    else:
        print("\n❌ Some issues found. Please fix them and run again.")
        sys.exit(1) 