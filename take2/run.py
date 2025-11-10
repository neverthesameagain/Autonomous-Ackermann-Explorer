#!/usr/bin/env python3
"""
Main entry point for the Autonomous Ackermann Explorer.
"""
import sys
import os

# Add the src directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

def main():
    """Run the goal-directed explorer."""
    try:
        from goal_directed_explorer import main as run_explorer
        run_explorer()
    except ImportError as e:
        print(f"Error: {e}")
        print("Please make sure you have installed all the required dependencies.")
        print("Run: pip install -r requirements.txt")
        sys.exit(1)

if __name__ == "__main__":
    main()
