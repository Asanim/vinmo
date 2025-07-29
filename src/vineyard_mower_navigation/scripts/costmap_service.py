#!/usr/bin/env python3
"""
Costmap Service executable wrapper
"""
import sys
import os

# Add the package to Python path
package_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
sys.path.insert(0, os.path.join(package_path, 'lib', 'python3.12', 'site-packages'))

try:
    from vineyard_mower_navigation.costmap_service import main
    if __name__ == '__main__':
        main()
except ImportError as e:
    print(f"Import error: {e}")
    print("Trying to run module directly...")
    import subprocess
    import sys
    
    # Try to run the module directly
    module_path = os.path.join(os.path.dirname(__file__), '..', 'vineyard_mower_navigation', 'costmap_service.py')
    if os.path.exists(module_path):
        sys.exit(subprocess.call([sys.executable, module_path] + sys.argv[1:]))
    else:
        print(f"Could not find module at {module_path}")
        sys.exit(1)
