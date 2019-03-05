# Avoid pytest path issues with relative path imports by adding the current path manually
import sys, os
myPath = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, myPath + '/../')