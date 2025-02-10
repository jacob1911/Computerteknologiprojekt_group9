import os

# Gets the current path
cur_dir = os.path.dirname(os.path.abspath(__file__))

# Joins paths
path = os.path.join(cur_dir, "Testfolder")

path = os.path.join(path, "joketime.txt")

# Creates file - x makes sure it doesn't already exist
f = open(path, "x")
