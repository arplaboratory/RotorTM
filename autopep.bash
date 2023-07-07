#!/bin/bash

# Directory path to run autopep8
directory_path="/Users/mrunalsarvaiya/Documents/NYU/RotorTM"

# Run autopep8 on all Python files in the directory
# find "$directory_path" -type f -name "*.py" -exec autopep8 --global-config /asd/asd {} \;
autopep8 --global-config /Users/mrunalsarvaiya/Documents/NYU/RotorTM/pyproject.toml 