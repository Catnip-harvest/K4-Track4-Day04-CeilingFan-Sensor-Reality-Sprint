#!/usr/bin/env sh
set -eu

if command -v python3 >/dev/null 2>&1; then
    PYTHON=python3
else
    PYTHON=python
fi

"$PYTHON" get_data.py --drives 0048 0005 0052
"$PYTHON" run_benchmark.py
"$PYTHON" validate_holdout.py --drives 0052 --label holdout
"$PYTHON" demo.py
