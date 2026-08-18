"""Support ``python -m pace_bench.cli`` in addition to ``pace-bench``."""

from pace_bench.cli.main import main

raise SystemExit(main())
