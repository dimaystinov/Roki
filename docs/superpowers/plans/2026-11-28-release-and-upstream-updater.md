# FPV Roki release and isolated upstream updater

User approved: publish current macOS/Windows builds in dimaystinov/Roki; updater installs only separate aleksey-yagubov/roki-ng-operator-gui. Never overwrite customized Roki or update robot firmware.

Design: bundle a separate updater launcher using the packaged Python/Qt runtime. Store pinned upstream snapshots and independent user settings in a user-owned directory. Fetch default branch SHA via GitHub API, download SHA archive, reject traversal/symlinks/oversized archives. Validate source structure, compile Python, run an isolated GUI smoke test. Activate atomic current pointer only after success; keep previous pointer for rollback. External source finder must not silently fall back to packaged operator modules. Inject only the platform docking adapter from Roki. Unsupported upstream dependencies fail explicitly; no silent pip upgrade of bundled Qt. Upstream GUI/runtime compatibility remains operator responsibility; updater never connects to a robot.

1. Tests: archive traversal, pinning, failed validation leaves current intact, rollback, source import isolation.
2. Implement updater core, minimal Qt dialog, external-source runner and separate macOS/Windows launchers.
3. Fix/run current GUI and robot-runtime regressions, test launchers; isolate test config and no robot connection.
4. Build and smoke-test macOS packaged app, include updater launchers. Windows Actions builds matching commit and executes smoke tests.
5. Commit reviewed source changes (exclude .deps symlink), publish through GitHub connector where supported; release assets via supported fallback if upload tools unavailable. Verify remote commit, artifact hashes, release URLs. Report any platform not actually tested.
