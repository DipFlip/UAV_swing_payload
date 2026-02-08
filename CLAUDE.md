# Claude Code Instructions

## Workflow

- After completing any request that changes the simulation code, automatically commit and push to `origin main` without asking. This triggers the GitHub Pages deployment.
- Always increment the version number in `frontend/index.html` when pushing changes. The version appears in three places:
  1. `style.css?v=X.Y` (cache bust)
  2. `main.js?v=X.Y` (cache bust)
  3. `<div id="hud-version">vX.Y</div>` (visible in HUD)
