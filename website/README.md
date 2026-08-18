# PACE-Bench website

Static project site for `https://thunlp.github.io/PACE-Bench/`.

The site intentionally has no framework or runtime dependency. All asset paths are
relative so the same files work locally and under the GitHub Pages `/PACE-Bench/`
project path.

## Preview locally

From the repository root:

```bash
python -m http.server 4173 --directory website
```

Then open `http://localhost:4173`.

## Content map

- `index.html`: compact story from leaderboard to benchmark, findings, and tasks
- `styles.css`: responsive Arial-based visual system
- `app.js`: protocol demo, leaderboard, and task gallery interactions
- `data/leaderboard.js`: values transcribed from the paper's main-results table;
  plain script format keeps the interactive table working when `index.html` is
  opened directly from disk
- `assets/tasks/`: verified Initial-environment simulation GIFs and first-frame posters
- `assets/demo/`: S-01 Initial/Stage-3 protocol walkthrough
- `assets/insights/`: result and analysis figures exported from the paper

## Deploy

The repository workflow `.github/workflows/deploy-website.yml` uploads this directory
as a GitHub Pages artifact. In the target repository, select **Settings → Pages →
Source → GitHub Actions** once. Pushes to `main` then deploy automatically.
